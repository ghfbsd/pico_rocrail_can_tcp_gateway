# Marklin packet sniffer for MS1/MS2/CS1/CS2 CAN protocol.  Reference:
#  Kommunikationsprotokoll Graphical User Interface Prozessor (GUI) <->
#  Gleisformat Prozessor (GFP) über CAN transportierbar über Ethernet
#  (Version vom 7.2.2012)

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)
#          or Pico-CAN-B board by Waveshare (https://www.waveshare.com)

# original version 15 Jan. '25
# last revision 23 Feb. '26

_VER = 'EB236'                    # version ID

CS2_SIZE = const(13)              # Fixed by protocol definition

QSIZE = const(500)                # Size of various I/O queues (overkill)

_CANBOARD = const('auto')         # Board choice: 'auto' for auto-detect or
                                  # 'JI' or 'WS'

INSTR_READ_STATUS = const(0xA0)   # PIO for fast reading of 250 kbps CAN bus
INSTR_WRITE       = const(0x02)
INSTR_READ        = const(0x03)
INSTR_RXBUF0      = const(0x90)
INSTR_RXBUF1      = const(0x94)
INSTR_BIT_MOD     = const(0x05)

MCP_CANINTF       = const(0x2C)
MCP_EFLG          = const(0x2D)

import uasyncio as asyncio
from threadsafe import ThreadSafeQueue
from machine import Pin
import sys, utime
from micropython import schedule
from rp2 import PIO, StateMachine
from array import array

from micropython import alloc_emergency_exception_buf as AEEB
AEEB(100)                         # boilerplate: IRQ-level exception reporting

print('CS1/CS2 protocol packet sniffer (%s)' % _VER)
try:
   from marklin import decode, CS2decoder     # Marklin CS2 CAN packet decoder
except:
   print('No Märklin packet decoding, only logging raw data.')
   # Dummy decoder if not available
   class CS2decoder:
      def __init__(self,*pos,**kwd):
         pass
      def decode(self,*pos,**kwd):
         pass
dec = CS2decoder(pfx='    ',detail=True,print=True)

def timed_function(f, *args, **kwargs):
    myname = str(f).split(' ')[1]
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        delta = utime.ticks_diff(utime.ticks_us(), t)
        print('Function {} Time = {:6.3f}ms'.format(myname, delta/1000))
        return result
    return new_func

@rp2.asm_pio(
   push_thresh=32, autopush=True,
   sideset_init=(rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW),   #CS, SCK
   out_init=rp2.PIO.OUT_LOW
)
def SPI_OP():
    wrap_target()
    pull()                   .side(0x1)         #CS=1, SCK=0
    out(y, 8)                .side(0x1)         #CS=1, SCK=0; y=byte count - 1
    label("byteloop")
    set(x, 7)
    label("bloop1")
    out(pins, 1)             .side(0x0)   [1]   #CS=0, SCK=0
    in_(pins, 1)             .side(0x2)         #CS=0, SCK=1
    jmp(x_dec, "bloop1")     .side(0x2)         #CS=0, SCK=1
    jmp(y_dec, "byteloop")

    push(noblock)            .side(0x1)   [1]   #CS=1, SCK=0; last unaligned
    wrap()

class iCAN:                       # interrupt driven CAN message sniffer
   from collections import namedtuple

   CQSIZE = 250                   # Big to read CONFIG DATA bursts

   CAN_pins = namedtuple('CAN pins',
      ['INT_PIN','SPI_CS', 'SPI_SCK', 'SPI_MOSI', 'SPI_MISO', 'name']
   )
   pins_JI = CAN_pins(
      name = 'Joy-IT',            # These pin assignments are appropriate for
                                  # a RB-P-CAN-485 Joy-IT board
      INT_PIN = 20,               # Interrupt pin for CAN board
      SPI_CS = 17,
      SPI_SCK = 18,
      SPI_MOSI = 19,
      SPI_MISO = 16
   )
   pins_WS = CAN_pins(
      name = 'Waveshare',         # These pin assignments are appropriate for
                                  # a Waveshare Pico-CAN-B board
      INT_PIN = 21,               # Interrupt pin for CAN board
      SPI_CS = 5,
      SPI_SCK = 6,
      SPI_MOSI = 7,
      SPI_MISO = 4
   )
   boards = dict(
      WS = pins_WS,
      JI = pins_JI
   )

   def __init__(self, conf=None):
      from machine import Pin, SPI
      from canbus import Can, CanError, CanMsg
      from canbus.internal import CAN_SPEED, ERROR

      if conf is None:
         raise RuntimeError("Need to provide CAN board type or 'auto'")
      if conf == 'auto':
         for bd in iCAN.boards:
            # Check if the initialization is successful
            pin = iCAN.boards[bd]
            self.spi = SPI(0,    # configure SPI to use this board's pins
                sck=Pin(pin.SPI_SCK),
                mosi=Pin(pin.SPI_MOSI),
                miso=Pin(pin.SPI_MISO)
            )
            # Create an instance of the Can class to interface with the CAN bus
            self.can = Can(spics=pin.SPI_CS)
            if self.can.begin() == CanError.ERROR_OK:
               break
         else:
            raise RuntimeError(
               "***Can't auto-detect board; set explicit board name***"
            )
      elif conf in iCAN.boards:   # Explicit board choice?
         pin = iCAN.boards[conf]
         self.spi = SPI(0,        # configure SPI to use this board's pins
             sck=Pin(pin.SPI_SCK),
             mosi=Pin(pin.SPI_MOSI),
             miso=Pin(pin.SPI_MISO)
         )
         # Create an instance of the Can class to interface with the CAN bus
         self.can = Can(spics=pin.SPI_CS)
         if self.can.begin() != CanError.ERROR_OK:
            raise RuntimeError("Error initializing %s CAN board - check type!" %
               pin.name
            )
      else:
         raise RuntimeError('***%s is an unsupported CAN board***' % conf)

      self.pin = Pin(pin.INT_PIN,Pin.IN,Pin.PULL_UP)
      self.can = Can(spics=pin.SPI_CS) # CS pin for hardware SPI 0
      # Initialize the CAN interface.  Reference says 250 kbps speed.
      ret = self.can.begin(bitrate=CAN_SPEED.CAN_250KBPS)
      if ret != CanError.ERROR_OK:
         raise RuntimeError('Error initializing CAN bus')
      print("Initialized successfully, waiting for traffic.")
      self.SPI_CS = Pin(pin.SPI_CS, Pin.OUT)

      self.sm = rp2.StateMachine(0)
      self.sm.init(SPI_OP,
            freq=32_000_000,      # WARNING: 32 MHz max rate before bit loss 
            in_base=pin.SPI_MISO,
            out_base=pin.SPI_MOSI,
            sideset_base=pin.SPI_CS,
            in_shiftdir=PIO.SHIFT_LEFT,
            out_shiftdir=PIO.SHIFT_LEFT
      )
      self.sm.active(1)

      self.cque = ThreadSafeQueue(iCAN.CQSIZE)
      self.melem = iCAN.CQSIZE*[None]
      for n in range(iCAN.CQSIZE):
         self.melem[n] = array('I',4*[0])
      self.eelem = bytearray(iCAN.CQSIZE)
      self.nque = 0
      self.stamp = 0
      self.sflag = 0
      self.stats = [0,0,0,0,0,0]  # timing statistics
      _intr_ref = self._intr
      self._intr_ref = _intr_ref
      self.pin.irq(
         trigger=Pin.IRQ_FALLING,
         handler=self._intr_ref,
      #  handler=lambda pin: schedule(_intr_ref, pin), # 200 us delay - too long
         hard=True)

# Notes on how this IRQ routine works.

# For speed reasons, the PIO machine does the SPI interactions with the MCP2515.
# Calling SPI functions directly from the code is too slow for a CAN bus
# running at 250 kbps.  Decoders talking over the rails speak slowly, and code
# can keep up with the resuting CAN packet traffic.  However, talk between
# multiple MS2s on a CAN bus, or a CS2 and an MS2 connected by a CAN bus runs
# at full speed, and micropython code cannot keep up with the blistering output
# rate.

# Timing of the basic interrupt - micropython SPI read takes about 2-3 ms.  At
# 250 kbps, packets on a CAN bus running at full speed arrive every 256 - 512 us
# depending on packet size.  Hence to keep up, drastic optimization is needed.

# Consequently, this routine runs at the IRQ level.  It can be run by the
# micropython schedule() command to avoid heap corruption; this adds a delay of
# ~200 us per IRQ.  It mightly tries to avoid using the heap, so many local
# vars and constants are allocated as static vars at the time the function is
# instantiated (in init() of the iCAN object).
# Bravely running from an IRQ directly does not seem to lead to any problems.
# Timing shows one IRQ takes ~527 us to process to quiescence.

# For speed optimization, PIO to the rescue.  A PIO state machine can run an
# SPI device fairly efficiently.  With a clock speed of 32 MHz and using 4
# cycles/bit, the PIO can read at 8 Mbps, which is 32x faster than the CAN
# bus bit rate.  However, the micropython code has to be highly optimized to
# respond quickly enough.  If there is 200 us latency to reach a safe place for
# an IRQ to run, that leaves only 60 - 300 us for the IRQ to issue commands to
# read the CAN bus.  Consequently, schedule() is not used.

# The PIO machine takes a 32 bit word as input on its input FIFO, and outputs a
# series of one or more 32 bit words (using PIO autopush, and an explicit push
# at the end) to its output FIFO.  Due to the way the MCP2515 SPI works, the PIO
# reads as it writes: even for writes, data is pushed onto the output FIFO which
# has to be cleared for the next use.  Each command is encoded with a byte
# count (-1 for how PIO looping works), a command, an (optional) address, and
# (optional) data (for writes).  Each byte written to the SPI bus has a
# corresponding read.  Each byte read is shifted into the 32 bit input register
# from the right, so 1 byte will be in the lowest 8 bits, 2 bytes in the lowest
# 16 bits, etc.  For most commands, this yields exactly one output fifo word.
# For the BIT_MODIFY and READ_RXn commands, multiple words are the result.
# The .get() method expects a buffer argument, which VERY CONVENIENTLY means
# that you get all the FIFO's words in one call - no looping needed.  Your
# buffer has to be big enough, however.  If the number of bytes read is NOT
# a multiple of 4, the last 32 bit word read will have leading zeroes in it
# (incompletely shifted).  The gap in the bytes is handled specially
# when processing the READ_RXn command output, because 13 data bytes are read.
# Adding in the 2 bytes for the MCP2515 command and address, this means there
# will be a 1 byte gap in the high bits of the last 32 bit word read.

# The PIO state machine's .get() truncates the 32 bit FIFO word to the type of
# the output buffer.  Hence, a bytearray will receive the low order byte in each
# 32 bit word, and a full 32 bit integer array will receive the entire word.

# If we are unlucky, multiple IRQ interrupts will happen as each MCP2515 of the
# two receive buffers fills up.  There is a lock-out flag to prevent the IRQ
# from being called again while inside itself.  The IRQ code checks for new data
# once it gets an interrupt and keeps reading until it runs to quiescence.
 
# Empirical observations of how the MCP2515 works:

# - You can't read the CANINTF and subsequent registers as a group; only
#   CANINTF singly (would be nice to read CANINTF and EFLG as a pair - oh well).

# - When you do a read of RX0 or RX1, the corresponding CANINTF status bit seems
#   to be automatically cleared, readying a detection of a new CAN packet. A
#   useful (undocumented) feature.

# - READ_RXBn is FAST: timing says ~40 us to read 16 bytes!  By comparison,
#   READ_REGISTER / READ_STATUS timing is ~42 us, yet this reads 1 byte.

   @micropython.native
   def _intr(self, pin,
      READ_STATUS_CMD=(    # PIO FIFO 1 word
         (2-1)             << 24 |
         INSTR_READ_STATUS << 16 |
         0
      ),
      READ_CANINTF_CMD=(   # PIO FIFO 1 word
         (3-1)             << 24 |
         INSTR_READ        << 16 | 
         MCP_CANINTF       <<  8 | 
         0
      ),
      READ_EFLG_CMD=(      # PIO FIFO 1 word
         (3-1)             << 24 |
         INSTR_READ        << 16 | 
         MCP_EFLG          <<  8 | 
         0
      ),
      WRITE_CANINTF_CMD=(  # PIO FIFO 1 word
         (3-1)             << 24 |
         INSTR_WRITE       << 16 | 
         MCP_CANINTF       <<  8 | 
         0
      ),
      READ_RX0_CMD=(       # PIO FIFO 4 words
         (15-1)            << 24 | # read more for PIO autopush of last byte
         INSTR_RXBUF0      << 16 |
         0
      ),
      READ_RX1_CMD=(       # PIO FIFO 4 words
         (15-1)            << 24 | # read more for PIO autopush of last byte
         INSTR_RXBUF1      << 16 |
         0
      ),
      BIT_MOD_CMD=(        # PIO FIFO 2 words; Data byte always zero (PIO shift)
         (4-1)             << 24 |
         INSTR_BIT_MOD     << 16 |
         MCP_CANINTF       <<  8 |
         0                 # bit clear mask to be ORed in
      ),
      b_1=bytearray(1),
      b_2=bytearray(2),
      qi=0,
      stat=0,
      intr=0,
      stamp=utime.ticks_us(),
      tramp=utime.ticks_us()
   ):

      if not self.sm.active():    # PIO not running?
         return                   # .send() in progress, PIO will clash with SPI
      if self.sflag: return       # Quick return if IRQ is busy
      self.sflag = 1              # Raise IRQ busy semaphore
         
      stamp = utime.ticks_us()
      self.sm.put(                # GET INTERRUPT FLAGS
         READ_CANINTF_CMD
      )
      self.sm.get(b_1)
      self.stats[1] += utime.ticks_diff(utime.ticks_us(), stamp)
      self.stats[0] += 1
      tramp = stamp
      stat = b_1[0]
      if stat & 0x20:             # Error of any sort?
         stamp = utime.ticks_us()
         self.sm.put(             # GET ERROR FLAGS
            READ_EFLG_CMD
         )
         self.sm.get(b_1)
         self.stats[1] += utime.ticks_diff(utime.ticks_us(), stamp)
         self.stats[0] += 1
         qi = self.nque % iCAN.CQSIZE
         self.eelem[qi] = b_1[0]
         self.cque.put_sync(qi)   # Queue error indication, interpret later
         self.nque += 1

      while stat & 0x03:          # cycle while something to read?

         if stat & 0x01:          # ... in RX0
            qi = self.nque % iCAN.CQSIZE
            stamp = utime.ticks_us()
            self.sm.put(          # READ RX0
               READ_RX0_CMD
            )
            self.sm.get(self.melem[qi])
            self.stats[3] += utime.ticks_diff(utime.ticks_us(), stamp)
            self.stats[2] += 1
            self.eelem[qi] = 0x00 # No error
            self.cque.put_sync(qi)
            self.nque += 1

         if stat & 0x02:          # ... in RX1
            qi = self.nque % iCAN.CQSIZE
            stamp = utime.ticks_us()
            self.sm.put(          # READ RX1
               READ_RX1_CMD
            )
            self.sm.get(self.melem[qi])
            self.stats[3] += utime.ticks_diff(utime.ticks_us(), stamp)
            self.stats[2] += 1
            self.eelem[qi] = 0x00 # No error
            self.cque.put_sync(qi)
            self.nque += 1

         stamp = utime.ticks_us()
         self.sm.put(             # READ INTERRUPT FLAGS (again)
            READ_CANINTF_CMD
         )
         self.sm.get(b_1)
         self.stats[1] += utime.ticks_diff(utime.ticks_us(), stamp)
         self.stats[0] += 1
         stat = b_1[0]
         if stat & 0x20:          # Error of any sort?
            self.sm.put(          # READ ERROR FLAGS
               READ_EFLG_CMD
            )
            self.sm.get(b_1)
            qi = self.nque % iCAN.CQSIZE
            self.eelem[qi] = b_1[0]
            self.cque.put_sync(qi)
            self.nque += 1

      self.sflag = 0              # Lower IRQ busy semaphore
      stamp = utime.ticks_us()
      self.sm.put(                # CLEAR ERROR FLAG; others already cleared
         BIT_MOD_CMD | 0x20       # by READ RX0 / READ RX1
      )                           # this preserves any new read indications
      self.sm.get(b_2)
      self.stats[1] += utime.ticks_diff(utime.ticks_us(), stamp)
      self.stats[0] += 1
      self.stats[5] += utime.ticks_diff(utime.ticks_us(), tramp)
      self.stats[4] += 1

   def __aiter__(self):           # enable await for pkt in .... feature
      return self

   async def __anext__(self):     # return next pkt in for pkt in ....
      i = await self.cque.get()
      return (self.melem[i], self.eelem[i])

   def send(self, ID=None, data=None, EFF=False, RTR=False):
      from canbus import CanMsg, CanMsgFlag

      self.sm.active(0)           # turn off PIO; interferes with SPI signals
      cflg =  CanMsgFlag.EFF if EFF else 0
      cflg |= CanMsgFlag.RTR if RTR else 0
      msg = CanMsg(can_id=ID, data=data, flags=cflg)
      ret = self.can.send(msg)
      self.sm.active(1)           # turn on PIO
      return ret

   def stop(self):
      self.sm.active(0)
      self.pin.irq(handler=None)

   def stat(self):
      return 'IRQs %d, status %d, reads %d: mean status %f 𝞵s, mean read %f 𝞵s, mean IRQ %f 𝞵s' % (
         self.stats[4], self.stats[0], self.stats[2],
         self.stats[1]/max(1,self.stats[0]), self.stats[3]/max(1,self.stats[2]),
         self.stats[5]/max(1,self.stats[4])
      )

async def DEBUG_OUT():
   from canbus import CanError

   #                  0               1
   #                  0123456789ABCDEF0123456789ABCDEF
   def mycode(ba, sp="□.........↲↓↧⇤.................."):
      return ''.join(map(
         lambda c: sp[c] if c < 0x20 else ('.' if c > 0x7f else chr(c)),
         ba
      ))

   def itob(wrd):
      return wrd.to_bytes(4,'big')

   buf = bytearray(CS2_SIZE)

   async for msg, err in can:     # Next queued CAN message
      if err:
         print('*** CAN receive error %s' % CanError.decode(error=err))
         continue

      dbu, dbl = itob(msg[0]), itob(msg[1])
      dbd = dbl[2:4] + itob(msg[2]) + itob(msg[3] << 8)
      ID = ((dbu[1] << 3) | (dbu[2] >> 5)) << 2 | (dbu[2] & 0x03)
      dlc = dbl[1] & 0x0f
      buf[0] = ID >> 8 & 0xff
      buf[1] = ID      & 0xff
      buf[2] = dbu[3]
      buf[3] = dbl[0]
      buf[4] = dlc
      buf[5:5+dlc] = dbd[0:dlc]
      buf[5+dlc:CS2_SIZE] = (8-dlc)*b'\x00'

      cmd = (int.from_bytes(buf[0:2]) >> 1) & 0x7f
      cdec = ('[%02x]' % buf[9]) if cmd == 0 else ('(%02x)' % cmd)
      data = '%04x %04x %02x %s %s *%s*' % (
         int.from_bytes(buf[0:2]), int.from_bytes(buf[2:4]),
         buf[4],
         ' '.join(map(''.join, zip(*[iter(buf[5:].hex())]*4))),
         cdec, mycode(buf[5:5+int(buf[4])])
      )
      print(data)
      dec.decode(
         int.from_bytes(buf[0:4]), buf[5:5+int(buf[4])]
      )

async def HEARTBEAT():
   # Slow LED flash heartbeat while running; boot button restarts.
   import gc
   gc.enable()

   pico_led = Pin("LED", Pin.OUT)
   while True:
      if rp2.bootsel_button() == 1: sys.exit()
      gc.collect()
      gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
      pico_led.on()
      await asyncio.sleep(1)
      pico_led.off()
      await asyncio.sleep(1)

beat = asyncio.create_task(HEARTBEAT())

# Create a Can object instance for interfacing with the CAN bus
try:
   can = iCAN(_CANBOARD)
   asyncio.run(DEBUG_OUT())      # run it / read it
except RuntimeError:
   print(
      '***Error initializing %s CAN board; check configuration/wiring.' %
      _CANBOARD
   )
except KeyboardInterrupt:
   can.stop()
   print(can.stat())
