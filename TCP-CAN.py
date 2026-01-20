# Wifi - CAN bus hub, using PIO for SPI interactions to speed up CAN read/write
#    Uses either TCP or UDP over Wifi.

# Copyright (c) 2025-2026 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)
#          or Pico-CAN-B board by Waveshare (https://www.waveshare.com)

# Runs with:
# MicroPython v1.24.1 on 2024-11-29; Raspberry Pi Pico W with RP2040

# 16 Jan. 2025
# last revision 20 Jan 2026

_VER = const('AN206')            # version ID

################################## Configuration variables start here...

SSID = "****"                    # Wifi network ID
PASS = "****"                    # Wifi network password

_CANBOARD = const('auto')        # Board choice: 'auto' for auto-detect or
                                 # 'JI' or 'WS'
_IPP = const('TCP')              # Protocol choice: TCP or UDP

################################## Configuration variables ...end here

CS2_PORT = const(15731)          # Marklin diktat: TCP
CS2_RPORT = const(15731)         # Marklin diktat: UDP
CS2_SPORT = const(15730)         # Marklin diktat: UDP
CS2_SIZE = const(13)             # Fixed by protocol definition

QSIZE = const(50)                # Size of various I/O queues (modest)

NODE_ID = const(1)               # "S88" node ID
SETTLE_TIME = const(65)          # "S88" contact settle time (ms) (was 125)

INSTR_READ_STATUS = const(0xA0)  # PIO for fast reading of 250 kbps CAN bus
INSTR_WRITE       = const(0x02)
INSTR_READ        = const(0x03)
INSTR_RXBUF0      = const(0x90)
INSTR_RXBUF1      = const(0x94)
INSTR_BIT_MOD     = const(0x05)

MCP_CANINTF       = const(0x2C)
MCP_EFLG          = const(0x2D)
MCP_TXB0          = const(0x30)
MCP_TXB1          = const(0x40)
MCP_TXB2          = const(0x50)

import uasyncio as asyncio
import network
import usocket as socket
import sys
from time import sleep
from asyncio import Loop
from machine import Pin, Timer
from rp2 import PIO, StateMachine
from array import array

from micropython import alloc_emergency_exception_buf as AEEB
AEEB(100)                        # boilerplate for IRQ-level exception reporting

@rp2.asm_pio(
   push_thresh=32, autopush=True, pull_thresh=32, autopull=True,
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

pico_led = Pin("LED", Pin.OUT)

class iCAN:                      # interrupt driven CAN message sniffer
   from collections import namedtuple

   CQSIZE = 250                  # Big to read CONFIG DATA bursts

   CAN_pins = namedtuple('CAN pins',
      ['INT_PIN','SPI_CS', 'SPI_SCK', 'SPI_MOSI', 'SPI_MISO', 'FBP', 'name']
   )
   boards = dict(
      JI = CAN_pins(
         name = 'Joy-IT',           # These pin assignments are appropriate for
                                    # a RB-P-CAN-485 Joy-IT board
         INT_PIN = 20,              # Interrupt pin for CAN board
         SPI_CS = 17,
         SPI_SCK = 18,
         SPI_MOSI = 19,
         SPI_MISO = 16,
         FBP = bytes((              # Feedback pins
              #0  1  2  3   4   5   6   7  channel number
               0, 1, 8, 9, 10, 11, 14, 15 #GPIO pin number
         ))
      ),
      WS = CAN_pins(
         name = 'Waveshare',        # These pin assignments are appropriate for
                                    # a Waveshare Pico-CAN-B board
         INT_PIN = 21,              # Interrupt pin for CAN board
         SPI_CS = 5,
         SPI_SCK = 6,
         SPI_MOSI = 7,
         SPI_MISO = 4,
         FBP = bytes((              # Feedback pins
              #0  1  2  3   4   5   6   7  channel number
               0, 1, 2, 3, 10, 11, 12, 13 #GPIO pin number
         ))
      )
   )

   def __init__(self, conf=None):
      from machine import Pin, SPI
      from canbus import Can, CanError
      from canbus.internal import CAN_SPEED

      try:
         CanError.decode()       # Needed for CAN bus error reports
      except:
         raise RuntimeError("Update canbus module - new features used")

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

      self._pins = pin
      self.pin = Pin(pin.INT_PIN, Pin.IN, Pin.PULL_UP)
      self.can = Can(spics=pin.SPI_CS) # CS pin for hardware SPI 0
      # Initialize the CAN interface.  Reference says 250 kbps speed.
      ret = self.can.begin(bitrate=CAN_SPEED.CAN_250KBPS)
      if ret != CanError.ERROR_OK:
         raise RuntimeError('Error initializing CAN bus')

      self.SPI_CS = Pin(pin.SPI_CS, Pin.OUT)

      self.sm = rp2.StateMachine(0)
      self.sm.init(SPI_OP,
            freq=32_000_000,     # WARNING: 32 MHz max rate before bit loss 
            in_base=pin.SPI_MISO,
            out_base=pin.SPI_MOSI,
            set_base=pin.SPI_SCK,
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
      self.sflag = 0
      self.buf = bytearray(13)
      self.a_4 = array('I',4*[0])
      self.a_2 = array('I',2*[0])
      self.b_4 = bytearray(4)
      self.b_2 = bytearray(2)
      self.b_1 = bytearray(1)
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
      BIT_MOD_CMD=(        # PIO FIFO 2 words; Data byte in 2nd word high byte
         (4-1)             << 24 |
         INSTR_BIT_MOD     << 16 |
         MCP_CANINTF       <<  8 |
         0                 # bit clear mask to be ORed in
      ),
      qi=0,
      stat=0,
      intr=0
   ):

      if self.sflag: return       # Quick return if IRQ is busy
      self.sflag = 1              # Raise IRQ busy semaphore

      a_4 = self.a_4              # Set up local vars (optimize using locals)
      a_2 = self.a_2
      b_2 = self.b_2
      b_1 = self.b_1
      sm = self.sm
      cque = self.cque
         
      sm.put(                     # GET INTERRUPT FLAGS
         READ_CANINTF_CMD
      )
      sm.get(b_1)
      stat = b_1[0]
      if stat & 0x20:             # Error of any sort?
         sm.put(                  # GET ERROR FLAGS
            READ_EFLG_CMD
         )
         sm.get(b_1)
         qi = self.nque % iCAN.CQSIZE
         self.eelem[qi] = b_1[0]
         cque.put_sync(qi)        # Queue error indication, interpret later
         self.nque += 1

      while stat & 0x03:          # cycle while something to read?

         if stat & 0x01:          # ... in RX0
            qi = self.nque % iCAN.CQSIZE
            a_4[0] = READ_RX0_CMD
            sm.put(a_4)           # READ RX0
            sm.get(self.melem[qi])
            self.eelem[qi] = 0x00 # No error
            cque.put_sync(qi)
            self.nque += 1

         if stat & 0x02:          # ... in RX1
            qi = self.nque % iCAN.CQSIZE
            a_4[0] = READ_RX1_CMD
            sm.put(a_4)           # READ RX1
            sm.get(self.melem[qi])
            self.eelem[qi] = 0x00 # No error
            cque.put_sync(qi)
            self.nque += 1

         self.sm.put(             # READ INTERRUPT FLAGS (again)
            READ_CANINTF_CMD
         )
         sm.get(b_1)
         stat = b_1[0]
         if stat & 0x20:          # Error of any sort?
            sm.put(               # READ ERROR FLAGS
               READ_EFLG_CMD
            )
            sm.get(b_1)
            qi = self.nque % iCAN.CQSIZE
            self.eelem[qi] = b_1[0]
            cque.put_sync(qi)
            self.nque += 1

      a_2[0] = BIT_MOD_CMD | 0x20 # CLEAR ERROR FLAG; others already cleared
      a_2[1] = 0
      sm.put(a_2)                 # by READ RX0 / READ RX1; 
      sm.get(b_2)                 # this preserves any new read indications
      self.sflag = 0              # Lower IRQ busy semaphore

   def __aiter__(self):           # enable await for pkt in .... feature
      return self

   async def __anext__(self):     # return next pkt in for pkt in ....
      i = await self.cque.get()
      return (self.melem[i], self.eelem[i])

   @property
   def pins(self):               # allow e.g. can.pins.FBP, can.pins.name
      return self._pins

   @micropython.native
   def send(self, ID=None, data=None, EFF=False, RTR=False):

      dlc = 0 if data is None else len(data)
      buf = self.buf
      if EFF:
         buf[0] = (ID >> 21) & 0xFF
         buf[1] = (
            ((ID & 0x1C0000) >> 13) |
            ((ID & 0x030000) >> 16) |
            0x08
         )
         buf[2] = (ID >> 8) & 0xFF
         buf[3] = ID        & 0xFF
      else:
         buf[0] = (ID >> 3) & 0xFF
         buf[1] = (ID & 0x07) << 5
      buf[4] = (
         (dlc & 0x0F) |
         (0x40 if RTR else 0x00)
      )
      if dlc > 0: buf[5:5+dlc] = data[0:dlc]

      self.sflag = 1              # raise semaphore to block reads

      b_1 = self.b_1
      for TXB in [MCP_TXB0, MCP_TXB1, MCP_TXB2]:
         READ_TXBF_CMD=(      # PIO FIFO 1 word
            (3-1)             << 24 |
            INSTR_READ        << 16 | 
            TXB               <<  8 | 
            0
         )
         self.sm.put(READ_TXBF_CMD)
         self.sm.get(b_1)
         if not(b_1[0] & 0x08):   # clear buf?
            break
      else:
         self.sflag = 0           # lower semaphore to unblock reads
         self._intr_ref(self.pin) # check for any read interrupts
         return 0x04              # return some kind of error

      a_4 = self.a_4
      b_4 = self.b_4
      WRITE_TXBF_CMD=(        # PIO FIFO 4 words
         (2+5+8-1)            << 24 |
         INSTR_WRITE          << 16 | 
         (TXB + 1)            <<  8 | 
         0
      )
      a_4[0] = WRITE_TXBF_CMD | buf[0]
      a_4[1] = int.from_bytes(buf[1:5])
      a_4[2] = int.from_bytes(buf[5:9])
      a_4[3] = int.from_bytes(buf[9:13])
      self.sm.put(a_4)
      self.sm.get(b_4)

      BIT_MOD_CMD=(        # PIO FIFO 2 words; bit clear value in 2nd word
         (4-1)             << 24 |
         INSTR_BIT_MOD     << 16 |
         TXB               <<  8 |
         0                 # bit clear mask to be ORed in
      )

      a_2 = self.a_2
      b_2 = self.b_2
      a_2[0] = BIT_MOD_CMD | 0x0B
      a_2[1] = (0x08 | ((ID >> 29) & 0x03)) << 24
      self.sm.put(a_2)            # mark buffer ready for transmit
      self.sm.get(b_2)

      self.sm.put(READ_TXBF_CMD)
      self.sm.get(b_1)
      stat = b_1[0] & 0x70        # TXB_ABTF | TXB_MLOA | TXB_TXERR

      self.sflag = 0              # lower semaphore after read
      self._intr_ref(self.pin)    # check for any read interrupts

      return stat

   def stop(self):
      # Turns off CAN interrupt handling, disabling board for reading.
      self.pin.irq(handler=None)
      self.sm.active(0)

class feedback:
   # Implement feedback by logic-level input from track sensors

   interrupt = False                  # True for interrupt or False for poll

   def __init__(self,node,pins):
      myhash = 0x5338
      self.n = len(pins)
      self._n, self._secs = 0, 0

      self.fbpp = bytearray(CS2_SIZE) # Feedback poll packet
      self.pool = self.n*[None]
      self.ptmr = bytearray(self.n)   # Feedback packet pool & pin timers
      self.fbpin = self.n*[None]      # Feedback pins
      self.chn = bytearray(self.n)    # Current contact level
      self.state = bytearray(self.n)  # Internal contact state
      self.pins = pins                # Save pin layout for reset

      self.wflag = asyncio.ThreadSafeFlag()
      self.flag = asyncio.ThreadSafeFlag()

      ch = 0
      for pin in pins:
         self.fbpin[ch] = Pin(pin, Pin.IN, Pin.PULL_UP)
         if feedback.interrupt: self.fbpin[ch].irq(
            # this defines the interrupt handler
               trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
               handler=lambda p, id=ch: self._intr(id),
               hard=True
            )

         val = not self.fbpin[ch].value()  # Get present pin state to initialize
         self.chn[ch], self.state[ch] = val, val

         self.pool[ch] = bytearray(CS2_SIZE)
         pkt = self.pool[ch]          # Allocate buffer for each channel
         pkt[0] = 0x11 >> 7 & 0xff
         pkt[1] = 0x11 << 1 & 0xff | 0x01   # set R flag
         pkt[2] = myhash >> 8 & 0xff
         pkt[3] = myhash & 0xff
         pkt[4] = 8
         pkt[5:7] = bytes((0,node))
         pkt[7:9] = bytes((0,ch))
         pkt[9], pkt[10] = self.state[ch], self.chn[ch]
         pkt[11:13] = 2*b'\x00'
         ch += 1

      # Build current state packet in response to an S88 POLL cmd
      val = 0
      for i in range(self.n): val |= self.state[i] << i
      self.fbpp[0] = 0x10 >> 7 & 0xff    # Build state packet
      self.fbpp[1] = 0x10 << 1 | 0x01    # set R flag
      self.fbpp[2] = myhash >> 8 & 0xff
      self.fbpp[3] = myhash & 0xff
      self.fbpp[4] = 7
      self.fbpp[5:9] = bytes((0x53,0x30,0,node))
      self.fbpp[9] = node
      self.fbpp[10], self.fbpp[11] = 0, val
      self.fbpp[12] = 0

      print('Feedback: %d channels, "S88" module %d.' % (self.n,node))

      self.ticker = Timer()
      self.ticker.init(
         mode=Timer.PERIODIC,
         period=1000,
         callback=self._tick
      )

      # defines pin change timer (if interrupt) or polling task
      try:
         self.task = asyncio.create_task(
            self._time() if feedback.interrupt else self._poll()
         )
      except CancelledError:
         pass


   async def _time(self):        # Wait for feedback pin change

      # Initialized.  Fall into processing loop.
      # Debouncing of channel signal is handled two ways:
      # 1) in interrupt routine, flag raised only if there is a state
      #    change on the pin; similarly in the poll routine.
      # 2) after a state change, a timer is set for a SETTLE_TIME delay to
      #    see whether the pin state changed.  If the pin changes in this
      #    interval, it is only recognized if it results in a change in
      #    state.

      self.timer = self.n*[None]
      while True:
         await self.flag.wait()
         pps = 0
         for i in range(self.n):
            if self.state[i] != self.chn[i] and not self.ptmr[i]:
               if self.timer[i] is None: self.timer[i] = machine.Timer()
               self.timer[i].init(# Check state later
                  mode=Timer.ONE_SHOT,
                  period=SETTLE_TIME,
                  callback=lambda t, ch=i: self._check(ch)
               )
               self.ptmr[i] = 1
            pps |= self.state[i] << i
         self.fbpp[11] = pps     # Maintain state in poll packet

   def _tick(self,arg):          # Seconds counter
      self._secs += 1

   async def _poll(self):        # Polling routine
      while True:
         self._n += 1
         pps = 0
         for n in range(self.n):
            self.chn[n] = not self.fbpin[n].value()
            if self.state[n] != self.chn[n]:
               self.wflag.set()
            pps |= self.state[n] << i
         self.fbpp[11] = pps     # Maintain state in poll packet
         await asyncio.sleep_ms(SETTLE_TIME)

   def _intr(self,pin):          # Interrupt handler
      self._n += 1
      self.chn[pin] = not self.fbpin[pin].value()
      if self.chn[pin] != self.state[pin]:
         self.flag.set()         # Signal waiting task (interrupt; poll ignores)

   def _check(self,pin):
      # Activated after change of state of feedback channel by the timer

      val = not self.fbpin[pin].value()  # Current channel state
      if self.state[pin] != val:    # Internal state same as present pin state?
         self.chn[pin] = val        # No - generate a feedback event
         pkt = self.pool[pin]
         pkt[9], pkt[10] = self.state[pin], self.chn[pin]
         self.wflag.set()
      else:
         self.state[pin] = self.chn[pin]
      self.ptmr[pin] = 0            # Handled channel, schedule next timer

   def stop(self):
      self.ticker.deinit()
      if feedback.interrupt:
         for i in range(self.n): self.fbpin[i].irq(handler=None)
      try:
         self.task.cancel()
      except:
         pass
      
   @property
   def state_packet(self):       # provide state packet
      return self.fbpp

   @property
   def stats(self):              # provide interrupt stats
      return (self._n, self._secs)

   def __aiter__(self):          # enable await for pkt in .... feature
      return self

   async def __anext__(self):    # return next pkt in for pkt in ....
      while True:
         await self.wflag.wait()
         for n in range(self.n):
            if self.state[n] != self.chn[n]:
               break             # found a change
         else:
            continue             # no change? go wait on flag again
         break                   # found a change
      pkt = self.pool[n]
      pkt[9], pkt[10] = self.state[n], self.chn[n]
      self.state[n] = self.chn[n]
      return pkt

from threadsafe import ThreadSafeQueue

CtoT, TtoC, DBQ = [], [], []
for i in range(QSIZE):
   CtoT.append(bytearray(CS2_SIZE))
   TtoC.append(bytearray(CS2_SIZE))
   DBQ.append(bytearray(CS2_SIZE))
CANtoTCP, ixCT = ThreadSafeQueue(QSIZE), 0
TCPtoCAN, ixTC = ThreadSafeQueue(QSIZE), 0
debugQUE, ixDB = ThreadSafeQueue(QSIZE), 0

TCP_R, TCP_W = None, None
TCP_RERR, TCP_WERR = False, False

rrhash = 0

qfTC, qfCT, qfDB = False, False, False
CANmsg = const('''
              ***********************
              ***CAN output q full***
              ***********************
''')
TCPmsg = const('''
              ************************
              ***WiFi output q full***
              ************************
''')
DBGmsg = const('''
                 ****************
                 ***log q full***
                 ****************
''')

def qput(pkt,queue,flag,msg):
   # Code readability device to avoid many try/except indents
   # Does synchronous put to a threadsafe queue and prints error message
   # depending on whether a previous error already reported.
   # Use flag arg idiomatically: FLAG = qput(pkt,queue,FLAG,msg)

   if queue.full():
      if not flag: print(msg)
      return True

   queue.put_sync(pkt)
   return False

async def TCP_SERVER(R, W):
   # Callback when RocRail client connects to us
   global TCP_R, TCP_W, TCP_RERR, TCP_WERR
   print('TCP connection made, waiting for traffic.')
   TCP_R, TCP_W = R, W
   while True:
      await asyncio.sleep(15)
      if TCP_RERR and TCP_WERR:
         # Must be a disconnect; finish and wait for a new connection
         print('*** TCP connection lost, waiting for reconnect.')
         break

async def TCP_READER():
   # packet layout:
   #    xx xx xx xx  xx  xx xx xx xx xx xx xx xx  -  13 bytes total = CS2_SIZE
   #    -----------  --  -----------------------
   #       CAN ID    len  data (left justified)
   global TCP_R, TCP_RERR, rrhash, ixTC, ixDB, ixCT

   while True:                   # Wait for connection
      if TCP_R is None:
         await asyncio.sleep_ms(10)
         continue

      try:                       # Serve it
         pkt = await TCP_R.readexactly(CS2_SIZE)
      except EOFError:           # Connection lost/client disconnect
         TCP_RERR, TCP_R = True, None
         continue
      except OSError:            # Connection reset/client disconnect
         TCP_RERR, TCP_R = True, None
         continue

      assert len(pkt) == CS2_SIZE
      rrhash = int.from_bytes(pkt[2:4])

      #  Quick command parse for special responses
      cmd = int.from_bytes(pkt[0:2]) >> 1 & 0xff
      sub = int(pkt[9]) if pkt[4] > 4 else -1
      rsp = pkt[1] & 0x01

      if cmd == 0x1b and not rsp:     # CAN BOOT?
         while not CANtoTCP.empty():
            CANtoTCP.get_sync()       # discard any queued responses
         while not debugQUE.empty():
            debugQUE.get_sync()       # discard any queued responses

      buf = TtoC[ixTC % QSIZE]   # Normal handling 
      buf[0:CS2_SIZE] = pkt
      await TCPtoCAN.put(buf)
      ixTC += 1
      buf = DBQ[ixDB % QSIZE]
      buf[0:CS2_SIZE] = pkt
      await debugQUE.put(buf)
      ixDB += 1

      if cmd == 0x10 and sub == NODE_ID and not rsp:
         fbpp = copy.copy(fdbk.state_packet) # Check for S88 state poll
         buf = CtoT[ixCT % QSIZE]
         buf[0:CS2_SIZE] = fbpp
         await CANtoTCP.put(buf)
         ixCT += 1
         buf = DBQ[ixDB % QSIZE]
         buf[0:CS2_SIZE] = fbpp
         await debugQUE.put(buf)
         ixDB += 1

async def UDP_READER(timeout=0):
   # packet layout:
   #    xx xx xx xx  xx  xx xx xx xx xx xx xx xx  -  13 bytes total = CS2_SIZE
   #    -----------  --  -----------------------
   #       CAN ID    len  data (left justified)
   import uselect as select
   global rrhash, qfCT, qfTC, qfDB

   s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   s.setblocking(False)
   s.bind(('0.0.0.0',CS2_RPORT))
   p = select.poll()
   p.register(s, select.POLLIN)
   print('UDP listening, waiting for traffic.')
   cpkt, npkt = [], 0
   for n in range(QSIZE): cpkt.append(bytearray(CS2_SIZE))

   while True:                   # Wait for activity on port
      pkt = cpkt[npkt % QSIZE]
      n = s.readinto(pkt, CS2_SIZE)
      if n is not None:
         assert n == CS2_SIZE
         mem = memoryview(pkt)
         rrhash = int.from_bytes(mem[2:4])
         qfTC = qput(pkt,TCPtoCAN,qfTC,CANmsg)
         qfDB = qput(pkt,debugQUE,qfDB,DBGmsg)

         #  Check for S88 state poll
         cmd = int.from_bytes(mem[0:2]) >> 1 & 0xff
         sub = int(pkt[9]) if pkt[4] > 4 else -1
         rsp = pkt[1] & 0x01
         if cmd == 0x10 and sub == NODE_ID and not rsp:
            fbpp = copy.copy(fdbk.state_packet)
            qfCT = qput(fbpp, CANtoTCP, qfCT, TCPmsg)
            qfDB = qput(fbpp, debugQUE, qfDB, DBGmsg)

         npkt += 1
         continue
      await asyncio.sleep_ms(timeout)

can = iCAN(_CANBOARD)
async def CAN_READER():
   from canbus import CanError
   global qfCT, ixCT, qfDB, ixDB

   def itob(wrd):
      return bytes(
         (wrd >> 24 & 0xff, wrd >> 16 & 0xff, wrd >> 8 & 0xff, wrd & 0xff)
      )

   print(
      "CAN %s board initialized successfully, waiting for traffic." %
      can.pins.name
   )

   buf = bytearray(CS2_SIZE)

   async for msg, err in can:     # Next queued CAN message
      if err:
         print('*** CAN receive error %s' % CanError.decode(txctl=err))
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

      pkt = CtoT[ixCT % QSIZE]
      pkt[0:CS2_SIZE] = buf
      qfCT = qput(pkt, CANtoTCP, qfCT, TCPmsg)
      ixCT += 1
      pkt = DBQ[ixDB % QSIZE]
      pkt[0:CS2_SIZE] = buf
      qfDB = qput(pkt, debugQUE, qfDB, DBGmsg)
      ixDB += 1

async def TCP_WRITER():
   global TCP_W, TCP_WERR

   while True:                   # Wait for connection
      if TCP_W is None:
         await asyncio.sleep_ms(10)
         continue

      try:                       # Serve it
         async for pkt in CANtoTCP:
            assert len(pkt) == CS2_SIZE
            TCP_W.write(pkt)
            TCP_W.drain()
      except OSError:            # Connection lost/client disconnect
         TCP_WERR, TCP_W = True, None

async def UDP_WRITER():
                                 # set up for UDP use
   s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
   s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
   s.bind(('0.0.0.0',0))         # assumed to be only one interface

   while True:
      async for pkt in CANtoTCP:
         assert len(pkt) == CS2_SIZE
         s.sendto(pkt, ('255.255.255.255',CS2_SPORT))

async def CAN_WRITER(MERR=5, MCNT=500):
   from canbus import CanError

   async for pkt in TCPtoCAN:     # process next incoming TCP packet
      assert len(pkt) == CS2_SIZE
      cnt = 0
      while True:                 # loop till sent or give up after errors
         errf = can.send(
            ID=int.from_bytes(pkt[0:4]),
            data=pkt[5:5+int(pkt[4])],
            EFF=True
         )
         if not errf:
            break                 # sent successfully
         cnt += 1
         if cnt <= MERR:
            print('   >>>CAN write error<<< (err %02x %s)%s' % 
               (errf, CanError.decode(error=errf),
                  ' - quelling further reports' if cnt >= MERR else '')
            )
         await asyncio.sleep_ms(10)
         if cnt > MCNT:          # Abandon packet after this many tries
            break

async def DEBUG_OUT():
   global rrhash, dec

   #                  0               1
   #                  0123456789ABCDEF0123456789ABCDEF
   def mycode(ba, sp="□.........↲↓↧⇤.................."):
      str = ''
      for b in ba:
         str += sp[b] if b < 0x20 else ('.' if b > 0x7f else chr(b))
      return str

   async for buf in debugQUE:
      assert len(buf) == CS2_SIZE
      cmd = (int.from_bytes(buf[0:2]) >> 1) & 0x7f
      data = '%04x %04x %02x %s (%02x%s) *%s*' % (
         int.from_bytes(buf[0:2]), int.from_bytes(buf[2:4]),
         buf[4],
         ' '.join(map(''.join, zip(*[iter(buf[5:].hex())]*4))),
         cmd, '' if cmd != 0 else ('/%02x' % buf[9]),
         mycode(buf[5:5+int(buf[4])])
      )
      cid = int.from_bytes(buf[2:4])
      dir = '->' if cid == rrhash else '<-'
      print('%s %s CAN %s' % (_IPP,dir,data))
      dec.decode(
         int.from_bytes(buf[0:4]), buf[5:5+int(buf[4])]
      )
      if TCPtoCAN.qsize() > 0 and TCPtoCAN.qsize() % 5 == 0:
         print('TCPtoCAN queue congestion: %d waiting' % TCPtoCAN.qsize())
      if CANtoTCP.qsize() > 0 and CANtoTCP.qsize() % 5 == 0:
         print('CANtoTCP queue congestion: %d waiting' % CANtoTCP.qsize())

async def HEARTBEAT():
   # Slow LED flash heartbeat while running; boot button restarts.
   global pico_led
   while True:
      if rp2.bootsel_button() == 1: sys.exit()
      pico_led.on()
      await asyncio.sleep(1)
      pico_led.off()
      await asyncio.sleep(1)

fdbk = feedback(NODE_ID,can.pins.FBP) # Feedback framework initialization

async def FEEDBACK():
   # Simulated S88 feedback

   def post(pkt):                # Called for every change in state
      global qfCT, qfDB
      qfCT = qput(pkt, CANtoTCP, qfCT, TCPmsg)
      qfDB = qput(pkt, debugQUE, qfDB, DBGmsg)

   async for pkt in fdbk:        # Wait for change in state of some feedback pin
      global qfCT, qfDB
      qfCT = qput(pkt, CANtoTCP, qfCT, TCPmsg)
      qfDB = qput(pkt, debugQUE, qfDB, DBGmsg)

async def RUNNER(ip):
   srvr = await asyncio.start_server(TCP_SERVER,ip,CS2_PORT)
   while True:
      await asyncio.sleep(60)

print('%s <-> CAN packet hub (%s)' % (_IPP,_VER))
try:
   from marklin import decode, CS2decoder    # Marklin CS2 CAN packet decoder
except:
   # Dummy decoder if not available
   class CS2decoder:
      def __init__(self,*pos,**kwd):
         pass
      def decode(self,*pos,**kwd):
         pass
   print('No Märklin packet decoding, only logging raw data.')
dec = CS2decoder(pfx='    ',detail=True,print=True)

#Connect to WLAN
wlan = network.WLAN(network.STA_IF)
if not wlan.isconnected():
   wlan.active(True)
   mac = wlan.config('mac')
   host = 'CS2hub-' + ''.join('{:02x}'.format(b) for b in mac[3:])
   wlan.config(hostname = host)
   wlan.connect(SSID, PASS)
else:
   host = wlan.config('hostname')

while not wlan.isconnected():
   # Fast flash while waiting for wifi connection; boot button restarts.
   if rp2.bootsel_button() == 1: sys.exit()

   pico_led.on()
   sleep(0.25)
   pico_led.off()
   sleep(0.25)

pico_led.on()
ip = wlan.ifconfig()[0]
print('Available at {} as {}, port {:d}.'.format(ip,host,CS2_PORT))

canr = asyncio.create_task(CAN_READER())
canw = asyncio.create_task(CAN_WRITER())
if _IPP == 'TCP':
   tcpr = asyncio.create_task(TCP_READER())
   tcpw = asyncio.create_task(TCP_WRITER())
else:
   udpr = asyncio.create_task(UDP_READER(1))
   udpw = asyncio.create_task(UDP_WRITER())
dbug = asyncio.create_task(DEBUG_OUT())
beat = asyncio.create_task(HEARTBEAT())
feed = asyncio.create_task(FEEDBACK())

try:
   if _IPP == 'TCP':
      runr = asyncio.create_task(RUNNER(ip))
      Loop.run_forever()
   else:
      Loop.run_until_complete(udpr)
except KeyboardInterrupt:
   can.stop()
   stats = fdbk.stats
   fdbk.stop()
   print('Feedback interrupts: %d, %.2f/sec' % (stats[0], stats[0]/stats[1]))
