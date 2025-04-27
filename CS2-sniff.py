# Marklin packet sniffer for MS1/MS2/CS1/CS2 CAN protocol.  Reference:
#  Kommunikationsprotokoll Graphical User Interface Prozessor (GUI) <->
#  Gleisformat Prozessor (GFP) über CAN transportierbar über Ethernet
#  (Version vom 7.2.2012)

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)
#          or Pico-CAN-B board by Waveshare (https://www.waveshare.com)

# original version 15 Jan. '25
# last revision 27 Apr. '25

_VER = 'PR275'                   # version ID

CS2_SIZE = const(13)             # Fixed by protocol definition

QSIZE = const(25)                # Size of various I/O queues (overkill)

_CANBOARD = const('auto')        # Board choice: 'auto' for auto-detect or
                                 # 'JI' or 'WS'

import uasyncio as asyncio
from threadsafe import ThreadSafeQueue
from machine import Pin
import sys

from micropython import alloc_emergency_exception_buf as AEEB
AEEB(100)                        # boilerplate for IRQ-level exception reporting

print('CS1/CS2 protocol packet sniffer (%s)' % _VER)
try:
   from marklin import decode    # Marklin CS2 CAN packet decoder
   avail = True
except:
   avail = False                 # don't use it if not available
   print('No Märklin packet decoding, only logging raw data.')

class iCAN:                      # interrupt driven CAN message sniffer
   from collections import namedtuple

   CAN_pins = namedtuple('CAN pins',
      ['INT_PIN','SPI_CS', 'SPI_SCK', 'SPI_MOSI', 'SPI_MISO', 'name']
   )
   pins_JI = CAN_pins(
      name = 'Joy-IT',           # These pin assignments are appropriate for
                                 # a RB-P-CAN-485 Joy-IT board
      INT_PIN = 20,              # Interrupt pin for CAN board
      SPI_CS = 17,
      SPI_SCK = 18,
      SPI_MOSI = 19,
      SPI_MISO = 16
   )
   pins_WS = CAN_pins(
      name = 'Waveshare',        # These pin assignments are appropriate for
                                 # a Waveshare Pico-CAN-B board
      INT_PIN = 21,              # Interrupt pin for CAN board
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
      from canbus import Can, CanError
      from canbus.internal import CAN_SPEED

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

      self.flag = asyncio.ThreadSafeFlag()
      self.pin = Pin(pin.INT_PIN,Pin.IN,Pin.PULL_UP)
      self.pin.irq(
         trigger=Pin.IRQ_FALLING,
         handler=lambda pin: self.flag.set(),
         hard=True)
      self.can = Can(spics=pin.SPI_CS) # CS pin for hardware SPI 0
      # Initialize the CAN interface.  Reference says 250 kbps speed.
      ret = self.can.begin(bitrate=CAN_SPEED.CAN_250KBPS)
      if ret != CanError.ERROR_OK:
         raise RuntimeError('Error initializing CAN bus')
      print("Initialized successfully, waiting for traffic.")

   async def run(self, proc):
      from canbus import CanError

      await asyncio.sleep(0)
      while True:
         await self.flag.wait()
         intr = self.can.getInterrupts()
         if intr & 0x03 == 0:
            # ignore interrupts except for reading
            self.can.clearInterrupts()
            continue
         while self.can.checkReceive():
            # CAN error code and message
            error, msg = self.can.recv()
            proc(msg, error != CanError.ERROR_OK)
         self.can.clearInterrupts()

   def send(self, ID=None, data=None, EFF=False, RTR=False):
      from canbus import CanMsg, CanMsgFlag

      cflg =  CanMsgFlag.EFF if EFF else 0
      cflg |= CanMsgFlag.RTR if RTR else 0
      msg = CanMsg(can_id=ID, data=data, flags=cflg)
      return self.can.send(msg)

   def stop(self):
      self.pin.irq(handler=None)

DBQ = QSIZE*[None]
for i in range(QSIZE): DBQ[i] = bytearray(CS2_SIZE)
debugQUE, qix = ThreadSafeQueue(QSIZE), 0

def sniffer(msg, err):
   global qix, DBQ
   # Print received message details
   if err:
      print('*** CAN receive error')
      return

   buf = DBQ[qix % QSIZE]
   buf[0] = msg.can_id >> 24 & 0xff 
   buf[1] = msg.can_id >> 16 & 0xff 
   buf[2] = msg.can_id >>  8 & 0xff 
   buf[3] = msg.can_id       & 0xff
   buf[4] = msg.dlc
   buf[5:5+msg.dlc] = msg.data
   buf[5+msg.dlc:CS2_SIZE] = (8-msg.dlc)*b'\x00'
   try:
      debugQUE.put_sync(buf)     # put_sync because no await used
   except:
      print('***logging q full')
   qix += 1

async def DEBUG_OUT():
   async for buf in debugQUE:
      assert len(buf) == CS2_SIZE
      data = '%04x %04x %02x %s' % (
         int.from_bytes(buf[0:2]), int.from_bytes(buf[2:4]),
         buf[4],
         ' '.join(map(''.join, zip(*[iter(buf[5:].hex())]*4)))
      )
      print(data)
      if avail:
         print('   ', decode(
               int.from_bytes(buf[0:4]), buf[5:5+int(buf[4])], detail=True
            )
         )

async def HEARTBEAT():
   # Slow LED flash heartbeat while running; boot button restarts.
   pico_led = Pin("LED", Pin.OUT)
   while True:
      if rp2.bootsel_button() == 1: sys.exit()
      pico_led.on()
      await asyncio.sleep(1)
      pico_led.off()
      await asyncio.sleep(1)

beat = asyncio.create_task(HEARTBEAT())
dbug = asyncio.create_task(DEBUG_OUT())

# Create a Can object instance for interfacing with the CAN bus
try:
   can = iCAN(_CANBOARD)
   asyncio.run(can.run(sniffer))
except RuntimeError:
   print(
      '***Error initializing %s CAN board; check configuration/wiring.' %
      _CANBOARD
   )
except KeyboardInterrupt:
   can.stop()
