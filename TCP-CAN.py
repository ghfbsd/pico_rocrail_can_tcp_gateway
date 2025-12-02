# TCP - CAN bus hub

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)
#          or Pico-CAN-B board by Waveshare (https://www.waveshare.com)

# Runs with:
# MicroPython v1.24.1 on 2024-11-29; Raspberry Pi Pico W with RP2040

# 16 Jan. 2025
# last revision 02 Dec 2025

_VER = const('EC035')            # version ID

SSID = "****"
PASS = "****"

CS2_PORT = const(15731)          # Marklin diktat
CS2_SIZE = const(13)             # Fixed by protocol definition

QSIZE = const(25)                # Size of various I/O queues (overkill)

NODE_ID = const(1)               # "S88" node ID
SETTLE_TIME = const(65)          # "S88" contact settle time (ms) (was 125)

_CANBOARD = const('auto')        # Board choice: 'auto' for auto-detect or
                                 # 'JI' or 'WS'
import uasyncio as asyncio
import network
import sys
from time import sleep
from asyncio import Loop
from machine import Pin, Timer

from micropython import alloc_emergency_exception_buf as AEEB
AEEB(100)                        # boilerplate for IRQ-level exception reporting

pico_led = Pin("LED", Pin.OUT)

class iCAN:                      # interrupt driven CAN message sniffer
   from collections import namedtuple

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
      self.flag = asyncio.ThreadSafeFlag()
      self.pin = Pin(pin.INT_PIN, Pin.IN, Pin.PULL_UP)
      self.pin.irq(              # this defines the interrupt handler (lambda)
         trigger=Pin.IRQ_FALLING,
         handler=lambda pin: self.flag.set(),
         hard=True)
      self.can = Can(spics=pin.SPI_CS) # CS pin for hardware SPI 0
      # Initialize the CAN interface.  Reference says 250 kbps speed.
      ret = self.can.begin(bitrate=CAN_SPEED.CAN_250KBPS)
      if ret != CanError.ERROR_OK:
         raise RuntimeError('Error initializing CAN bus')

   @property
   def pins(self):               # allow e.g. can.pins.FBP, can.pins.name
      return self._pins

   @property
   def intf(self):               # hardware interface level access if needed
      return self.can

   def run(self, proc):
      # Method never returns; calls proc upon every interrupt.
      # Usage:
      #    proc(msg, err) 
      # with
      #    msg - Can message
      #    err - True if read error; False if OK
      # no return value

      task = asyncio.create_task(self.intr(proc))
      Loop.run_until_complete(task)

   async def intr(self, proc):
      # Handle CAN card interrupt.
      # Response to any interrupt is implemented via a callback method
      # rather than as a generator (using e.g. async for x in iCan(): ...)
      # because the interrupt response appears to be faster.  This is
      # due to the checkReceive() loop in the handler itself.

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
            await asyncio.sleep(0)
         self.can.clearInterrupts()

   def send(self, ID=None, data=None, EFF=False, RTR=False):
      # Send a CAN message.
      # Called with:
      #   ID - (integer) CAN id
      #   data - (bytearray) data payload
      #   EFF - set EFF flag in frame for long IDs
      #   RTR - set RTR flag in frame
      # returns
      #   True if send error, False if sent OK

      from canbus import CanMsg, CanMsgFlag, CanError

      cflg =  CanMsgFlag.EFF if EFF else 0
      cflg |= CanMsgFlag.RTR if RTR else 0
      msg = CanMsg(can_id=ID, data=data, flags=cflg)
      return self.can.send(msg) != CanError.ERROR_OK

   def stop(self):
      # Turns off CAN interrupt handling, disabling board for reading.
      self.pin.irq(handler=None)

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
      if not feedback.interrupt:
         # defines polling task if not interrupt-driven
         try:
            self.task = asyncio.create_task(self._poll())
         except CancelledError:
            pass

   def _tick(self,arg):          # Seconds counter
      self._secs += 1

   async def _poll(self):        # Polling routine
      while True:
         self._n += 1
         for n in range(self.n):
            self.chn[n] = not self.fbpin[n].value()
            if self.state[n] != self.chn[n]:
               self.flag.set()
         await asyncio.sleep_ms(SETTLE_TIME)

   def _intr(self,pin):          # Interrupt handler
      self._n += 1
      self.chn[pin] = not self.fbpin[pin].value()
      if self.chn[pin] != self.state[pin]:
         self.flag.set()         # Signal waiting task in run()

   def _check(self,pin):
      # Activated after change of state of feedback channel by the timer

      val = not self.fbpin[pin].value()  # Current channel state
      if self.state[pin] != val:    # Internal state same as present pin state?
         self.chn[pin] = val        # No - generate a feedback event
         pkt = self.pool[pin]
         pkt[9], pkt[10] = self.state[pin], self.chn[pin]
         self.callback(pkt)         # Invoke callback with packet
      self.ptmr[pin] = 0            # Handled channel, schedule next timer
      self.state[pin] = self.chn[pin]

   def stop(self):
      self.ticker.deinit()
      if feedback.interrupt:
         for i in range(self.n): self.fbpin[i].irq(handler=None)
      else:
         try:
            self.task.cancel()
         except:
            pass
      
   async def run(self,proc):
      self.callback = proc

      # Initialized.  Fall into processing loop.
      # Debouncing of channel signal is handled two ways:
      # 1) in interrupt routine, flag raised only if there is a state change on
      #    the pin; similarly in the poll routine.
      # 2) after a state change, a timer is set for a SETTLE_TIME delay to see
      #    whether the pin state changed.  If the pin changes in this interval,
      #    it is only recognized if it results in a change in state.

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

   @property
   def state_packet(self):       # provide state packet
      return self.fbpp

   @property
   def stats(self):              # provide interrupt stats
      return (self._n, self._secs)

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

qfCT, qfDB = False, False
TCPmsg = const('''
              ***********************
              ***TCP output q full***
              ***********************
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

   try:
      queue.put_sync(pkt)
      return False
   except:
      if not flag: print(msg)
      return True

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
      buf = TtoC[ixTC % QSIZE]
      buf[0:CS2_SIZE] = pkt
      await TCPtoCAN.put(buf)
      ixTC += 1
      buf = DBQ[ixDB % QSIZE]
      buf[0:CS2_SIZE] = pkt
      await debugQUE.put(buf)
      ixDB += 1

      #  Quick command parse for special responses
      cmd = int.from_bytes(pkt[0:2]) >> 1 & 0xff
      sub = int(pkt[9]) if pkt[4] > 4 else -1
      rsp = pkt[1] & 0x01
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

can = iCAN(_CANBOARD)
async def CAN_READER():
   print(
      "CAN %s board initialized successfully, waiting for traffic." %
      can.pins.name
   )
   can.run(CAN_IN)

def CAN_IN(msg, err, buf=bytearray(CS2_SIZE)):
   from canbus import CanError
   global ixCT, ixDB, qfCT, qfDB
   if err:
      stat = can.intf.getStatus()     # Order matters: status first ...
      intr = can.intf.getInterrupts() # ...then interrupt reg
      errf = can.intf.getErrorFlags()
      print('   >>>CAN read error<<< stat %02x %s intr %02x %s err %02x %s' % (
         stat, CanError.decode(status=stat),
         intr, CanError.decode(interrupt=intr),
         errf, CanError.decode(error=errf)
      ))
      return
   buf[0] = msg.can_id >> 24 & 0xff 
   buf[1] = msg.can_id >> 16 & 0xff 
   buf[2] = msg.can_id >>  8 & 0xff 
   buf[3] = msg.can_id       & 0xff
   buf[4] = msg.dlc
   buf[5:5+msg.dlc] = msg.data
   buf[5+msg.dlc:CS2_SIZE] = (8-msg.dlc)*b'\x00'
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

async def CAN_WRITER(MERR=5, MCNT=500):
   from canbus import CanError
   async for pkt in TCPtoCAN:
      assert len(pkt) == CS2_SIZE
      cnt = 0
      while can.send(
            ID=int.from_bytes(pkt[0:4]),
            data=pkt[5:5+int(pkt[4])],
            EFF=True
      ):
         cnt += 1
         errf = can.intf.getErrorFlags() # Error Flag register
         if cnt <= MERR:
            print('   >>>CAN write error<<< (err %02x %s)%s' % 
               (errf, CanError.decode(error=errf),
                  ' - quelling further reports' if cnt >= MERR else '')
            )
         if errf & 0x30:         # TXBO/Bus-Off or TXEP/TX-Passive
            can.intf.clearErrorFlags(MERR=True)
         await asyncio.sleep_ms(10)
         if cnt > MCNT:          # Abandon packet after this many tries
            break

async def DEBUG_OUT():
   global rrhash, dec
   async for buf in debugQUE:
      assert len(buf) == CS2_SIZE
      data = '%04x %04x %02x %s' % (
         int.from_bytes(buf[0:2]), int.from_bytes(buf[2:4]),
         buf[4],
         ' '.join(map(''.join, zip(*[iter(buf[5:].hex())]*4)))
      )
      cid = int.from_bytes(buf[2:4])
      if cid == rrhash:
         print('TCP -> CAN', data)
      else:
         print('CAN -> TCP', data)
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

   await fdbk.run(post)

async def RUNNER(ip):
   srvr = await asyncio.start_server(TCP_SERVER,ip,CS2_PORT)
   while True:
      await asyncio.sleep(60)

print('TCP <-> CAN packet hub (%s)' % _VER)
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
tcpr = asyncio.create_task(TCP_READER())
tcpw = asyncio.create_task(TCP_WRITER())
canw = asyncio.create_task(CAN_WRITER())
dbug = asyncio.create_task(DEBUG_OUT())
beat = asyncio.create_task(HEARTBEAT())
feed = asyncio.create_task(FEEDBACK())
runr = asyncio.create_task(RUNNER(ip))

try:
   Loop.run_forever()
except KeyboardInterrupt:
   can.stop()
   stats = fdbk.stats
   fdbk.stop()
   print('Feedback interrupts: %d, %.2f/sec' % (stats[0], stats[0]/stats[1]))
