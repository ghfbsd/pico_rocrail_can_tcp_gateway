# TCP - CAN bus hub

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)
#          or Pico-CAN-B board by Waveshare (https://www.waveshare.com)

# Runs with:
# MicroPython v1.24.1 on 2024-11-29; Raspberry Pi Pico W with RP2040

# 16 Jan. 2025
# last revision 21 Apr. 2025

_VER = const('PR215')            # version ID

SSID = "****"
PASS = "****"

CS2_PORT = const(15731)          # Marklin diktat
CS2_SIZE = const(13)             # Fixed by protocol definition

QSIZE = const(25)                # Size of various I/O queues (overkill)

NODE_ID = const(1)               # "S88" node ID

_CANBOARD = const('WS')

if _CANBOARD == 'JI':
   # These pin assignments are appropriate for a RB-P-CAN-485 Joy-IT board
   INT_PIN = 20                  # Interrupt pin for CAN board
   SPI_CS = 17
   SPI_SCK = 18
   SPI_MOSI = 19
   SPI_MISO = 16
   FBP = dict(                   # Feedback pins
      #   +---- channel number
      #   |  +- GPIO pin number
      #   |  |
      #   v  v
      c0=(0, 0), c1=(1, 1), c2=(2, 8), c3=(3, 9),
      c4=(4,10), c5=(5,11), c6=(6,14), c7=(7,15)
   )
elif _CANBOARD == 'WS':
   # These pin assignments are appropriate for a Waveshare Pico-CAN-B board
   INT_PIN = 21                  # Interrupt pin for CAN board
   SPI_CS = 5
   SPI_SCK = 6
   SPI_MOSI = 7
   SPI_MISO = 4
   FBP = dict(                   # Feedback pins
      #   +---- channel number
      #   |  +- GPIO pin number
      #   |  |
      #   v  v
      c0=(0, 0), c1=(1, 1), c2=(2, 2), c3=(3, 3),
      c4=(4,10), c5=(5,11), c6=(6,12), c7=(7,13)
   )
else:
   raise RuntimeError('***%s is an unsupported CAN board***' % _CANBOARD)

import uasyncio as asyncio
import network
import sys
from time import sleep

from micropython import alloc_emergency_exception_buf as AEEB
AEEB(100)                        # boilerplate for IRQ-level exception reporting

from machine import Pin
pico_led = Pin("LED", Pin.OUT)

class iCAN:                      # interrupt driven CAN message sniffer
   from canbus import Can, CanError
   from canbus.internal import CAN_SPEED
   from machine import Pin, SPI

   prep = SPI(0,                 # configure SPI to use correct pins
      sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO)
   )

   def __init__(self, intr=None):
      from machine import Pin
      from canbus import Can, CanError
      from canbus.internal import CAN_SPEED

      if intr is None:
         raise RuntimeError('Need to provide CAN interrupt pin')
      self.flag = asyncio.ThreadSafeFlag()
      self.pin = Pin(intr, Pin.IN, Pin.PULL_UP)
      self.pin.irq(              # this defines the interrupt handler (lambda)
         trigger=Pin.IRQ_FALLING,
         handler=lambda pin: self.flag.set(),
         hard=True)
      self.can = Can(spics=SPI_CS) # CS pin for hardware SPI 0
      # Initialize the CAN interface.  Reference says 250 kbps speed.
      ret = self.can.begin(bitrate=CAN_SPEED.CAN_250KBPS)
      if ret != CanError.ERROR_OK:
         raise RuntimeError('Error initializing CAN bus')
      print("CAN initialized successfully, waiting for traffic.")

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

from threadsafe import ThreadSafeQueue

ixCT, ixTC, ixDB = 0, 0, 0
qfCT, qfDB = False, False
CtoT, TtoC, DBQ = [], [], []
for i in range(QSIZE):
   CtoT.append(bytearray(CS2_SIZE))
   TtoC.append(bytearray(CS2_SIZE))
   DBQ.append(bytearray(CS2_SIZE))
CANtoTCP = ThreadSafeQueue(QSIZE)
TCPtoCAN = ThreadSafeQueue(QSIZE)
debugQUE = ThreadSafeQueue(QSIZE)

TCP_R, TCP_W = None, None
TCP_RERR, TCP_WERR = False, False

rrhash = 0

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
   global TCP_R, TCP_RERR, rrhash, ixTC, ixDB, fbis
   while True:                   # Wait for connection
      if TCP_R is None:
         await asyncio.sleep_ms(10)
         continue

      try:                       # Serve it
         pkt = await TCP_R.readexactly(CS2_SIZE)
      except EOFError:           # Connection lost/client disconnect
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

      #  Check for S88 state poll
      cmd = int.from_bytes(pkt[0:2]) >> 1 & 0xff
      sub = int(pkt[9]) if pkt[4] > 4 else -1
      if cmd == 0x10 and sub == NODE_ID:
         await CANtoTCP.put(fbis)
         await debugQUE.put(fbis)

async def CAN_READER():
   global can, INT_PIN
   can = iCAN(intr=INT_PIN)
   can.run(CAN_IN)

def CAN_IN(msg, err, buf=bytearray(CS2_SIZE)):
   global ixCT, ixDB, qfCT, qfDB
   if err:
      stat = can.intf().getStatus()     # Order matters: status first ...
      intr = can.intf().getInterrupts() # ...then interrupt reg
      errf = can.intf().getErrorFlags()
      print('   >>>CAN read error<<< stat %02x intr %02x err %02x' %
         (stat,intr,errf)
      )
      return
   buf[0] = msg.can_id >> 24 & 0xff 
   buf[1] = msg.can_id >> 16 & 0xff 
   buf[2] = msg.can_id >>  8 & 0xff 
   buf[3] = msg.can_id       & 0xff
   buf[4] = msg.dlc
   buf[5:14] = 8*b'\x00'
   for i in range(msg.dlc): buf[5 + i] = msg.data[i]
   pkt = CtoT[ixCT % QSIZE]
   pkt[0:CS2_SIZE] = buf
   try:
      CANtoTCP.put_sync(pkt)     # put_sync because no await used
      qfCT = False
      ixCT += 1
   except:
      if not qfCT: print('***CANtoTCP q full')
      qfCT = True
   pkt = DBQ[ixDB % QSIZE]
   pkt[0:CS2_SIZE] = buf
   try:
      debugQUE.put_sync(pkt)     # put_sync because no await used
      qfDB = False
      ixDB += 1
   except:
      if not qfDB: print('***debug q full')
      qfDB = True

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
   global can
   async for pkt in TCPtoCAN:
      assert len(pkt) == CS2_SIZE
      cnt = 0
      while can.send(
            ID=int.from_bytes(pkt[0:4]),
            data=pkt[5:5+int(pkt[4])],
            EFF=True
      ):
         cnt += 1
         errf = can.intf().getErrorFlags() # Error Flag register
         if cnt <= MERR:
            print('   >>>CAN write error<<< (err %02x)%s' % 
               (errf, ' - quelling further reports' if cnt >= MERR else '')
            )
         if errf & 0x30:         # TXBO/Bus-Off or TXEP/TX-Passive
            can.intf().clearErrorFlags(MERR=True)
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

fbis = bytearray(CS2_SIZE)       # Feedback initial state packet

async def FEEDBACK():
   # Feedback via subset of pins
   global qfCT, qfDB, fbis
   chn, state = bytearray(8), bytearray(8)
   myhash = 0x5338

   flag = asyncio.ThreadSafeFlag()
   def intr(pin):                # Interrupt handler
      chn[pin] ^= 1              # Toggle state
      flag.set()                 # Signal waiting task

   pool, fbpin, n = 8*[None], 8*[None], 0
   for pid in FBP:
      ch, pin = FBP[pid]
      chn[ch], state[ch] = 1, 1
      fbpin[ch] = Pin(pin, Pin.IN, Pin.PULL_UP)
      fbpin[ch].irq(             # this defines the interrupt handler
         trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
         handler=lambda p, id=ch: intr(id),
         hard=True)
      val = not fbpin[ch].value()# Get present state to initialize
      chn[ch], state[ch] = val, val
      pool[ch] = bytearray(CS2_SIZE)
      pkt = pool[ch]             # Allocate buffer and format static CS2 packet
      pkt[0] = 0x11 >> 7 & 0xff
      pkt[1] = 0x11 << 1 & 0xff | 0x01   # set R flag
      pkt[2] = myhash >> 8 & 0xff
      pkt[3] = myhash & 0xff
      pkt[4] = 8
      pkt[5:7] = bytes((0,NODE_ID))
      pkt[7:9] = bytes((0,ch))
      pkt[9], pkt[10] = state[ch], chn[ch]
      pkt[11:13] = 2*b'\x00'
      n += 1

   val = 0
   for i in range(8): val |= state[i] << i
   fbis[0] = 0x10 >> 7 & 0xff    # Build initial state packet
   fbis[1] = 0x10 << 1 | 0x01    # set R flag
   fbis[2] = myhash >> 8 & 0xff
   fbis[3] = myhash & 0xff
   fbis[4] = 7
   fbis[5:9] = bytes((0x53,0x30,0,NODE_ID))
   fbis[9] = NODE_ID
   fbis[10], fbis[11] = 0, val
   fbis[12] = 0                  # This will be fired off after an S88 POLL cmd

   print('Feedback: %d channels, "S88" module %d.' % (n,NODE_ID))
   asyncio.sleep(0)
   while True:
      await flag.wait()
      chg = 0
      for i in range(n):
         val = not fbpin[i].value()  # Get present state to debounce
         if state[i] != chn[i] or state[i] != val:
            chg |= 1 << i
            chn[i] = val         # Make sure internal state agrees
      if chg:                    # Respond with current status
         val = 0
         for i in range(n):
            if chg & (1 << i):
               pkt = pool[i]
               pkt[9], pkt[10] = state[i], chn[i]
               try:
                  CANtoTCP.put_sync(pkt)
                  qfCT = False
               except:
                  if not qfCT: print('***CAN q full')
                  qfCT = True
               try:
                  debugQUE.put_sync(pkt)
                  qfDB = False
               except:
                  if not qfDB: print('***log q full')
                  qfDB = True
               state[i] = chn[i]
               val |= chn[i] << i
            if state[i] == fbpin[i].value():
               print('***contact %d state mismatch (corrected)' % i)
         fbis[11] = val          # Maintain state in poll packet

from asyncio import Loop

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

try:
   Loop.run_until_complete(asyncio.start_server(TCP_SERVER,ip,CS2_PORT))
except KeyboardInterrupt:
   can.stop()
