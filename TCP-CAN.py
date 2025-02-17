# TCP - CAN bus hub

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)

# Runs with:
# MicroPython v1.24.1 on 2024-11-29; Raspberry Pi Pico W with RP2040

# 16 Jan. 2025
# last revision 17 Feb. 2025

VER = 'EB175'                    # version ID

SSID = "****"
PASS = "****"

CS2_PORT = 15731                 # Marklin diktat
CS2_SIZE = 13                    # Fixed by protocol definition

QSIZE = 25                       # Size of various I/O queues (overkill)

INT_PIN = 20                     # Interrupt pin for CAN board

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

   # These pin assignments are appropriate for RB-P-CAN-485 Joy-IT board
   SPI_CS = 17
   SPI_SCK = 18
   SPI_MOSI = 19
   SPI_MISO = 16

   # These should work with a Waveshare Pico-CAN-B board; use with INT_PIN = 21
   # SPI_CS = 5                  # untested
   # SPI_SCK = 6                 # untested
   # SPI_MOSI = 7                # untested
   # SPI_MISO = 4                # untested


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
      self.can = Can(spics=self.SPI_CS) # CS pin for hardware SPI 0
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

CANtoTCP = ThreadSafeQueue(QSIZE*[bytearray(CS2_SIZE)])
TCPtoCAN = ThreadSafeQueue(QSIZE*[bytearray(CS2_SIZE)])
debugQUE = ThreadSafeQueue(QSIZE*[bytearray(CS2_SIZE)])

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
   global TCP_R, TCP_RERR, rrhash
   while True:                   # Wait for connection
      if TCP_R is None:
         await asyncio.sleep_ms(10)
         continue

      try:                       # Serve it
         pkt = await TCP_R.readexactly(CS2_SIZE)
         assert len(pkt) == CS2_SIZE
         rrhash = int.from_bytes(pkt[2:4])
         await TCPtoCAN.put(pkt)
         await debugQUE.put(pkt)
      except EOFError:           # Connection lost/client disconnect
         TCP_RERR, TCP_R = True, None
         continue

async def CAN_READER():
   global can, INT_PIN
   can = iCAN(intr=INT_PIN)
   can.run(CAN_IN)

def CAN_IN(msg, err, buf=bytearray(CS2_SIZE)):
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
   buf[5:14] = 8*b'0'
   for i in range(msg.dlc): buf[5 + i] = msg.data[i]
   try:
      CANtoTCP.put_sync(buf)     # put_sync because no await used
   except:
      print('***CANtoTCP q full')
   try:
      debugQUE.put_sync(buf)     # pub_sync because no await used
   except:
      print('***debug q full')

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
         asyncio.sleep_ms(10)
         if cnt > MCNT:          # Abandon packet after this many tries
            break

async def DEBUG_OUT():
   global rrhash, avail
   async for buf in debugQUE:
      assert len(buf) == CS2_SIZE
      data = ' '.join(           # put space between every octet
         map(''.join, zip(*[iter(buf.hex())]*2))
      )
      cid = int.from_bytes(buf[2:4])
      if cid == rrhash:
         print('TCP -> CAN %s' % data)
      else:
         print('CAN -> TCP %s' % data)
      if avail:
         print('   %s' % decode(
               int.from_bytes(buf[0:4]), buf[5:5+int(buf[4])], detail=True
            )
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

from asyncio import Loop

print('TCP <-> CAN packet hub (%s)' % VER)
try:
   from marklin import decode    # Marklin CS2 CAN packet decoder
   avail = True
except:
   avail = False                 # don't use it if not available
   print('No Märklin packet decoding, only logging raw data.')

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

Loop.run_until_complete(asyncio.start_server(TCP_SERVER,ip,CS2_PORT))
