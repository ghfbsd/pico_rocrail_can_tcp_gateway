# UDB - CAN bus hub

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)

# Runs with:
# MicroPython v1.24.1 on 2024-11-29; Raspberry Pi Pico W with RP2040

# 09 Mar. 2025
# last revision 09 Mar. 2025

VER = 'AR095'                    # version ID

SSID = "****"
PASS = "****"

CS2_SPORT = 15730                # Marklin diktat
CS2_RPORT = 15731                # Marklin diktat
CS2_SIZE = 13                    # Fixed by protocol definition

QSIZE = 25                       # Size of various I/O queues (overkill)

INT_PIN = 20                     # Interrupt pin for CAN board

import uasyncio as asyncio
import network
import usocket as socket
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

CANtoUDP = ThreadSafeQueue(QSIZE*[bytearray(CS2_SIZE)])
UDPtoCAN = ThreadSafeQueue(QSIZE*[bytearray(CS2_SIZE)])
debugQUE = ThreadSafeQueue(QSIZE*[bytearray(CS2_SIZE)])

rrhash = 0

async def UDP_READER(timeout=0):
   # packet layout:
   #    xx xx xx xx  xx  xx xx xx xx xx xx xx xx  -  13 bytes total = CS2_SIZE
   #    -----------  --  -----------------------
   #       CAN ID    len  data (left justified)
   import uselect as select
   global ip, rrhash

   s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   s.setblocking(False)
   s.bind((ip,CS2_RPORT))
   p = select.poll()
   p.register(s, select.POLLIN)
   print('UDP listening, waiting for traffic.')

   while True:                   # Wait for activity on port
      if p.poll(timeout):
         pkt, addr = s.recvfrom(CS2_SIZE)
         assert len(pkt) == CS2_SIZE
         rrhash = int.from_bytes(pkt[2:4])
      #  await UDPtoCAN.put(pkt)    # might miss incoming packet here?
      #  await debugQUE.put(pkt)    # might miss incoming packet here?
         UDPtoCAN.put_sync(pkt)     # put_sync because no await used
         debugQUE.put_sync(pkt)     # put_sync because no await used
         continue
      await asyncio.sleep(0)

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
      CANtoUDP.put_sync(buf)     # put_sync because no await used
   except:
      print('***CANtoUDP q full')
   try:
      debugQUE.put_sync(buf)     # pub_sync because no await used
   except:
      print('***debug q full')

async def UDP_WRITER():
   global ip
                                 # set up for UDP use
   s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
   s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
   s.bind((ip,0))                # assumed to be only one interface

   while True:
      async for pkt in CANtoUDP:
         assert len(pkt) == CS2_SIZE
         s.sendto(pkt, ('255.255.255.255',CS2_SPORT))

async def CAN_WRITER(MERR=5, MCNT=500):
   global can
   async for pkt in UDPtoCAN:
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
         print('UDP -> CAN %s' % data)
      else:
         print('CAN -> UDP %s' % data)
      if avail:
         print('   %s' % decode(
               int.from_bytes(buf[0:4]), buf[5:5+int(buf[4])], detail=True
            )
         )
      if UDPtoCAN.qsize() > 0 and UDPtoCAN.qsize() % 5 == 0:
         print('UDPtoCAN queue congestion: %d waiting' % UDPtoCAN.qsize())
      if CANtoUDP.qsize() > 0 and CANtoUDP.qsize() % 5 == 0:
         print('CANtoUDP queue congestion: %d waiting' % CANtoUDP.qsize())

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

print('UDP <-> CAN packet hub (%s)' % VER)
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
print('Available at {} as {}.'.format(ip,host))

canr = asyncio.create_task(CAN_READER())
udpr = asyncio.create_task(UDP_READER())
udpw = asyncio.create_task(UDP_WRITER())
canw = asyncio.create_task(CAN_WRITER())
dbug = asyncio.create_task(DEBUG_OUT())
beat = asyncio.create_task(HEARTBEAT())

Loop.run_until_complete(udpr)
