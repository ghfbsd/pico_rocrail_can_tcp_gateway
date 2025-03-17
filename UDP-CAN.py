# UDB - CAN bus hub (without asyncio)

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# CPU: Raspberry Pi pico W
# CAN device: RB-P-CAN-RS485 board by Joy-IT (www.joy-it.net)
#          or Pico-CAN-B board by Waveshare (https://www.waveshare.com)

# Runs with:
# MicroPython v1.24.1 on 2024-11-29; Raspberry Pi Pico W with RP2040

# 15 Mar. 2025
# last revision 17 Mar. 2025

import network
import usocket as socket
import sys
from time import sleep, sleep_ms, ticks_diff, ticks_us

VER = 'AR175'                    # version ID

SSID = "****"
PASS = "****"

CS2_SPORT = const(15730)         # Marklin diktat
CS2_RPORT = const(15731)         # Marklin diktat
CS2_SIZE = const(13)             # Fixed by protocol definition

CAN_MERR = const(5)              # CAN error processing
CAN_MCNT = const(500)            # CAN error processing

QSIZE = const(25)                # Size of various I/O queues (overkill)

LOGP = const(False)              # Abbreviated log for debug
LOGM = const(False)              # Full log (use with caution - packet loss)

CANBOARD = 'JI'

if CANBOARD == 'JI':
   # These pin assignments are appropriate for a RB-P-CAN-485 Joy-IT board
   INT_PIN = 20                  # Interrupt pin for CAN board
   SPI_CS = 17
   SPI_SCK = 18
   SPI_MOSI = 19
   SPI_MISO = 16
elif CANBOARD == 'WS':
   # These pin assignments are appropriate for a Waveshare Pico-CAN-B board
   INT_PIN = 21                  # Interrupt pin for CAN board
   SPI_CS = 5
   SPI_SCK = 6
   SPI_MOSI = 7
   SPI_MISO = 4
else:
   raise RuntimeError('***%s is an unsupported CAN board***' % CANBOARD)

from micropython import alloc_emergency_exception_buf as AEEB
AEEB(100)                        # boilerplate for IRQ-level exception reporting

from machine import Pin
pico_led = Pin("LED", Pin.OUT)

class iCAN:                      # interrupt driven CAN controller
   from canbus import Can, CanError
   from canbus.internal import CAN_SPEED
   from machine import Pin, SPI

   prep = SPI(0,                 # configure SPI to use correct pins
      sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO)
   )

   def __init__(self, intr=None):
      from canbus import Can, CanError
      from canbus.internal import CAN_SPEED
      from machine import Pin
      if intr is None:
         raise RuntimeError('Need to provide CAN interrupt pin')
      self.flag = False
      self.pin = Pin(intr, Pin.IN, Pin.PULL_UP)
      self.pin.irq(              # this defines the interrupt handler (lambda)
         trigger=Pin.IRQ_FALLING,
         handler=self.set_flag,
         hard=True)
      self.can = Can(spics=SPI_CS) # CS pin for hardware SPI 0
      # Initialize the CAN interface.  Reference says 250 kbps speed.
      ret = self.can.begin(bitrate=CAN_SPEED.CAN_250KBPS)
      if ret != CanError.ERROR_OK:
         raise RuntimeError('Error initializing CAN bus')
      print("CAN initialized successfully, waiting for traffic.")

   def set_flag(self,pin):
      self.flag = True

   def intf(self):               # hardware interface level access if needed
      return self.can

   def intr(self):
      from canbus import CanError
      if not self.flag:          # interrupt?
         return None

      iflg = self.can.getInterrupts()
      if iflg & 0x03 == 0:       # ignore interrupts except for reading
         self.flag = False
         self.can.clearInterrupts()
         return None

      if self.can.checkReceive():
         # CAN error code and message
         error, msg = self.can.recv()
         return (msg, error != CanError.ERROR_OK)

      self.flag = False
      self.can.clearInterrupts()
      return None

   def cont(self):               # check for more queued I/O after interrupt
      from canbus import CanError
      if not self.flag:
         raise RuntimeError('Interrupt not pending on CAN bus')
         
      if self.can.checkReceive():
         # CAN error code and message
         error, msg = self.can.recv()
         return (msg, error != CanError.ERROR_OK)

      self.flag = False
      self.can.clearInterrupts()
      return None

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

def DEBUG_OUT(buf):
      global rrhash, avail

      assert len(buf) == CS2_SIZE
      data = ' '.join(           # put space between every octet
         map(''.join, zip(*[iter(buf.hex())]*2))
      )
      data = '%04x %04x %02x %s' % (
         int.from_bytes(buf[0:2]), int.from_bytes(buf[2:4]),
         buf[4],
         ' '.join(map(''.join, zip(*[iter(buf[5:].hex())]*4)))
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

print('UDP <-> CAN packet hub (%s)' % VER)
try:
   from marklin import decode    # Marklin CS2 CAN packet decoder
   avail = LOGM                  # Conditionally use it
except:
   avail = False                 # don't use it if not available
if not avail:
   print('No Märklin packet decoding, only logging raw data.')

#Connect to WLAN; power management on chip has to be turned off in order to not
#  drop UDP packets; see https://forums.raspberrypi.com/viewtopic.php?t=365691
wlan = network.WLAN(network.STA_IF)
if not wlan.isconnected():
   wlan.active(True)
   mac = wlan.config('mac')
   host = 'CS2hub-' + ''.join('{:02x}'.format(b) for b in mac[3:])
   wlan.config(hostname = host)
   wlan.connect(SSID, PASS)
else:
   host = wlan.config('hostname')
wlan.config(pm=0xA11140)         # disable power management on chip

while not wlan.isconnected():
   # Fast flash while waiting for wifi connection; boot button restarts.
   if rp2.bootsel_button() == 1: sys.exit()

   pico_led.on()
   sleep(0.25)
   pico_led.off()
   sleep(0.25)

ip = wlan.ifconfig()[0]
print('Available at {} as {}.'.format(ip,host))

                                  # set up read socket for UDP use
sckr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sckr.setblocking(False)
sckr.bind(('0.0.0.0',CS2_RPORT))
                                  # set up write socket for UDP use
sckw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sckw.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sckw.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sckw.bind(('0.0.0.0',0))          # assumed to be only one interface

print('UDP listening, waiting for traffic.')

cti, ctb = 0, 0
cpkt, cbuf = QSIZE*[bytearray(CS2_SIZE)], QSIZE*[bytearray(CS2_SIZE)]

can = iCAN(intr=INT_PIN)

t0 = ticks_us()

while True:
   # packet layout:
   #    xx xx xx xx  xx  xx xx xx xx xx xx xx xx  -  13 bytes total = CS2_SIZE
   #    -----------  --  -----------------------
   #       CAN ID    len  data (left justified)

   pkt = cpkt[cti % QSIZE]
   n = sckr.readinto(pkt, CS2_SIZE)
   if n is not None:             # First test: check for incoming packets
      assert n == CS2_SIZE
      mem = memoryview(pkt)
      rrhash = int.from_bytes(mem[2:4])
      debugQUE.put_sync(pkt)     # put_sync because no await used

      if LOGP:
         comm = int.from_bytes(mem[0:2]) >> 1 & 0xff
         sub = int(pkt[9])
         desc = '???'
         if comm == 0x04: desc = 'LSP'
         if comm == 0x05: desc = 'LDI'
         if comm == 0x06: desc = 'LFU'
         if comm == 0x0B: desc = 'ACC'
         if comm == 0x18: desc = 'PNG'
         if comm == 0x00 and sub == 0: desc = 'STP'
         if comm == 0x00 and sub == 1: desc = 'SGO'
         if comm == 0x00 and sub == 2: desc = 'SHL'
         if comm == 0x00 and sub == 32: desc = 'CLK'
         dtus = ticks_diff(ticks_us(),t0)
         print('%4d.%06d < %s %04x %04x %02x %s' % (
            dtus//1000000, dtus%1000000,
            desc,
            int.from_bytes(mem[0:2]), int.from_bytes(mem[2:4]),
            pkt[4],
            ' '.join(map(''.join, zip(*[iter(mem[5:].hex())]*4)))
         ))

      cti += 1
      if can.send(               # Try immediate send of packet
            ID=int.from_bytes(pkt[0:4]),
            data=pkt[5:5+int(pkt[4])],
            EFF=True
      ):
         UDPtoCAN.put_sync(pkt)  # Failed? queue it
      continue

   any, tst = False, can.intr()
   while tst is not None:        # Second test: check for incoming CAN msgs
      msg, err = tst
      if err:
         stat = can.intf().getStatus()     # Order matters: status first ...
         intr = can.intf().getInterrupts() # ...then interrupt reg
         errf = can.intf().getErrorFlags()
         print('   >>>CAN read error<<< stat %02x intr %02x err %02x' %
            (stat,intr,errf)
         )
      else:
         buf = cbuf[ctb % QSIZE]
         buf[0] = msg.can_id >> 24 & 0xff 
         buf[1] = msg.can_id >> 16 & 0xff 
         buf[2] = msg.can_id >>  8 & 0xff 
         buf[3] = msg.can_id       & 0xff
         buf[4] = msg.dlc
         buf[5:14] = 8*b'\x00'
         for i in range(msg.dlc): buf[5 + i] = msg.data[i]

         CANtoUDP.put_sync(buf)  # put_sync because no await used
         debugQUE.put_sync(buf)  # put_sync because no await used

         if LOGP:                # partial logging
            comm = msg.can_id >> 17 & 0xff
            sub = int(buf[9])
            desc = '???'
            if comm == 0x04: desc = 'LSP'
            if comm == 0x05: desc = 'LDI'
            if comm == 0x06: desc = 'LFU'
            if comm == 0x0B: desc = 'ACC'
            if comm == 0x18: desc = 'PNG'
            if comm == 0x00 and sub == 0: desc = 'STP'
            if comm == 0x00 and sub == 1: desc = 'SGO'
            if comm == 0x00 and sub == 2: desc = 'SHL'
            if comm == 0x00 and sub == 32: desc = 'CLK'
            dtus = ticks_diff(ticks_us(),t0)
            print('%4d.%06d > %s %04x %04x %02x %s' % (
               dtus//1000000, dtus%1000000,
               desc,
               int.from_bytes(buf[0:2]), int.from_bytes(buf[2:4]),
               buf[4],
               ' '.join(map(''.join, zip(*[iter(buf[5:].hex())]*4)))
            ))

         any = True
         ctb += 1
      tst = can.cont()
   if any: continue

   if UDPtoCAN.qsize() > 0:      # Third test: Put out a packet on the CAN bus
      pkt = UDPtoCAN.get_sync()
      cnt = 0
      while can.send(
            ID=int.from_bytes(pkt[0:4]),
            data=pkt[5:5+int(pkt[4])],
            EFF=True
      ):
         cnt += 1
         errf = can.intf().getErrorFlags() # Error Flag register
         if cnt <= CAN_MERR:
            print('   >>>CAN write error<<< (err %02x)%s' % 
               (errf, ' - quelling further reports' if cnt >= CAN_MERR else '')
            )
         if errf & 0x30:         # TXBO/Bus-Off or TXEP/TX-Passive
            can.intf().clearErrorFlags(MERR=True)
         sleep_ms(10)
         if cnt > CAN_MCNT:      # Abandon packet after this many tries
            break
      continue

   if CANtoUDP.qsize() > 0:      # Fourth test: Put out a packet on Wifi
      pkt = CANtoUDP.get_sync()
      sckw.sendto(pkt, ('255.255.255.255',CS2_SPORT))
      continue

   if debugQUE.qsize() > 0:      # Fifth task: Put out debug info
      DEBUG_OUT(debugQUE.get_sync())
      continue

                                 # Housekeeping
   if UDPtoCAN.qsize() > 0 and UDPtoCAN.qsize() % 5 == 0:
      print('UDPtoCAN queue congestion: %d waiting' % UDPtoCAN.qsize())
   if CANtoUDP.qsize() > 0 and CANtoUDP.qsize() % 5 == 0:
      print('CANtoUDP queue congestion: %d waiting' % CANtoUDP.qsize())

   td = ticks_diff(ticks_us(),t0)
   if (td//1000) % 2:            # ... heartbeat
      pico_led.on()
   else:
      pico_led.off()

   if rp2.bootsel_button():      # Panic button
      sys.exit()
