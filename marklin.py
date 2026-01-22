# Decodes CAN packet according to Marklin protocol

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# See https://github.com/ghfbsd

class CS2decoder:
   channel = dict()              # Built-in values for 60116 Gleisbox

   # fbx x=1..4: RGB color encoded rrggbbxx (2 bits each color, xx unused)
   # ebx x=1..4: value at end of range 1..4 in given units

   channel[1] = ("TRACK","A", 0.,  2.5,
   # pow  fb1 fb2 fb3 fb4 zero  lim  eb1  eb2  eb3  eb4
      -3,  48,240,224,192,  15,2060,1648,1730,1895,2060
   )
   channel[3] = ("VOLT", "V", 10., 27.,
      -3, 192, 12, 48,192,   0,3145, 925,1202,2590,3145
   )
   channel[4] = ("TEMP", "C",  0., 80.,
       0,  12,  8,240,192,   0, 219, 107, 164, 205, 219
   )

   def __init__(self,detail=False,pfx='',print=False):
      self.detail = detail
      self.pfx = pfx
      self.print = print

   def _CRC(byt,acc):
      acc = (acc ^ (byt << 8)) & 0xffff
      for i in range(8):
         if acc & 0x8000 == 0x8000:
            acc = (acc << 1) ^ 0x1021
         else:
            acc <<= 1
         acc &= 0xffff
      return acc

   def decode(self,ID,data):

      detail = self.detail
      comm = ID >> 17 & 0xff
      sub = data[4] if len(data) > 4 else -1
      resp = ID & 0x00010000
      dlc = len(data)

      if comm == 0x00 and sub == 0x0b and resp and detail:
         if resp and dlc != 8:
            return decode(ID, data, detail)
         chan = int(data[5])
         if chan not in self.channel:
            return decode(ID, data, detail)
         (name, unit, cmin, cmax, pow,
            fb1, fb2, fb3, fb4, zero, lim,
            eb1, eb2, eb3, eb4) = self.channel[chan]
         val = (cmax - cmin) / (lim - zero)
         val = val * (int.from_bytes(data[6:8],'big') - zero) + cmin
         mess = 'R' if resp else 'C'
         mess += ' SYSTEM STATUS 00/0B (channel %d)' % chan
         mess += ' %s %.2f%s' % (name, val, unit)
      elif comm == 0x21 and detail:
         mess = ('R' if resp else 'C') + ' CONFIG DATA STREAM'
         if resp or dlc not in [6,7,8]:
            mess += ' (garbled)'
         if dlc == 7 or dlc == 6:
            self.cnt = int.from_bytes(data[0:4],'big')
            self.val = int.from_bytes(data[4:6],'big')
            mess += ' (start: count %d, CRC %04x)' % (
               self.cnt, self.val
            )
            self.txt, self.crc = '', 0xffff
         elif dlc == 8:
            _CRC = CS2decoder._CRC
            for c in data:
               self.crc = _CRC(c,self.crc)
            self.txt += data.decode('utf-8')
            self.cnt -= 8

            if self.cnt <= 0:
               mess += ' (end)'
               if self.val != self.crc:
                  mess += ' (invalid CRC)'
               if self.cnt != 0:
                  mess += ' (incorrect count)'
               mess += '  text:\n' + self.txt
      else:
         mess = decode(ID, data, detail)

      if self.print: print('%s%s' % (self.pfx,mess))
      return mess


def decode(ID,data,detail=False) -> str:
   hash = ID & 0xffff
   resp = 'R' if ID & 0x00010000 else 'C'
   comm = ID >> 17 & 0xff
   prio = ID >> 25 & 0x03
   dlen = len(data)
   dhex = ' '.join(           # put space between every octet
       map(''.join, zip(*[iter(data.hex())]*2))
   )
   gen = '%s %02x hash %04x prio: %d data: %s' % (
          resp, comm, hash, prio, dhex if dlen > 0 else ''
      )

   if not detail:
      return gen

   if comm == 0x00:          # SYSTEM
      mess = resp + ' SYSTEM'
      if dlen < 5:
         mess += ' (garbled)'
         return mess
      sub = int(data[4])
      if sub == 0x00:        # STOP
         mess += ' STOP 00/00'
         if data[0:4] == 4*b'\x00':
            mess += ' (everybody)'
         else:
            mess += ' (%s)' % data[0:4].hex()
         return mess
      if sub == 0x01:        # GO
         mess += ' GO 00/01'
         if data[0:4] == 4*b'\x00':
            mess += ' (everybody)'
         else:
            mess += ' (%s)' % data[0:4].hex()
         return mess
      if sub == 0x02:        # SYSTEM HALT
         mess += ' HALT 00/02'
         if data[0:4] == 4*b'\x00':
            mess += ' (everybody)'
         else:
            mess += ' (%s)' % data[0:4].hex()
         return mess
      if sub == 0x03:        # LOCO EMERGENCY STOP
         mess += ' EMERGENCY HALT 00/03 (%s)' % data[0:4].hex()
         return mess
      if sub == 0x04:        # LOCO FORGET
         mess += ' LOCO FORGET 00/04'
         if data[0:4] == 4*b'\x00':
            mess += ' (everybody)'
         else:
            mess += ' (%s)' % data[0:4].hex()
         return mess
      if sub == 0x05:        # LOCO DATA PROTOCOL
         if dlen != 6:
            mess += ' LOCO PROTOCOL CHANGE 00/05 (%s): garbled' % (
               data[0:4].hex()
            )
            return mess
         mess += ' LOCO PROTOCOL CHANGE 00/05 (%s): %02x' % (
            data[0:4].hex(),int(data[5])
         )
         return mess
      if sub == 0x06:        # ACCESSORY SWITCH TIME
         if dlen != 7:
            mess += ' ACCESSORY SWITCHING TIME 00/06 (%s): garbled' % (
               data[0:4].hex()
            )
            return mess
         mess += ' ACCESSORY SWITCH TIME 00/06 (%s): %d ms' % (
            data[0:4].hex(),int.from_bytes(data[5:7])
         )
         return mess
      if sub == 0x08:        # PROTOCOL AVAILABILITY
         if dlen != 6:
            mess += ' PROTOCOL AVAILABILITY 00/08 (%s): garbled' % (
               data[0:4].hex()
            )
            return mess
         mess += ' PROTOCOL AVAILABILITY 00/08 (%s):' % data[0:4].hex()
         if data[5] & 0x01: mess += ' MM2'
         if data[5] & 0x02: mess += ' MFX'
         if data[5] & 0x04: mess += ' DCC'
         return mess
      if sub == 0x09:        # MFX NEW REGISTRATION COUNTER
         if dlen != 7:
            mess += ' MFX NEW REG COUNTER 00/09 (%s): garbled' % (
               data[0:4].hex()
            )
            return mess
         mess += ' MFX NEW REG COUNTER 00/09 (%s): %d' % (
            data[0:4].hex(),int.from_bytes(data[6:8])
         )
         return mess
      if sub == 0x0a:        # OVERLOAD
         if dlen != 6:
            mess += ' OVERLOAD! 00/0A (%s): garbled' % (
               data[0:4].hex()
            )
            return mess
         mess += ' OVERLOAD! 00/0A (%s): channel %d' % (
            data[0:4].hex(), int(data[5])
         )
         return mess
      if sub == 0x0b:        # STATUS
         if dlen == 6 and resp == 'C':
            mess += ' STATUS 00/0B (%s): channel %d' % (
               data[0:4].hex(), int(data[5])
            )
            return mess
         if dlen == 8 and resp == 'C':
            mess += ' STATUS 00/0B (%s): channel %d config %d %d' % (
               data[0:4].hex(),
               int(data[5]), int(data[6]), int(data[7])
            )
            return mess
         if dlen == 8 and resp == 'R':
            mess += ' STATUS 00/0B (%s): channel %d values %d %d' % (
               data[0:4].hex(),
               int(data[5]), int(data[6]), int(data[7])
            )
            return mess
         if dlen == 7 and resp == 'R':
            mess += ' STATUS 00/0B (%s): channel %d is %s' % (
               data[0:4].hex(), int(data[5]), 1 == int(data[6])
            )
            return mess
         if dlen == 7 and resp == 'R':
            mess += ' STATUS 00/0B (%s): channel %d is %s' % (
               data[0:4].hex(), int(data[5]), 1 == int(data[6])
            )
            return mess
         mess += ' STATUS 00/0B (%s): garbled' % (
            data[0:4].hex()
         )
         return mess
      if sub == 0x0c:        # DEVICE ID
         if dlen == 5 and resp == 'C':
            mess += ' DEVICE ID 00/0C (%s)' % data[0:4].hex()
            return mess
         if dlen == 7 and resp == 'C':
            mess += ' DEVICE ID 00/0C (%s): channel %d' % (
               data[0:4].hex(), int.from_bytes(data[5:7])
            )
            return mess
         if dlen == 7 and resp == 'R':
            mess += ' DEVICE ID 00/0C (%s): channel %d' % (
               data[0:4].hex(), int.from_bytes(data[5:7])
            )
            return mess
         mess += ' DEVICE ID 00/0C (%s): garbled' % (
            data[0:4].hex(),
         )
         return mess
      if sub == 0x20:        # TICK (seems to be Rocrail command)
         if dlen == 8:
            mess += ' CLOCK 00/20 (%s): %d:%02d tick %ds' % (
               data[0:4].hex(), int(data[5]), int(data[6]), int(data[7])
            )
         return mess
      if sub == 0x80:        # RESET
         if dlen != 6:
            mess += ' RESET 00/80 (%s): garbled' % data[0:4].hex()
            return mess
         mess += ' RESET 00/80 (%s): target %d' % (
            data[0:4].hex(), int(data[5])
         )
         return mess
      if dlen >= 5:       # Unknown, but decodeable
         mess += ' ***UNKNOWN COMMAND: 00/%02x (%s)' % (
            sub, data[0:4].hex()
         )
         return mess
      return gen
         
   if comm == 0x04:          # LOCO SPEED
      mess = resp + ' LOCO SPEED 04'
      if dlen == 4:
         mess += ' (%s)' % data[0:4].hex()
         return mess
      if dlen == 6:
         mess += ' (%s): %d' % (
            data[0:4].hex(), int.from_bytes(data[4:6])
         )
         return mess
      mess += ' (garbled)'
      return mess
         
   if comm == 0x05:          # LOCO DIRECTION
      mess = resp + ' LOCO DIRECTION 05'
      if dlen == 4:
         mess += ' (%s)' % data[0:4].hex()
         return mess
      if dlen == 5:
         dirn =  int(data[4])
         mess += ' (%s): %d - ' % (
            data[0:4].hex(), dirn
         )
         if dirn == 0: mess += 'maintain'
         elif dirn == 1: mess += 'forward'
         elif dirn == 2: mess += 'reverse'
         elif dirn == 3: mess += 'change'
         else: mess += ' (undefined), maintain'
         return mess
      mess += ' (garbled)'
      return mess

   if comm == 0x06:          # LOCO FUNCTION
      mess = resp + ' LOCO FUNCTION 06'
      if dlen == 5:
         mess += ' (%s): F%d' % (data[0:4].hex(),int(data[4]))
         return mess
      if dlen == 6:
         mess += ' (%s): F%d value %d' % (
            data[0:4].hex(),
            int(data[4]),
            int(data[5])
         )
         return mess
      if dlen == 8:
         mess += ' (%s): F%d value %d range %d' % (
            data[0:4].hex(),
            int(data[4]),
            int(data[5]),
            int.from_bytes(data[6:8])
         )
         return mess
      mess += ' (%s): garbled' % data[0:4].hex()
      return mess

   if comm == 0x07:          # READ CONFIG
      mess = resp + ' READ CONFIG 07'
      loc = int.from_bytes(data[0:4]) & 0xffff
      rng = loc >> 8 & 0xff
      if dlen == 6 or dlen == 7:
         mess += ' (%s): CV%d ' % (
            data[0:4].hex(),
            int.from_bytes(data[4:6]) & 0x3ff
         )
         if rng >= 0x40 and rng <= 0x7f: # MFX check
            mess += ' MFX CV index %d ' % (int(data[4]) >> 2 & 0x3f)
         if dlen == 7:
            if resp == 'C':
               mess += '%d bytes wanted' % (
                  256 if data[6] == b'\x00' else int(data[6])
               )
            else:
               mess += 'value %d 0x%02x' % (int(data[6]),int(data[6]))
            return mess
         mess += 'value UNREADABLE'
      else:
         mess += ' (garbled)'
      return mess

   if comm == 0x08:          # WRITE CONFIG
      mess = resp + ' WRITE CONFIG 08'
      if dlen != 8:
         mess += ' (garbled)'
         return mess
      loc = int.from_bytes(data[0:4]) & 0xffff
      rng = loc >> 8 & 0xff
      mess += ' (%s): CV%d ' % (
         data[0:4].hex(),
         int.from_bytes(data[4:6]) & 0x3ff
      )
      if rng >= 0x40 and rng <= 0x7f: # MFX check
         mess += 'MFX CV index %d ' % (int(data[4]) >> 2 & 0x3f)
      mess += 'value %d 0x%02x' % (int(data[6]), int(data[6]))
      if resp == 'R':
         mess += ', write %s verify %s' % (
            'OK' if data[7] & 0x80 else 'ERROR',
            'OK' if data[7] & 0x40 else 'ERROR'
         )
      return mess

   if comm == 0x0b:          # ACCESSORIES
      mess = resp + ' ACC 0B '
      if dlen >= 6:
         mess += '(%s): posn %d current %d' % (
            data.hex()[0:8], int(data[4]), int(data[5])
         )
      if dlen == 6: return mess
      if dlen == 8:
         mess += ' switch time %dx10 ms' % int.from_bytes(data[6:8])
         return mess
      mess += '(garbled)'
      return mess

   if comm == 0x10:          # S88 POLL
      mess = resp + ' S88 POLL 10 '
      if dlen >= 5:
         mess += '(%s) module %d' % (data[0:4].hex(), int(data[4]))
         if dlen == 7:
            mess += ' %0x04' % int.from_bytes(data[5:7])
         if dlen == 5 or dlen == 7: return mess
      mess += '(garbled)'
      return mess

   if comm == 0x11:          # FEEDBACK
      mess = resp + ' FEEDBACK 11 '
      if dlen < 4:
         mess += '(garbled)'
         return mess

      mess += 'addr %02x%02x %02x%02x ' % (
         int(data[0]),int(data[1]),int(data[2]),int(data[3])
      )
      if dlen == 5:
         mess += 'par %02x' % int(data[4])
         return mess
      if dlen == 8:
         mess += 'old %d new %d speed %d' % (
            int(data[4]),int(data[5]),int.from_bytes(data[6:8])
         )
      else:
         mess += '(garbled)'
      return mess

   if comm == 0x18:          # PING
      mess = resp + ' PING 18 '
      if dlen == 0:
         mess += '(everybody)'
      elif dlen != 8:
         mess += '(garbled)'
      else:
         mess += '(%s): ' % data[0:4].hex()
         if data[6] == 0x00:
            rng = int.from_bytes(data[0:2]) # top halfword of UID
            if data[7] == 0x00:
               if rng & 0xff00 == 0x4200:
                  mess += 'Booster (6017x)'
               else:
                  mess += 'Gleis Fmt Processor'
            elif data[7] == 0x10: 
               mess += 'Gleisbox 60116'
            elif data[7] == 0x11: 
               mess += 'Gleisbox 60117'
            elif data[7] == 0x20: 
               mess += 'Gleisbox 6021 (60128)'
            elif data[7] >= 0x30 and data[7] <= 0x34:
               mess += 'MS2 6065%d' % (3+(int(data[7]) & 0x07))
            elif data[7] == 0x40: 
               if rng & 0xfff0 == 0x5330:
                  mess += 'LinkS88'
               elif rng == 0x4342:
                  mess += 'S88 Gateway'
               else:
                  mess += 'S88 (unknown)'
            elif data[7] == 0x51 and rng == 0x4d43:
               mess += 'MäCAN bus coupler'
            elif data[7] == 0x53:
               if rng == 0x4d43:
                  mess += 'MäCAN Dx32'
               else:
                  mess += 'Cg servo'
            elif data[7] == 0x54:
               mess += 'Cg feedback device'
            else:
               mess += '(unknown)'
         elif data[6:8] == b'\x12\x34': 
            mess += 'MäCAN switch decoder'
         elif data[6:8] == b'\x46\x81' or data[6:8] == b'\x46\xff': 
            mess += 'Rocrail server'
         elif data[6:8] == b'\xee\xee': 
            mess += 'CS2 software'
         elif data[6:8] == b'\xff\xe0': 
            mess += 'Wireless device'
         elif data[6:8] == b'\xff\xf0' or data[6:8] == b'\xff\xff': 
            mess += 'CS2-GUI (Master)'
         else:
            mess += '(unknown)'
         mess += ' ver %d.%d' % (int(data[4]),int(data[5]))
      return mess

   if comm == 0x1b:          # CAN BOOT
      mess = resp + ' CAN BOOT 1B '
      if dlen == 0:
         mess += '(everybody)'
      elif dlen >= 5:
         mess += '(%s): ' % data.hex()[0:2*dlen]
      else:
         mess += '(garbled)'
      return mess

   if comm == 0x1c:          # RAIL BOOT
      mess = resp + ' RAIL BOOT 1C '
      if dlen >= 4:
         mess += '(%s): ' % data.hex()[0:2*dlen]
      else:
         mess += '(garbled)'
      return mess

   if comm == 0x80:          # PROGRAMMING (undocumented)
      mess = resp + ' PROGRAMMING 80 '
      if dlen != 5:
         mess += '(garbled)'
      else:
         mess += '(%s): %d 0x%02x' % (
            data.hex()[0:8], int(data[4]), int(data[4])
         )
      return mess

   return gen
