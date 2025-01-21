# Decodes CAN packet according to Marklin protocol

# Copyright (c) 2025 George Helffrich
# Released under the MIT License (MIT) - see LICENSE file

# See https://github.com/ghfbsd

def decode(ID,data,detail=False) -> str:
   hash = ID & 0xffff
   resp = 'R' if ID >> 16 & 0x01 == 1 else 'C'
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
         mess += ' LOCO PROTOCOL CHANGE 00/05 (%s): %s' % (
            data[0:4].hex(),data[5].hex()
         )
         return mess
      if sub == 0x06:        # ACCESSORY SWITCH TIME
         if dlen != 7:
            mess += ' ACCESSORY SWITCHING TIME 00/06 (%s): garbled' % (
               data[0:4].hex(),
            )
            return mess
         mess += ' ACCESSORY SWITCH TIME 00/06 (%s): %d ms' % (
            data[0:4].hex(),int.from_bytes(data[5:7])
         )
         return mess
      if sub == 0x08:        # PROTOCOL AVAILABILITY
         if dlen != 6:
            mess += ' PROTOCOL AVAILABILITY 00/08 (%s): garbled' % (
               data[0:4].hex(),
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
               data[0:4].hex(),
            )
            return mess
         mess += ' MFX NEW REG COUNTER 00/09 (%s): %d' % (
            data[0:4].hex(),int.from_bytes(data[6:8])
         )
         return mess
      if sub == 0x0a:        # OVERLOAD
         if dlen != 6:
            mess += ' OVERLOAD! 00/0A (%s): garbled' % (
               data[0:4].hex(),
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
         if dlen == 8 and resp == 'C':
            mess += ' STATUS 00/0B (%s): channel %d config %d' % (
               data[0:4].hex(),
               int(data[5]),
               int.from_bytes(data[6:8])
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
            data[0:4].hex(),
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

   if comm == 0x18:          # PING
      mess = resp + ' PING 18 '
      if dlen == 0:
         mess += '(everybody)'
      else:
         mess += '(%s): ' % data.hex()[0:8]
         if data[6:8] == b'\x00\x00':
            mess += 'Gleis Fmt Processor ver %s' % data[4:6].hex()
         elif data[6:8] == b'\x00\x10': 
            mess += 'Gleisbox 601xx ver %s' % data[4:6].hex()
         elif data[6:8] == b'\x00\x20': 
            mess += 'Gleisbox 6021 (60128) ver %s' % data[4:6].hex()
         elif data[6:8] == b'\x00\x30': 
            mess += 'MS2 60653 etc. ver %s' % data[4:6].hex()
         elif data[6:8] == b'\x46\xff': 
            mess += 'Rocrail server ver %s' % data[4:6].hex()
         elif data[6:8] == b'\xff\xe0': 
            mess += 'Wireless device ver %s' % data[4:6].hex()
         elif data[6:8] == b'\xff\xff': 
            mess += 'CS2-GUI (Master) ver %s' % data[4:6].hex()
         else:
            mess += '(unknown) ver %s' % data[4:6].hex()
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

   return gen
