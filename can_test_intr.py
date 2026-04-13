# Raspberry Pi pico CAN bus test with Joy-IT expansion board,
#    with or without interrupts.
# Also should work with Waveshare Pico-CAN-B board (but as yet untested)

# Import necessary libraries
import sys, time
from canbus import Can, CanError, CanMsg, CanMsgFlag
from machine import Pin, SPI
from collections import namedtuple

POLL = False               # True for polling, False for interrupt

_CANBOARD = const('auto')  # Board choice: 'auto' for auto-detect or
                           # 'JI' or 'WS'

CAN_pins = namedtuple('CAN pins',
   ['INT_PIN','SPI_CS', 'SPI_SCK', 'SPI_MOSI', 'SPI_MISO', 'name']
)
pins_JI = CAN_pins(
   # These pin assignments are appropriate for a RB-P-CAN-485 Joy-IT board
   name = 'Joy-IT',
   INT_PIN = 22,                 # Interrupt pin for CAN board
   SPI_CS = 17,
   SPI_SCK = 18,
   SPI_MOSI = 19,
   SPI_MISO = 16
)
pins_WS = CAN_pins(
   # These pin assignments are appropriate for a Waveshare Pico-CAN-B board
   name = 'Waveshare',
   INT_PIN = 21,                 # Interrupt pin for CAN board
   SPI_CS = 5,
   SPI_SCK = 6,
   SPI_MOSI = 7,
   SPI_MISO = 4
)
boards = dict(
   WS = pins_WS,
   JI = pins_JI
)

if _CANBOARD == 'auto':
   for bd in boards:
      # Check if the initialization is successful
      pin = boards[bd]
      prep = SPI(0,              # configure SPI to use this board's pins
          sck=Pin(pin.SPI_SCK), mosi=Pin(pin.SPI_MOSI), miso=Pin(pin.SPI_MISO)
      )
      # Create an instance of the Can class to interface with the CAN bus
      can = Can(spics=pin.SPI_CS)
      if can.begin() == CanError.ERROR_OK:
         board = pin.name
         break
   else:
      raise RuntimeError(
         "***Can't auto-detect board; set explicit board name***"
      )
elif _CANBOARD in boards:        # Explicit board choice?
   pin = boards[_CANBOARD]
   prep = SPI(0,                 # configure SPI to use this board's pins
       sck=Pin(pin.SPI_SCK), mosi=Pin(pin.SPI_MOSI), miso=Pin(pin.SPI_MISO)
   )
   # Create an instance of the Can class to interface with the CAN bus
   can = Can(spics=pin.SPI_CS)
   if can.begin() != CanError.ERROR_OK:
      raise RuntimeError("Error initializing %s CAN board - check type!" %
         pin.name
      )
   board = pin.name
else:
   raise RuntimeError('***%s is an unsupported CAN board***' % _CANBOARD)

pico_led = Pin("LED")
pico_led.off()

print("Initialized %s board successfully, %s mode." %
    (board, 'polling' if POLL else 'interrupt')
)

ret = can.setLoopback()
if ret != CanError.ERROR_OK: # Check if the initialization was successful
    raise RuntimeError("Error setting CAN loopback!")

def recv(can):
    # Receive data from the CAN bus; returns an error code and the message
    error, msg = can.recv()
    # Check if data was received without errors
    if error == CanError.ERROR_OK:
        # Print received message details
        # Print the CAN ID in hexadecimal format and whether RTR format
        data = ' '.join(           # put space between every octet
            map(''.join, zip(*[iter(msg.data.hex())]*2))
        )
        print("    CAN id: %#x%s%s (%d bytes): %s" % (
            msg.can_id,
            ' (RTR)' if msg.is_remote_frame else '',
            ' (EFF)' if msg.is_extended_id else '',
            msg.dlc,
            data if msg.dlc > 0 else ''
        ))
    return error

# Data to be sent over the CAN bus in bytes format
data = b"\x12\x34\x56\x78\x9A\xBC\xDE\xF0"
n = 0

# Main loop to send data to the CAN bus
while POLL:
    n += 1
    print('---------------------------------')

    # Create a standard format frame CAN message
    # can_id is the identifier for the CAN message, data is the bytes to send
    msg = CanMsg(can_id=0x123, data=data)
    error = can.send(msg) # Send the CAN message and store the error status
    if error == CanError.ERROR_OK: # Check if the message was sent successfully
        print('{:3d} send normal------------------'.format(n))
    if can.checkReceive():
        if recv(can) != CanError.ERROR_OK:
            print('{:3d} -----------------receive FAIL'.format(n))

    # Create an extended format frame CAN message
    # EFF flag indicates an extended frame format
    msg = CanMsg(can_id=0x12345678, data=data, flags=CanMsgFlag.EFF)
    error = can.send(msg) # Send the CAN message and store the error status
    if error == CanError.ERROR_OK: # Check if the message was sent successfully
        print('{:3d} send EFF---------------------'.format(n))
    if can.checkReceive():
        if recv(can) != CanError.ERROR_OK:
            print('{:3d} -----------------receive FAIL'.format(n))

    pico_led.toggle()
    time.sleep(1) # Wait for 1 second before sending the next set of messages


# Falls through to here if interrupt driven

def trigger(pin):
    global can, POLL
    stat = can.getStatus()       # Order seems to matter here: status first ...
    intr = can.getInterrupts()   # ...then interrupt reg
    if POLL:
        print("Interrupt in polling mode?!  intr %02x stat %02x" % (intr,stat))
        can.clearInterrupts()
        return
    if intr & 0x03 == 0:
        # ignore interrupts except for reading
        print("    >>> Non-read interrupt:  intr %02x stat %02x" % (intr,stat))
        can.clearInterrupts()
        return
    while can.checkReceive():
        if recv(can) != CanError.ERROR_OK:
            print('{:3d} -----------------receive FAIL'.format(n))
    can.clearInterrupts()

print("Interrupt mask is %02x" % can.getInterruptMask())

pin = Pin(pin.INT_PIN,Pin.IN,Pin.PULL_UP)
pin.irq(trigger=Pin.IRQ_FALLING,handler=trigger)

try:
    while not POLL:
        n += 1
        print('---------------------------------')

        # Create a standard format frame CAN message
        # can_id - identifier for the CAN message, data - bytes to send
        print('{:3d} send normal------------------'.format(n))
        msg = CanMsg(can_id=0x123, data=data[:n%9])
        error = can.send(msg) # Send the CAN message and store the error status
        if error != CanError.ERROR_OK:
            # Check if the message was sent successfully
            print('{:3d} -------------------------FAIL'.format(n))

        # Create an extended format frame CAN message
        # EFF flag indicates an extended frame format
        print('{:3d} send EFF---------------------'.format(n))
        msg = CanMsg(can_id=0x12345678, data=data[:n%9], flags=CanMsgFlag.EFF)
        error = can.send(msg) # Send the CAN message and store the error status
        if error != CanError.ERROR_OK:
            # Check if the message was sent successfully
            print('{:3d} -------------------------FAIL'.format(n))

        pico_led.toggle()
        time.sleep(1)

except KeyboardInterrupt:
    pass

pin.irq(handler=None)
