# Implement feedbacks from GPIO pins on microcontrollers that run micropython 

import uasyncio as asyncio
from machine import Pin, Timer

class Feedback:
   # Implement feedback by logic-level input from track sensors

   interrupt = False                  # True for interrupt or False for poll
   CS2_SIZE = const(13)               # Fixed by protocol definition
   SETTLE_TIME = const(65)            # "S88" contact settle time (ms) (was 125)

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
         if Feedback.interrupt: self.fbpin[ch].irq(
            # this defines the interrupt handler
               trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
               handler=lambda p, id=ch: self._intr(id),
               hard=True
            )

         val = not self.fbpin[ch].value()  # Get present pin state to initialize
         self.chn[ch], self.state[ch] = val, val

         self.pool[ch] = bytearray(Feedback.CS2_SIZE)
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
            self._time() if Feedback.interrupt else self._poll()
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
                  period=Feedback.SETTLE_TIME,
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
            pps |= self.state[n] << n
         self.fbpp[11] = pps     # Maintain state in poll packet
         await asyncio.sleep_ms(Feedback.SETTLE_TIME)

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

   def stop(self,print=True):
      self.ticker.deinit()
      if Feedback.interrupt:
         for i in range(self.n): self.fbpin[i].irq(handler=None)
      try:
         self.task.cancel()
      except:
         pass
      if print:
         print(
            'Feedback interrupts: %d, %.2f/sec' % (self._n, self._n/self._secs)
         )
      
   @property
   def state_packet(self):       # provide state packet
      return self.fbpp

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
