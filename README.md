# Raspberry Pi Pico-based TCP-CAN or UDP-CAN hub

![Gleisbox and board image](https://github.com/ghfbsd/pico_rocrail_can_tcp_gateway/blob/main/images/complete.jpg?raw=true)

## Introduction

This is a low-cost solution to the goal of running a Märklin train set by a
remote computer/handheld device without needing a CS2/CS3 controller.  This
device works with the 60116 Gleisbox[^1] typically provided with starter train
sets (along with the 60657 MS2 controller).

[^1]: Also works with the 60112, 60113, or 60114 Gleisbox.

The hardware and software implement a wireless hub that does two things:
* takes incoming train control packets over WiFi and sends them to the trains
and accessories on the rails
* takes the signals from the trains and the accessories on the rails and sends
them out over the WiFi network

The communications protocol used is a public one
[published by Märklin](https://streaming.maerklin.de/public-media/cs2/cs2CAN-Protokoll-2_0.pdf) using either TCP as or UDP a transport layer on WiFi and the CAN
protocol over wires connecting the various Märklin boxes.
This same protocol is used by popular train control software, in particular
[Rocrail](https://wiki.rocrail.net/doku.php?id=start),
[iTrain](https://www.berros.eu/en/itrain/),
[TrainController](https://www.freiwald.com/pages/traincontroller.htm),
[JMRI](https://www.jmri.org) and possibly also with
[BTrain](https://github.com/jean-bovet/BTrain), among others.

## Requirements

There are only two pieces of hardware required along with cabling:
* a Raspberry Pi Pico Wireless (with headers);
* a Joy-IT CAN-RS485 board for the Raspberry Pi Pico (RB-P-CAN-485) or
Waveshare Pico-CAN-B board;
* two male-male breadboard jumper wires (only needed for the Joy-IT board),
plus another three for connecting the CAN board to the Gleisbox;
* USB to mini-USB cable.

Total cost of these items is about 20 euros (Jan. 2025).
There is no soldering required, although a minimal amount (two connections on
the Joy-IT board)
yields a more robust setup.

## Setup

### Development environment

This description uses [rshell](https://github.com/dhylands/rshell) for a
development environment.  Others may prefer [Thonny](https://thonny.org).

Get your Raspberry Pi Pico (RPP) working with **rshell** by downloading the
[MicroPython](https://docs.micropython.org/en/latest/) firmware onto it.
The appropriate firmware for a RPP can be found [here](https://micropython.org/download/?port=rp2); make sure to choose the wireless version.

### Assemble the board

Familiarize yourself with the
[RB-P-CAN-485 board](https://github.com/ghfbsd/RB-P-CAN-485) if you're using it;
the board requires some setting up before it will work with your RPP.

* If you want to solder jumpers to the CS and INT connections on the
RB-P-CAN-485, do it now. 
* Plug the RPP into the RB-P-CAN-485 board.
* Jumper CS to pin GP17, and INT to pin GP20[^2].
* Plug the USB cable into the RPP and connect it to your computer.

[^2]: The photo shows different jumper pins for a previous code release; be assured the text is correct.

The Waveshare board does not need any preparation before use.

* Plug the RPP into the Pico-CAN-B board.
* Plug the USB cable into the RPP and connect it to your computer.


### Download the software

The program needs a few bits and bobs to run with MicroPython.
The most important one is the CAN bus driver.  You can get it
[here](https://github.com/ghfbsd/MicroPython_CAN_BUS_MCP2515).  It extends the
one provided by the manufacturer to enable more advanced features of the board 
that the CAN interface uses (interrupts, error classification,
loopback testing).

Assuming that you are running **rshell** from the directory where you downloaded
this repository's contents, and that the CAN bus driver is in a subdirectory
of it called `MicroPython_CAN_BUS_MCP2515`, proceed as follows.
In **rshell**, after connecting to the RPP:

```
cd MicroPython_CAN_BUS_MCP2515
rsync canbus /pyboard/canbus
cd ..
```
after you do this, you should have the Python CAN bus driver code in the
directory `canbus` on the RPP.  (You can check by `ls /pyboard` from **rshell**;
you should only see the directory `canbus`.)

#### Test the CAN board

Start **rshell** and connect to the RPP to test the CAN interface board.
Try the `can_test_intr.py` in the
[RB-P-CAN-485 repository](https://github.com/ghfbsd/RB-P-CAN-485)
to verify that it is working properly.
If using the Waveshare board, make sure you configure
`can_test_intr.py` properly to use the different pin assignments.

### Download more software

Next, from this repository, download two utilities used by the program:
queuing routines[^3] and a Märklin packet decoder[^4].
In **rshell**, again:

[^3]: The [asynchronous queue management](https://github.com/peterhinch/micropython-async/tree/master) will eventually be incorporated into MicroPython
itself; not yet, though.

[^4]: The Märklin packet decoder is for debug output.
It is optional, but you might be curious to see the details of the traffic between your train controller and the Märklin Gleisbox.

```
cp marklin.py /pyboard
cp threadsafe.py /pyboard
```

At this point, you need to choose whether you want to connect over WiFi with
TCP or UDP.  This depends on what controller software you plan to use.
Here are some guidelines:

* Use UDP with JMRI - as of March 2025, it does not support TCP
* Rocrail works with either TCP or UDP, but Märklin's preference is for TCP
* If you plan to use a single controller, use TCP
* Use UDP if you want to have multiple controllers running the same layout.

The TCP hub is `TCP-CAN.py` and the UDP hub is `UDP-CAN.py`.  With either one,
you have to edit the program's text to add your WiFi network credentials.
With your favorite editor, change the lines,
```
SSID = "****"
PASS = "****"
```
to the appropriate network name and password for your WiFi environment.
The default configuration assumes you're using the Joy-IT board.
If you have a Waveshare board, edit the code to use the right pin assignments
for it.  Then, save the file.

Finally, using **rshell**, load the program and make it run
automatically when the RPP starts up.
```
cp TCP-CAN.py /pyboard/main.py
```
or
```
cp UDP-CAN.py /pyboard/main.py
```

### Connect to the Gleisbox

The connection to the Märklin Gleisbox is by way of pins on the mini-DIN 10
pin connection socket on the side of the box.  See the diagram below:

![mini-DIN 10 connector image](https://github.com/ghfbsd/pico_rocrail_can_tcp_gateway/blob/main/images/mini-DIN10.jpg?raw=true)

You need to connect the CAN low, CAN high and ground ("Masse") to your
CAN board.
Male-male breadboard jumpers are ideal for this: screw one set of pins to the
board, and insert the other pins into the proper holes in the connector socket
on the Gleisbox.

**Be very careful to avoid the "Versorgung +" pin.  This is the 18V power
supply output from the Gleisbox.  If 18V is connected to your CAN bus, it will
be damaged (the bus operates on 3.3V).**

### Test the board

Start the program by going into REPL mode and then running the Python code:
```
xxx> repl pyboard
>>> execfile('main.py')
```
(`xxx>` is the **rshell** prompt; `xxx` is your local directory name where
you're running **rshell**, and `>>>` is the MicroPython REPL prompt.)

You should see something similar to

```
TCP <-> CAN packet hub (AN245)
Available at 10.0.1.28 as CS2hub-747a13, port 15731.
CAN initialized successfully, waiting for traffic.
TCP connection made, waiting for traffic.
...
```
or

```
UDP <-> CAN packet hub (AR095)
Available at 10.0.1.7 as CS2hub-747a13.
CAN initialized successfully, waiting for traffic.
UDP listening, waiting for traffic.
...
```

There are a few useful things to note in the startup messages.
First, they give the IP address and device name that you will need to set
up your train controller to communicate with the wireless hub.  Second, it
gives you the port number the TCP traffic is expected on.  You'll also need
this to configure your train controller.  (Port 15731 is the port customarily
used by Märklin to communicate by TCP with a Gleisbox.)

Another useful thing is the host name of the hub, `CS2hub-xxxxxx`.
This is a permanent, unique host name assigned to the RPP; while your IP
number might change, depending on your local network setup, the host name
will not.  If you have DNS (or mDNS) features enabled on your network,
you can get the IP address from that to configure your train controller.
If your train controller accepts host names, then you can provide the name and
you don't need to know the IP address -- or care whether it changes in future --
at all.

To check whether you have DNS discovery of the RPP on your network, try
```
ping -p 15731 CS2hub-xxxxxx
```
and see if you get a reply.  If you do, great - otherwise use the IP address
to configure your train controller ... and pay attention to the initial
connection dialog.

## Using the Hub with Rocrail

Rocrail has built-in support for 
Märklin's TCP and UDP protocol: the MBUS protocol.
In Rocrail's `Rocrail properties ...` dialog, navigate to the `Controller`
panel.
You want to add a new controller to the list.
Find the `New` button, select the `mbus` type, and `Add` it.
You'll see a dialog like this:

![dialog box image](https://github.com/ghfbsd/pico_rocrail_can_tcp_gateway/blob/main/images/Add-dialog.jpg?raw=true)

Then:
* Change `NEW` to some name that you prefer for the hub, e.g. RPIW-CAN.
* Select `TCP` as the type if using TCP or `UDP` if using UDP.
* Fill in `Hostname` with the IP number (or `CS2hub-xxxxxx` if you have DNS).
* Leave the `:` field following `Hostname` blank, which Rocrail assumes to
mean 15731 for TCP and 15731/15730 for UDP.
* Click `OK` to add.

It should look like this:

![dialog box image](https://github.com/ghfbsd/pico_rocrail_can_tcp_gateway/blob/main/images/mbus-1.jpg?raw=true)

If you want to run **without** the MS2 controller attached to the Gleisbox,
go to the `Options` panel, and make sure `Master` is selected, as shown below:

![dialog box image](https://github.com/ghfbsd/pico_rocrail_can_tcp_gateway/blob/main/images/mbus-2.jpg?raw=true)

Otherwise, leave it unselected.

The loco you want to drive has to be connected to this controller (say, RPIW-CAN
like we used above).  Navigate to the `Tables` -> `Locomotives` dialog box,
and then click on the `Interface` tab.  In the `Interface ID` box type
`RPIW-CAN`.  Select the protocol appropriate for your loco, then click `OK`.
At this point, you can start running the loco.  Repeat as needed for further
locos.

Power up the system by plugging in the Märklin power supply to the Gleisbox[^5].
At this point, you should start to see packets flowing from Rocrail and from
the Gleisbox.  They will resemble this:

```
TCP -> CAN 0036 4767 00 0000 0000 0000 0000
   C CAN BOOT 1B (everybody)
TCP -> CAN 0030 4767 00 0000 0000 0000 0000
   C PING 18 (everybody)
CAN -> TCP 0031 4f20 08 4746 e70b 013e 0010
   R PING 18 (4746e70b): Gleisbox 601xx ver 013e
```

[^5]: You do not need the MS2 (or CS2/CS3) to operate the system
(but you may wish to connect it anyway -- it will act as a slave/repeater to
your computer controller software).
If you have the MS2 connected, after initializing it will be in the STOP state
(red STOP lights on); press STOP to put it into operation mode
(red STOP lights off).



Anything prefixed with `C` is a command, and with `R` is a response.
The command is telling the system to initialize itself.  Then there is a
ping to locate the Gleisbox. The response is from the Gleisbox telling you
that it is there and listening for commands.
The Gleisbox sends `PING`s every 6 s or so - you'll see a lot of these.

Start running your train with either the MS2 or Rocrail.  You will see
packets flowing back and forth whenever the train's state changes, or when
accessories are commanded to change.

Once you're confident your board is working, you don't need to have the
USB connected to your computer.  Rather, you can connect it to a USB power
source to run the board and the RPP.  It does not matter which USB connection
you use: either of the connectors on the RPP or the CAN board will
run them both.  Only the RPP's USB connection will talk to **rshell** though.

## Additional features

The hub also provides a small number of feedback pathways to the train
controller software like Rocrail.  It is equivalent to a Märklin S88
connected to a CS2/CS3 controller (but with only 8 feedback paths).
Each feedback path is activated by shorting one of the RPP pins to ground.
The simulated S88 node ID is 1, and contacts 0-7 are available:

| contact | Joy-IT board    | Waveshare board |
| ------- | --------------- | --------------- |
|    0    | GPIO 0  = pin 1 | GPIO 0 = pin 1  |
|    1    | GPIO 1  = pin 2 | GPIO 1 = pin 2  |
|    2    | GPIO 8  = pin 11| GPIO 2 = pin 4  |
|    3    | GPIO 9  = pin 12| GPIO 3 = pin 5  |
|    4    | GPIO 10 = pin 14| GPIO 10 = pin 14|
|    5    | GPIO 11 = pin 15| GPIO 11 = pin 15|
|    6    | GPIO 14 = pin 19| GPIO 12 = pin 16|
|    7    | GPIO 15 = pin 20| GPIO 13 = pin 17|

Connect your feedback devices using breadboard jumper wires: one to one of
the RPP ground pins (pin 3, 8, 13 or 18) and either a male or female
jumper wire to the appropriate pin for the contact channel.

## Operational notes

There are two board monitor functions provided via the LED on the RPP.
* When connecting to WiFi, there is a fast LED flash.  Once the connection
is made, you will see a slower flash.
To retry the WiFi connection, press the `BOOT SEL` button on the RPP.
(If you're unable to connect, check that you entered your WiFi credentials
correctly in the program before downloading it to the RPP.)
* While running the hub, there is a slow heartbeat on the RPP to show you
that the program is running.  If the flashing stops (no light or continuous
light), something is hung.  To restart, press the `BOOT SEL` button on the RPP.

If you power off the Gleisbox, there is no longer an active CAN bus.
The program handles this, but may not be able to successfully sync with the
CAN bus after the Gleisbox powers up again.  If this turns out to be a problem,
press the `BOOT SEL` button on the RPP to restart.  Or, cycle the power by
unplugging it and plugging it back in.

## Utilities

If you want to monitor the CAN traffic between the Gleisbox and your controller
(an MS1, MS2, a CS1, CS2, or a CS3), download the `CS2-sniff.py` program to
your RPP and run it while attached via the USB cable.  It will print out a log
of all of the packets exchanged over the CAN bus.  In *rshell*,

```
xxx> cp CS2-sniff.py /pyboard
xxx> repl pyboard
>>> execfile('CS2-sniff.py')
```
will start up the CAN packet sniffer.  It will run until it is
interrupted with a control-C.

## Bugs and Wish List

* It would be great to have the hub advertise the train control service so that
Rocrail could discover it.  This involves using mDNS to advertise the
CANservice to Rocrail (services of Rocrail's interest are shown
[here](https://wiki.rocrail.net/doku.php?id=mdns-en))
so that Rocrail can discover the hub automatically.
A restriction in MicroPython's support of the RPP (as of Jan. 2025) prevents
this, however, because use of port 5353 (mDNS) is blocked by MicroPython's own
use of it.

## Acknowledgements

* Christophe Bobille for the inspiration (see [this post](https://www.locoduino.org/spip.php?article361))
* @xxup, particularly, and others on the marklin-users.net community forum for
feedback
