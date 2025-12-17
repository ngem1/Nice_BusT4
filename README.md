# ESPHOME component for controlling Nice drives via the Bus T4 protocol
# Nice Bus T4 protocol

I wanted to understand the protocol for controlling Nice gates.
Perespektiva - low-cost devices based on the esp8266 for smart home control.

Modern drive control units have a Bus T4 connector, which contains GND, +V, Can-Rx, and Can-Tx. The V voltage can vary from 24 to 35 volts depending on the control unit.

# Current component capabilities
* Sending commands: "Open", "Stop", "Close", "Partial Open", "Step-by-Step (SBS)", and others via buttons.
* Sending arbitrary HEX commands via the "raw_command" service. The command must be pre-formatted or found somewhere. Byte separators can be periods or spaces. Example: 55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c or 55.0D.00.FF.00.66.08.06.97.00.04.99.00.00.9D.0D
* Generate and send arbitrary GET/SET requests via the "send_inf_command" service. Allows you to configure the device or retrieve its status.
* Display packets from all devices on the busT4 network in the log.

# BusT4:

This is a modified UART 19200 8n1 with a uart.break duration of 519us-590us before each packet.
Multiple devices can be connected; CAN-BUS transceivers have been added to the physical layer for this purpose. Physical transmission most often occurs via CAN transceivers, but there are no CAN frames.

# What's been done:
* Connected FTDI232 to GND, Can-Rx, and Can-Tx. Packets are visible and decipherable.
* Using a logic analyzer, I saw the signal shape and packet composition and adjusted the UART parameters.
* Successfully simulated a read packet via an Arduino Mega; the drive responds.
* Received OPEN CLOSE and other commands.
* Received drive status byte.
* Read the main commands and partially decoded the byte values.
* Assembled a prototype device and tested it. * Built a sniffer for capturing packets between OVIEW and busT4 devices
* Wrote a component that can control drives and receivers via the BusT4 protocol
* Tested the operation on a Wingo5000 with the [MCA5](img/IMG_20220113_160221.jpg), [Robus RB500HS](img/3hs.jpg), Soona SO2000, [Rd400](img/rba4.jpg), [D-PRO924](img/924.jpg), [Walky WL1024C](img/walky.jpg), [SPIN 22BDKCE](img/spin.jpg) unit.

![alt text](img/Schematic_esphome_bust4_adapter.png "Bus-T4 Adapter Schematic")

The ESP8266 signal level doesn't match the T4 BUS. Add a 3.3V to 5V level converter for the Tx transistor.
The ESP's Rx is 5V-tolerant, but a diode is needed for stable operation. It works for me with a random germanium diode, but a silicon diode might work as well.

The circuit was subsequently modified. ![alt text](img/Schematic_busT4adapter_xl.png "Bus-T4 adapter schematic with modified power supply")
![alt text](img/IMG20230306201230.png "Complete 2.0 device")

![alt text](img/hassio-bust4.png "Bus-T4 component operation test")

Walky drives require a schematic with a CAN transceiver. The Bus-T4 connector is hidden under a cover. This same diagram is also suitable for other control units with an RJ-11 (6p4c) connector, such as [mc824h](img/mc824h.jpg) or [RBA3/C](img/rba3c.jpg)
![alt text](img/Schematic_bust4_2023-10-18.png "CAN + bus-t4 diagram")

The component supports sending a custom command to the drive via the ESPHome service: nice_bust4_uart_raw_command in Home assistant. ```
SBS: 55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c
Open: 55 0c 00 03 05 81 01 05 83 01 82 03 64 e4 0c
Close: 55 0c 00 03 05 81 01 05 83 01 82 04 64 e3 0c
Stop: 55 0c 00 03 00 81 01 05 86 01 82 02 64 e5 0c
``

During startup and operation, the ESP polls devices connected to the BusT4 bus and outputs information about them to the log.
![log](img/log.png "Log")
![log](img/log2.png "Log2")

# Updates
* Services have been added to the component interface to more easily launch the sash length recognition procedure and the BlueBus device recognition procedure without disassembling the drive housing (or even remotely).
* Added configuration log output of L1, L2, and L3 states read from the device (Automatic
Close, Close After
Photocell, Always Close)
* Improved compatibility with DPRO924 drives
* The STOP button is always available in the object's User Interface
* Improved compatibility with Walky WL1024C drives
* Improved compatibility with Spin drives ([@TheGoblinHero](https://github.com/TheGoblinHero))
* Added the ability to set a custom drive position ([@TheGoblinHero](https://github.com/TheGoblinHero))

If you're interested in the project, you can [buy me a beer or coffee](https://www.tinkoff.ru/cf/12xvN3UtJkO)
