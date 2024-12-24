**Fork to support both ESP8266 and ESP32 with ability to define custom pins for UART TX/RX**

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/makstech/Nice_BusT4
    refresh: 1min

cover:
  - platform: bus_t4
    name: "Nice Cover"
    device_class: gate
    id: nice_cover
    # Optional variables to set custom UART TX/RX pins
    uart_num: "UART_NUM_2"
    tx_pin: 17
    rx_pin: 16
```

[English](https://github.com/xdanik/Nice_BusT4)

# ESPHome Component for Controlling Nice Actuators via the Bus T4 Protocol

I became interested in understanding the protocol for controlling Nice gates.  
The goal: inexpensive devices based on the ESP8266 for integration with a smart home.

Modern actuator control units feature a Bus T4 connector with GND, +V, Can-Rx, and Can-Tx pins. The voltage (V) can vary between 24 and 35 volts depending on the control unit.

## Current Capabilities of the Component

- Sending commands such as "Open," "Stop," "Close," "Partial Open," "Step-by-Step (SBS)," and others via buttons.
- Sending arbitrary HEX commands through the `raw_command` service. Commands need to be prepared in advance or found elsewhere. Byte delimiters can be dots or spaces.  
  Example:  
  `55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c` or `55.0D.00.FF.00.66.08.06.97.00.04.99.00.00.9D.0D`.
- Generating and sending custom GET/SET requests through the `send_inf_command` service, allowing you to configure the device or retrieve its status.
- Logging packets from all devices on the Bus T4 network.

## Bus T4 Protocol

This is a modified UART (19200 8n1) with a `uart.break` duration of 519–590 µs before each packet.  
Multiple devices can be connected, as CAN-BUS transceivers are added at the physical level.  
Transmission often occurs through CAN transceivers, but no CAN frames are present.

## Accomplishments

- Connected an FTDI232 to GND, Can-Rx, and Can-Tx. Packets are visible and can be decoded.
- Observed signal shape and packet composition using a logic analyzer, adjusted UART parameters.
- Successfully emulated a captured packet using Arduino Mega; the actuator responds.
- Captured commands for OPEN, CLOSE, etc.
- Retrieved the actuator status byte.
- Decoded basic commands, partially interpreted byte values.
- Built a prototype device and tested functionality.
- Built a sniffer to capture packets between OVIEW and Bus T4 devices.
- Created a component capable of controlling actuators and receivers via the Bus T4 protocol.
- Tested functionality with Wingo5000 using the [MCA5](img/IMG_20220113_160221.jpg) unit, [Robus RB500HS](img/3hs.jpg), Soona SO2000, [Rd400](img/rba4.jpg), [D-PRO924](img/924.jpg), [Walky WL1024C](img/walky.jpg), and [SPIN 22BDKCE](img/spin.jpg).

![Schematic of the Bus T4 adapter](img/Schematic_esphome_bust4_adapter.png)

The ESP8266 signal level does not match Bus T4, so a 3.3V → 5V level shifter for Tx using a transistor is needed.  
The ESP's Rx pin tolerates 5V, but a diode is required for stability. Mine works with a random germanium diode; silicon might also work.

Later, the schematic was modified.  
![Schematic of the modified Bus T4 adapter](img/Schematic_busT4adapter_xl.png)  
![Completed device 2.0](img/IMG20230306201230.png)

![Component test for Bus T4](img/hassio-bust4.png)

For Walky actuators, a CAN transceiver is required. The Bus T4 port is hidden under a cover. This same schematic is suitable for other units with an exposed RJ-11 (6P4C) port, e.g., [mc824h](img/mc824h.jpg) or [RBA3/C](img/rba3c.jpg).  
![Schematic with CAN + Bus T4](img/Schematic_bust4_2023-10-18.png)

The component supports sending arbitrary commands to the actuator via the ESPHome service: `nice_bust4_uart_raw_command` in Home Assistant.  
SBS: 55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c
Open: 55 0c 00 03 05 81 01 05 83 01 82 03 64 e4 0c
Close: 55 0c 00 03 05 81 01 05 83 01 82 04 64 e3 0c
Stop: 55 0c 00 03 00 81 01 05 86 01 82 02 64 e5 0c

When the ESP starts and runs, it queries devices connected to the Bus T4 network and outputs information about them in the log.  
![Log example](img/log.png)  
![Log example 2](img/log2.png)

## Updates

- Added services to the component interface for easier execution of leaf length recognition and BlueBus device recognition procedures without opening the actuator housing (even remotely).
- Added log output for configuration states (L1, L2, L3: Automatic Closing, Close after Photocell, Always Close).
- Improved compatibility with DPRO924 actuators.
- STOP button is always available in the User Interface object.
- Enhanced compatibility with Walky WL1024C actuators.
- Improved compatibility with Spin actuators ([@TheGoblinHero](https://github.com/TheGoblinHero)).
- Added functionality to set arbitrary actuator positions ([@TheGoblinHero](https://github.com/TheGoblinHero)).

If you find this project interesting, you can [buy me a beer or coffee](https://www.tinkoff.ru/cf/12xvN3UtJkO).
