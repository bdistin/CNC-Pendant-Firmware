# CNC-Pendant-Firmware

This is firmware to run on an Arduino Pro Micro (preferred) or Arduino Nano to interface a popular style of wired CNC pendant to the PanelDue port of Duet electronics. Build it using Arduino IDE. 

For a full guide to building the pendant, see [the Duet3D wiki here](https://docs.duet3d.com/en/User_manual/Connecting_hardware/IO_CNC_Pendant).

## Fork Notes

This is a fork meant to adapt a D15 ended MPG. This has minor modifications due to the specific wiring of an off the shelf MPG. Please verify your specific MPG wiring
against the below diagram. 
```
     ________________________________________
    / GND  A-   B-   YIN  4    X10   EP      \
   / 5v   A+   B+   XIN  ZIN  X100  5    COM  \
   --------------------------------------------
     ______________________
    / 9  10 11 12 13 14 15 \
   / 1  2  3  4  5  6  7  8 \
   --------------------------
```
Because of the available pins this is slightly less capable than the Head of this fork. There is no way to control the LED, nor a way to differenciate between No Axis
and the 6th Axis. The X1 multiplier setting is possible with the adjusted logic in this fork by only checking the X10/X100 pins.

## Pendant to Arduino Pro Micro connections

| Function   | D15 Pin    | Pro Micro |
|:-----------|:-----------|:----------|
| 5v         | 1          | VCC       |
| A+         | 2          | D10       |
| B+         | 3          | D14       |
| XIN        | 4          | D2        |
| ZIN        | 5          | D4        |
| X100       | 6          | D6        |
| Axis5      | 7          | D9        |
| GND/COM    | 8, 9, 16   | GND       |
| YIN        | 12         | D3        |
| Axis4      | 13         | D5        |
| X10        | 14         | D7        |
| EStop      | 15         | D8        |

Arduino Pro Micro to Duet 3 IO_0 connector or Duet 2 PanelDue connector wiring (3- or 4-core cable):

| Pro Micro | Duet |
|:----------|:-----|
| VCC       | +5V (red wire) |
| GND       | GND (yellow wire) |
| TXO<br>GND | Through 6K8 resistor to IO_0_IN (Duet 3) or URXD0 (Duet 2)<br>Also connect 10K resistor between GND and IO_0_IN (Duet 3) or URXD0 (Duet 2) (blue wire from resistor junction to Duet) |

To connect a PanelDue as well (the Arduino Pro Micro passes the PanelDue commands through to the Duet):

| PanelDue | Pro Micro / Duet |
|:---------|:-----------------|
| +5V      | +5V/VCC (red wire to Ardiuno or Duet) |
| GND      | GND (yellow wire to Ardiuno or Duet) |
| DIN      | **Duet** IO_0_OUT (Duet 3) or UTXD0 (Duet 2) (green wire) |
| DOUT     | **Pro Micro** RXI (blue wire of PanelDue cable to green wire of pendant cable) |

For wiring differences and hardware changes needed if using an Arduino Nano, see the comments at the start of the CNC-pendant.ino file.

## Support requests

Please use the [forum](https://forum.duet3d.com) for support requests.
