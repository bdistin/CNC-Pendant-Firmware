// CNC pendant interface to Duet
// D Crocker, started 2020-05-04

/* This Arduino sketch can be run on either Arduino Nano or Arduino Pro Micro. 
 * It should alo work on an Arduino Uno (using the same wiring scheme as for the Nano) or Arduino Leonardo (using the same wiring scheme as for the Pro Micro).
 * The recommended board is the Arduino Pro Micro because the passthrough works without any modificatoins to the Arduino. 

/* This is a fork meant to adapt a D15 ended MPG. This has minor modifications due to the specific wiring of an off the shelf MPG. Please verify your specific MPG wiring
 * against the below diagram. 
 *   ________________________________________
 *  / GND  A-   B-   YIN  4    X10   ES      \
 * / 5v   A+   B+   XIN  ZIN  X100  5    COM  \
 * --------------------------------------------
 *   ______________________
 *  / 9  10 11 12 13 14 15 \
 * / 1  2  3  4  5  6  7  8 \
 * --------------------------
 * Note: Because of the available pins this is slightly less capable than the Head of this fork. There is no way to control the LED, nor a way to differentiate between No Axis
 * and the 6th Axis. The X1 multiplier setting is possible with the adjusted logic in this fork by only checking the X10/X100 pins.

*** Pendant to Arduino Pro Micro connections ***

Function    D15 Pin    Pro Micro
5v          1          VCC
A+          2          D10
B+          3          D14
XIN         4          D2
ZIN         5          D4
X100        6          D6
Axis5       7          D9
GND/COM     8, 9, 16   GND
YIN         12         D3
Axis4       13         D5
X10         14         D7
EStop       15         D8

*** Arduino Pro Micro to Duet PanelDue connector connections ***

Pro Micro Duet
VCC       +5V
GND       GND
TX1/D0    Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND

To connect a PanelDue as well:

PanelDue +5V to +5V/VCC
PanelDue GND to GND
PanelDue DIN to Duet UTXD or IO_0_OUT
PanelDue DOUT to /Pro Micro RX1/D0.

*** Arduino Nano to Duet PanelDue connector connections ***

Nano    Duet
+5V     +5V
GND     GND
TX1/D0  Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND

To connect a PanelDue as well:

PanelDue +5V to +5V
PanelDue GND to GND
PanelDue DIN to Duet UTXD or IO_0_OUT
PanelDue DOUT to Nano/Pro Micro RX1/D0.

On the Arduino Nano is necessary to replace the 1K resistor between the USB interface chip by a 10K resistor so that PanelDiue can override the USB chip.
On Arduino Nano clones with CH340G chip, it is also necessary to remove the RxD LED or its series resistor.

*/

// Configuration constants
const int PinA = 10;
const int PinB = 14;
const int PinX = 2;
const int PinY = 3;
const int PinZ = 4;
const int PinAxis4 = 5;
const int PinAxis5 = 9;
const int PinStop = 8;
const int PinTimes10 = 7;
const int PinTimes100 = 6;

const unsigned long BaudRate = 57600;
const int PulsesPerClick = 4;
const unsigned long MinCommandInterval = 20;
const int HoldTime = 1000;

// Table of commands we send, one entry for each axis
const char* const MoveCommands[] = {
  "G91 G0 F6000 X",     // X axis
  "G91 G0 F6000 Y",     // Y axis
  "G91 G0 F600 Z"      // Z axis
};

const char* Home = "G28";
const char* Park = "G27";
const char* EStop = "M112";
const char* Recover = "M999";

#include "RotaryEncoder.h"
#include "GCodeSerial.h"
#include "PassThrough.h"

RotaryEncoder encoder(PinA, PinB, PulsesPerClick);
PassThrough passThrough;

int serialBufferSize;
uint32_t whenLastCommandSent = 0;

const int axisPins[] = { PinX, PinY, PinZ, PinAxis4, PinAxis5 };

// Auto count axis array
const int numAxis = sizeof(axisPins) / sizeof(int);

#if defined(__AVR_ATmega32U4__)     // Arduino Leonardo or Pro Micro
# define UartSerial   Serial1
#elif defined(__AVR_ATmega328P__)   // Arduino Uno or Nano
# define UartSerial   Serial
#endif

GCodeSerial output(UartSerial);

// Prototype
void sendCommand(char* command, int distance=0);

// Setup
void setup() {
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  pinMode(PinX, INPUT_PULLUP);
  pinMode(PinY, INPUT_PULLUP);
  pinMode(PinZ, INPUT_PULLUP);
  pinMode(PinAxis4, INPUT_PULLUP);
  pinMode(PinAxis5, INPUT_PULLUP);
  pinMode(PinTimes10, INPUT_PULLUP);
  pinMode(PinTimes100, INPUT_PULLUP);
  pinMode(PinStop, INPUT_PULLUP);

  output.begin(BaudRate);

  serialBufferSize = output.availableForWrite();

#if defined(__AVR_ATmega32U4__)     // Arduino Leonardo or Pro Micro
  TX_RX_LED_INIT;
#endif
}

// Check for received data from PanelDue, store it in the pass through buffer, and send it if we have a complete command
void checkPassThrough() {
  unsigned int commandLength = passThrough.Check(UartSerial);
  if (commandLength != 0 && UartSerial.availableForWrite() == serialBufferSize) output.write(passThrough.GetCommand(), commandLength);
}

void sendCommand(char* command, int distance) {
#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
  TXLED0;                           // turn on transmit LED
#endif

  whenLastCommandSent = millis();
  output.write(command);
  if (distance != 0) output.print(distance/10.); // Need to send the binary of the ascii representation of the float rather than the binary of the float itself
  output.write('\n');
}

int pollDistanceMultiplier() {
  if (digitalRead(PinTimes10) == LOW) return 10;
  else if (digitalRead(PinTimes100) == LOW) return 100;
  return 1;
}

int pollAxis() {
  for (int i=0; i<numAxis; i++) if (digitalRead(axisPins[i]) == LOW) return i;
  return -1;
}

void eStop() {
    do {
      sendCommand(EStop);
      uint16_t now = (uint16_t)millis();

      while (digitalRead(PinStop) == HIGH && (uint16_t)millis() - now < 2000) checkPassThrough();

      encoder.getChange();      // ignore any movement
    } while (digitalRead(PinStop) == HIGH);

    sendCommand(Recover);
}

void loop() {
#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
    TXLED1;                         // turn off transmit LED
#endif

  // 0. Poll the encoder. Ideally we would do this in the tick ISR, but after all these years the Arduino core STILL doesn't let us hook it.
  // We could possibly use interrupts instead, but if the encoder suffers from contact bounce then that isn't a good idea.
  // In practice this loop executes fast enough that polling it here works well enough
  encoder.poll();

  if (output.availableForWrite() == serialBufferSize) { // No sense checking inputs and doing calculations if we can't send a command

    // 1. Check for emergency stop
    if (digitalRead(PinStop) == HIGH) eStop();

    // 2. Poll the feed amount switch
    int distanceMultiplier = pollDistanceMultiplier();

    // 3. Poll the axis selector switch
    int axis = pollAxis();
    
    // 4. Send a command based on the accumulated data if criteria is met.
    if ((uint32_t)millis() - whenLastCommandSent >= MinCommandInterval) {
      int distance = encoder.getChange() * distanceMultiplier;
      if (axis == 4) { // No Axis 5, so see if we should home. Distance is discarded.
        delay(HoldTime);
        if (digitalRead(PinAxis5) == LOW) {
          sendCommand(Home);
          delay(HoldTime); // Prevent Double Calls
        }
      } else if (axis == 3) { // No Axis 4, so see if we should park. Distance is discarded.
        delay(HoldTime);
        if (digitalRead(PinAxis4) == LOW) {
          sendCommand(Park);
          delay(HoldTime); // Prevent Double Calls
        }
      } else if (axis >= 0 && distance != 0) { // Move X, Y, or Z
        sendCommand(MoveCommands[axis], distance);
      }
    }
  }

  // 5. Check if there is passthrough data from the PanelDue
  checkPassThrough();
}

// End
