// CNC pendant interface to Duet
// D Crocker, started 2020-05-04

/* This Arduino sketch can be run on either Arduino Nano or Arduino Pro Micro. 
 * It should alo work on an Arduino Uno (using the same wiring scheme as for the Nano) or Arduino Leonardo (using the same wiring scheme as for the Pro Micro).
 * The recommended board is the Arduino Pro Micro because the passthrough works without any modificatoins to the Arduino. 

/* This is a fork meant to adapt a D15 ended MPG. This has minor modifications due to the specific wiring of an off the shelf MPG. Please verify your specific MPG wiring
 * against the below diagram. 
 *   ________________________________________
 *  / GND  A-   B-   YIN  4    X10   EP      \
 * / 5v   A+   B+   XIN  ZIN  X100  5    COM  \
 * --------------------------------------------
 *   ______________________
 *  / 9  10 11 12 13 14 15 \
 * / 1  2  3  4  5  6  7  8 \
 * --------------------------
 * Note: Because of the available pins this is slightly less capable than the Head of this fork. There is no way to control the LED, nor a way to differentiate between No Axis
 * and the 6th Axis. The X1 multiplier setting is possible with the adjusted logic in this fork by only checking the X10/X100 pins.

*** Pendant to Arduino Pro Micro connections ***

Pro Micro D15 Pin
VCC       1
GND       8, 9
D2        2
D3        3
D4        4
D5        12
D6        5
D7        13
D8        7
A0        15
A2        14
A3        6

Not Used  10/11

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

*** Pendant to Arduino Nano connections ***

Nano    D15
+5V     1
GND     8, 9
D2      2
D3      3
D4      4
D5      12
D6      5
D7      13
D8      7
D11     14
D12     6
A0      15

Not Used  10/11

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
const int PinA = 2;
const int PinB = 3;
const int PinX = 4;
const int PinY = 5;
const int PinZ = 6;
const int PinAxis4 = 7;
const int PinAxis5 = 8;
const int PinStop = A0;

#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
const int PinTimes10 = A2;
const int PinTimes100 = A3;
#endif

#if defined(__AVR_ATmega328P__)     // Arduino Nano or Uno
const int PinTimes10 = 11;
const int PinTimes100 = 12;
#endif


const unsigned long BaudRate = 57600;
const int PulsesPerClick = 4;
const unsigned long MinCommandInterval = 20;

// Table of commands we send, one entry for each axis
const char* const MoveCommands[] =
{
  "G91 G0 F6000 X",     // X axis
  "G91 G0 F6000 Y",     // Y axis
  "G91 G0 F600 Z",      // Z axis
  "G91 G0 F6000 U",     // axis 4
  "G91 G0 F6000 V"      // axis 5
};

#include "RotaryEncoder.h"
#include "GCodeSerial.h"
#include "PassThrough.h"

RotaryEncoder encoder(PinA, PinB, PulsesPerClick);
PassThrough passThrough;

int serialBufferSize;
int distanceMultiplier;
int axis;
uint32_t whenLastCommandSent = 0;

const int axisPins[] = { PinX, PinY, PinZ, PinAxis4, PinAxis5 };

#if defined(__AVR_ATmega32U4__)     // Arduino Leonardo or Pro Micro
# define UartSerial   Serial1
#elif defined(__AVR_ATmega328P__)   // Arduino Uno or Nano
# define UartSerial   Serial
#endif

GCodeSerial output(UartSerial);

void setup()
{
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
void checkPassThrough()
{
  unsigned int commandLength = passThrough.Check(UartSerial);
  if (commandLength != 0 && UartSerial.availableForWrite() == serialBufferSize)
  {
    output.write(passThrough.GetCommand(), commandLength);
  }
}

void loop()
{
  // 0. Poll the encoder. Ideally we would do this in the tick ISR, but after all these years the Arduino core STILL doesn't let us hook it.
  // We could possibly use interrupts instead, but if the encoder suffers from contact bounce then that isn't a good idea.
  // In practice this loop executes fast enough that polling it here works well enough
  encoder.poll();

  // 1. Check for emergency stop
  if (digitalRead(PinStop) == HIGH)
  {
    // Send emergency stop command every 2 seconds
    do
    {
      output.write("M112 ;" "\xF0" "\x0F" "\n");
      uint16_t now = (uint16_t)millis();
      while (digitalRead(PinStop) == HIGH && (uint16_t)millis() - now < 2000)
      {
        checkPassThrough();
      }
      encoder.getChange();      // ignore any movement
    } while (digitalRead(PinStop) == HIGH);

    output.write("M999\n");
  }

  // 2. Poll the feed amount switch
  distanceMultiplier = 1;
  if (digitalRead(PinTimes10) == LOW) {
    distanceMultiplier = 10;
  } else if (digitalRead(PinTimes100) == LOW) {
    distanceMultiplier = 100;
  }

  // 3. Poll the axis selector switch
  axis = -1;
  int localAxis = 0;
  for (int pin : axisPins)
  {
    if (digitalRead(pin) == LOW)
    {
      axis = localAxis;
      break;
    }
    ++localAxis;    
  }
  
  // 5. If the serial output buffer is empty, send a G0 command for the accumulated encoder motion.
  if (output.availableForWrite() == serialBufferSize)
  {
#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
    TXLED1;                         // turn off transmit LED
#endif
    const uint32_t now = millis();
    if (now - whenLastCommandSent >= MinCommandInterval)
    {
      int distance = encoder.getChange() * distanceMultiplier;
      if (axis >= 0 && distance != 0)
      {
#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
        TXLED0;                     // turn on transmit LED
#endif
        whenLastCommandSent = now;
        output.write(MoveCommands[axis]);
        if (distance < 0)
        {
          output.write('-');
          distance = -distance;
        }
        output.print(distance/10);
        output.write('.');
        output.print(distance % 10);
        output.write('\n');
      }
    }
  }

  checkPassThrough();
}

// End
