#include <Arduino.h>
#include "motor.h"

Motor grip = Motor("grip", 2, 3);
Motor lift = Motor("lift", 2, 3);

int steps = 0;
Motor *motor = &grip;
bool shaft = true;
bool activeInstruction = false;

void readSerial()
{
  if (Serial.available())
  {
    // <motor> <direction> <steps>
    // G 0 200
    String s = Serial.readStringUntil('\n');
    Serial.println(s);

    if (s[0] == 'G')
      motor = &grip;
    else if (s[0] == 'L')
      motor = &lift;
    else
    {
      Serial.println("Invalid motor");
      return;
    }

    if (s[2] == '0')
      shaft = false;
    else if (s[2] == '1')
      shaft = true;
    else
    {
      Serial.println("Invalid shaft");
      return;
    }

    steps = s.substring(4).toInt();

    activeInstruction = true;
  }
}

void setup()
{
  grip.setupSteppers();
  lift.setupSteppers();
  Serial.begin(115200);
  Serial.println("Ready");
  Serial.setTimeout(100);
}

void loop()
{
  readSerial();
  if (activeInstruction)
  {
    (*motor).moveStepper(steps, shaft);
    (*motor).printStatus();
    activeInstruction = false;
  }

  delay(100);
}
