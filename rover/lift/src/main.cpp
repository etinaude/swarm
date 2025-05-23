#include <Arduino.h>
#include <motor.h>

// OP <parm> <parm>
// eg
// l 0 1000
// u=up <dir> <steps>
// d=down <dir> <steps>
// s=speed <speed>
// e=enable <enable>

Motor lift2Motor(39, 37, 35);
Motor liftMotor(33, 18, 16);

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);
}

void readSerial()
{
  if (Serial.available())
  {
    String s = Serial.readStringUntil('\n');
    s.trim();
    char op = s[0];
    Serial.println(op);

    if (op == 'u')
    {
      int dir = s.substring(2, 3).toInt();
      int steps = s.substring(3).toInt();

      liftMotor.move(steps, dir);
      lift2Motor.move(steps, dir);
    }
    if (op == 'd')
    {
      int dir = s.substring(2, 3).toInt();
      int steps = s.substring(3).toInt();

      liftMotor.move(steps, -dir);
      lift2Motor.move(steps, -dir);
    }

    if (op == 's')
    {
      int speedIn = s.substring(1).toInt();
      liftMotor.setSpeed(speedIn);
      gripMotor.setSpeed(speedIn);
    }

    if (op == 'e')
    {
      int en = s.substring(1, 2).toInt();
      liftMotor.setEnable(en);
      gripMotor.setEnable(en);
    }
  }
}

void loop()
{
  readSerial();
  delay(100);
}