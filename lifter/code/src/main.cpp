#include <Arduino.h>
#include <drive.h>

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);
}

void Stop()
{
  arm.stop();
  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();
  Serial.println("STOP");
  delay(500);
}

void readSerial()
{
  if (Serial.available())
  {
    String s = Serial.readStringUntil('\n');
    s.trim();
    char op = s[0];
    Serial.println(op);

    if (op == 'f')
    {
      MoveFwd(defaultSpeed);
    }
    else if (op == 'b')
    {
      MoveBack(defaultSpeed);
    }
    else if (op == 'l')
    {
      MoveLeft(defaultSpeed);
    }
    else if (op == 'r')
    {
      MoveRight(defaultSpeed);
    }
    else if (op == 'c')
    {
      RotateClockwise(defaultSpeed);
    }
    else if (op == 'o')
    {
      RotateCounterClockwise(defaultSpeed);
    }

    else if (op == 'u')
    {
      LiftArm(defaultSpeed);
    }
    else if (op == 'd')
    {
      LowerArm(defaultSpeed);
    }
    else if (op == 'a')
    {
      ArmClockwise(defaultSpeed);
    }
    else if (op == 'd')
    {
      ArmCounterClockwise(defaultSpeed);
    }
    else if (op == 'e')
    {
      rotate.setEnable(!rotate.enable);
    }

    else if (op == 's')
    {
      Stop();
    }
  }
}

void loop()
{
  readSerial();
}