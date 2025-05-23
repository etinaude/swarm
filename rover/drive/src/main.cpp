#include <Arduino.h>
#include <motor.h>

Motor rightFront(1, 2);
Motor rightBack(41, 42);

Motor leftFront(4, 5);
Motor leftBack(7, 6);

int defaultSpeed = 255;

void MoveRightSide(int speed, bool dir)
{
  rightFront.move(speed, dir);
  rightBack.move(speed, dir);
}

void MoveLeftSide(int speed, bool dir)
{
  leftFront.move(speed, dir);
  leftBack.move(speed, dir);
}

void MoveFwd(int speed)
{
  MoveRightSide(speed, true);
  MoveLeftSide(speed, true);
}

void MoveBack(int speed)
{
  MoveRightSide(speed, false);
  MoveLeftSide(speed, false);
}

void MoveRight(int speed)
{
  MoveRightSide(speed, false);
  MoveLeftSide(speed, true);
}

void MoveLeft(int speed)
{
  MoveRightSide(speed, true);
  MoveLeftSide(speed, false);
}

void Stop()
{

  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();
  Serial.println("STOP");
  delay(500);
}

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
