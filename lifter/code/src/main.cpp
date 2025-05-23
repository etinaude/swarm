#include <Arduino.h>
#include <motor.h>

Motor rightFront(1, 2);
Motor rightBack(41, 42);

Motor leftFront(4, 5);
Motor leftBack(7, 6);

int defaultSpeed = 255;

void MoveFwd(int speed)
{
  rightFront.move(speed, dir);
  rightBack.move(speed, dir);
  leftFront.move(speed, dir);
  leftBack.move(speed, dir);
}

void MoveBack(int speed)
{
  rightFront.move(speed, -dir);
  rightBack.move(speed, -dir);
  leftFront.move(speed, -dir);
  leftBack.move(speed, -dir);
}

void MoveRight(int speed)
{
  rightFront.move(speed, -dir);
  rightBack.move(speed, dir);
  leftFront.move(speed, dir);
  leftBack.move(speed, -dir);
  Serial.println("Rotate Right");
}

void MoveLeft(int speed)
{
  rightFront.move(speed, dir);
  rightBack.move(speed, -dir);
  leftFront.move(speed, -dir);
  leftBack.move(speed, dir);
  Serial.println("Rotate Left");
}

void RotateClockwise(int speed)
{
  rightFront.move(speed, -dir);
  rightBack.move(speed, -dir);
  leftFront.move(speed, dir);
  leftBack.move(speed, dir);

  Serial.println("Rotate Clockwise");
}

void RotateCounterClockwise(int speed)
{
  rightFront.move(speed, dir);
  rightBack.move(speed, dir);
  leftFront.move(speed, -dir);
  leftBack.move(speed, -dir);
  Serial.println("Rotate Counter Clockwise");
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
    else if (op == 'c')
    {
      RotateClockwise(defaultSpeed);
    }
    else if (op == 'o')
    {
      RotateCounterClockwise(defaultSpeed);
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
