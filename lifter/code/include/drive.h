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
