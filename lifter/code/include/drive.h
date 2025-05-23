#include <Arduino.h>
#include <motor.h>
#include <stepper.h>

Motor rightFront(1, 2);
Motor rightBack(41, 42);

Motor leftFront(4, 5);
Motor leftBack(7, 6);

Motor arm(1, 2);
Stepper rotate(1, 2, 3);

int defaultSpeed = 255;

void MoveFwd(int speed)
{
  rightFront.move(speed, 1);
  rightBack.move(speed, 1);
  leftFront.move(speed, 1);
  leftBack.move(speed, 1);
}

void MoveBack(int speed)
{
  rightFront.move(speed, 0);
  rightBack.move(speed, 0);
  leftFront.move(speed, 0);
  leftBack.move(speed, 0);
}

void MoveRight(int speed)
{
  rightFront.move(speed, 0);
  rightBack.move(speed, 1);
  leftFront.move(speed, 1);
  leftBack.move(speed, 0);
  Serial.println("Rotate Right");
}

void MoveLeft(int speed)
{
  rightFront.move(speed, 1);
  rightBack.move(speed, 0);
  leftFront.move(speed, 0);
  leftBack.move(speed, 1);
  Serial.println("Rotate Left");
}

void RotateClockwise(int speed)
{
  rightFront.move(speed, 0);
  rightBack.move(speed, 0);
  leftFront.move(speed, 1);
  leftBack.move(speed, 1);

  Serial.println("Rotate Clockwise");
}

void RotateCounterClockwise(int speed)
{
  rightFront.move(speed, 1);
  rightBack.move(speed, 1);
  leftFront.move(speed, 0);
  leftBack.move(speed, 0);
  Serial.println("Rotate Counter Clockwise");
}

void LiftArm(int speed)
{
  arm.move(speed, 1);
  Serial.println("Lift Arm");
}

void LowerArm(int speed)
{
  arm.move(speed, 0);
  Serial.println("Lower Arm");
}

void ArmClockwise(int steps)
{
  rotate.move(steps, 1);
  Serial.println("Rotate Arm Clockwise");
}

void ArmCounterClockwise(int steps)
{
  rotate.move(steps, 0);
  Serial.println("Rotate Arm Counter Clockwise");
}
