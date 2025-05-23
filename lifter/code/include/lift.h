#include <Arduino.h>
#include <motor.h>
#include <stepper.h>

Motor arm(1, 2);
Stepper rotate(1, 2, 3);

void LiftArm(int speed)
{
  arm.move(speed, dir);
  Serial.println("Lift Arm");
}

void LowerArm(int speed)
{
  arm.move(speed, dir);
  Serial.println("Lower Arm");
}

void ArmClockwise(int steps)
{
  rotate.move(steps, dir);
  Serial.println("Rotate Arm Clockwise");
}

void ArmCounterClockwise(int steps)
{
  rotate.move(steps, -dir);
  Serial.println("Rotate Arm Counter Clockwise");
}
