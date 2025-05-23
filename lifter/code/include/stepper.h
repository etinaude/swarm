#include <Arduino.h>

class Stepper
{
public:
    int speed = 500;
    bool enable = false;
    bool direction = false;
    int stepPin = 7;
    int dirPin = 5;
    int enPin = 9;

    Stepper(int enPin, int stepPin, int dirPin)
    {
        this->stepPin = stepPin;
        this->dirPin = dirPin;
        this->enPin = enPin;

        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enPin, OUTPUT);
        digitalWrite(enPin, LOW); // Enable the motor driver
    }

    void move(int steps, bool direction)
    {
        digitalWrite(stepPin, direction);
        for (int i = 0; i < steps; i++)
        {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(speed);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(speed);
        }
        Serial.print("Moved ");
    }

    void setSpeed(int speedIn)
    {
        if (speedIn > 0 && speedIn < 10000)
        {
            speed = speedIn;
            Serial.println(speed);
        }
        else
        {
            Serial.println("Invalid speed. Must be between 1 and 10000.");
        }
    }

    void setEnable(bool en)
    {
        enable = en;
        digitalWrite(enPin, enable ? LOW : HIGH);
        Serial.print("Motor ");
        Serial.println(enable ? "enabled" : "disabled");
    }
};