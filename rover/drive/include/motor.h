#include <Arduino.h>

class Motor
{
public:
    int pinA = 7;
    int pinB = 5;
    int dataPin = 9;

    Motor(int pinA, int pinB, int dataPin)
    {
        this->pinA = pinA;
        this->pinB = pinB;
        this->dataPin = dataPin;

        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
        stop();
    }

    void move(int speed, bool direction)
    {
        digitalWrite(pinA, direction);
        digitalWrite(pinB, !direction);

        analogWrite(dataPin, speed);
    }

    void stop()
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        analogWrite(dataPin, 0);
    }

    void moveFor(int speed, bool direction, int time)
    {
        move(speed, direction);
        delay(time);
        stop();
    }
};