#include <Arduino.h>

class Motor
{
public:
    int pinA = 7;
    int pinB = 5;

    Motor(int pinA, int pinB)
    {
        this->pinA = pinA;
        this->pinB = pinB;

        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
        stop();
    }

    void move(int speed, bool direction)
    {
        digitalWrite(pinA, direction);
        digitalWrite(pinB, !direction);
    }

    void stop()
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
    }

    void moveFor(int speed, bool direction, int time)
    {
        move(speed, direction);
        delay(time);
        stop();
    }
};