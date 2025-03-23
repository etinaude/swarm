#include <Arduino.h>
#include <TMCStepper.h>

// #define R_SENSE 0.11f       // Match to your driver
// #define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

class Motor
{

public:
    int DIR_PIN = 3;  // Direction
    int STEP_PIN = 3; // Step
    int totalSteps = 0;
    int RX = 0;
    int TX = 0;
    String NAME = "";

    // TMC2209Stepper driver = TMC2209Stepper(&Serial1, R_SENSE, DRIVER_ADDRESS);

    Motor(String name, int dir, int step)
    {
        DIR_PIN = dir;
        STEP_PIN = step;
        NAME = name;
    }

    Motor(String name, int dir, int step, int cs, int tx, int rx)
    {
        DIR_PIN = dir;
        STEP_PIN = step;
        NAME = name;
        RX = rx;
        TX = tx;
    }

    void setupSteppers()
    {
        pinMode(STEP_PIN, OUTPUT);
        pinMode(DIR_PIN, OUTPUT);

        // driver.begin();          //  SPI: Init CS pins and possible SW SPI pins
        //                          // UART: Init SW UART (if selected) with default 115200 baudrate
        // driver.toff(5);          // Enables driver in software
        // driver.rms_current(600); // Set motor RMS current
        // driver.microsteps(8);    // Set microsteps to 1/16th

        // // driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
        // // driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
        // driver.pwm_autoscale(true); // Needed for stealthChop
    }

    void moveStepper(int steps, bool dir)
    {
        // set pins
        // Serial1.begin(115200, SERIAL_8N1, RX, TX);

        for (int i = steps; i > 0; i--)
        {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(160);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(160);
        }

        if (dir)
            totalSteps += steps;
        else
            totalSteps -= steps;
    }

    void resetStepper()
    {
        moveStepper(totalSteps, false);
        totalSteps = 0;
    }

    void printStatus()
    {
        Serial.print("Motor: ");
        Serial.print(NAME);
        Serial.print("\t\tTotal: ");
        Serial.println(totalSteps);
    }
};