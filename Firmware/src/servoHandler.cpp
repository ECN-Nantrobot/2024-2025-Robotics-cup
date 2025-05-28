#include "config.h"
#include "servoHandler.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "debug.h"

#define SERVOMIN 190 // Minimum pulse length
#define SERVOMAX 800 // Maximum pulse length

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40, myWire);

int angleToPulse(int ang)
{
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    SerialLog("Angle to pulse: angle = " + String(ang) + ", pulse = " + String(pulse));
    return pulse;
}

void initServo()
{
    myWire.begin(SDAPin, SLCPin);

    if (!board1.begin())
    {
        SerialCritical("Servo driver not initialized");
    }
    else
    {
        SerialLog("Servo driver initialized");
    }

    board1.setPWMFreq(60); // Set PWM frequency for servos

    // Initialize servos 0-3 to 0
    for (int i = 0; i < 4; i++)
    {
        board1.setPWM(i, 0, angleToPulse(0));
        delay(20);
    }
    // Initialize servo 4 to 180
    board1.setPWM(4, 0, angleToPulse(130));
    delay(20);

    SerialSuccess("Servo initialization complete");
}

void setServo(int servo, int value)
{
    if (value < 0)
        value = 0;
    if (value > 180)
        value = 110;

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
        SerialLog("Set servo " + String(servo) + " to " + String(value));
        board1.setPWM(servo, 0, angleToPulse(value));
        xSemaphoreGive(i2cMutex);
    }
}

void testServo()
{
    SerialLog("Testing servos...");

    // Move all servos to 90 (except 4 to 90 from 180)
    for (int i = 0; i <= 4; i++)
    {
        setServo(i, 90);
    }
    vTaskDelay(3000); // Hold position

    // Return them to initial state (0 or 180 for #4)
    for (int i = 0; i <= 3; i++)
    {
        setServo(i, 0);
    }
    setServo(4, 180);
    vTaskDelay(3000); // Hold position

    SerialSuccess("Servo test complete");
}