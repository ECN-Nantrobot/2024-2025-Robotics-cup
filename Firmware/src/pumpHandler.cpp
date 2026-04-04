<<<<<<< HEAD
#include "config.h"
#include "pumpHandler.h"
#include "debug.h"

void initPump()
{
=======
#include "Config.h"
#include "pumpHandler.h"
#include "debug.h"

void initPump() {
>>>>>>> main
    pinMode(pumpPin, OUTPUT);
    pinMode(vanPin, OUTPUT);

    digitalWrite(pumpPin, LOW);
    digitalWrite(vanPin, LOW);

    SerialSuccess("Pump initialized successfully");
}

<<<<<<< HEAD
void pumpActivate()
{
=======
void pumpActivate() {
>>>>>>> main
    digitalWrite(pumpPin, HIGH);
    digitalWrite(vanPin, LOW);
    SerialLog("Pump activated");
}

<<<<<<< HEAD
void pumpDeactivate()
{
=======
void pumpDeactivate() {
>>>>>>> main
    digitalWrite(pumpPin, LOW);
    digitalWrite(vanPin, HIGH);
    unsigned long startTime = millis();
    vTaskDelay(500);
    digitalWrite(vanPin, LOW);
    SerialLog("Pump deactivated (valve purge complete)");
}

<<<<<<< HEAD
void testPump()
{
=======
void testPump() {
>>>>>>> main
    SerialLog("Testing pump...");
    pumpActivate();
    vTaskDelay(1000); // 1s test duration
    pumpDeactivate();
    SerialSuccess("Pump test complete");
<<<<<<< HEAD
}
=======
}
>>>>>>> main
