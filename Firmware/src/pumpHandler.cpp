#include "Config.h"
#include "pumpHandler.h"
#include "debug.h"

void initPump() {
    pinMode(pumpPin, OUTPUT);
    pinMode(vanPin, OUTPUT);

    digitalWrite(pumpPin, LOW);
    digitalWrite(vanPin, LOW);

    SerialSuccess("Pump initialized successfully");
}

void pumpActivate() {
    digitalWrite(pumpPin, HIGH);
    digitalWrite(vanPin, LOW);
    SerialLog("Pump activated");
}

void pumpDeactivate() {
    digitalWrite(pumpPin, LOW);
    digitalWrite(vanPin, HIGH);
    unsigned long startTime = millis();
    vTaskDelay(500);
    digitalWrite(vanPin, LOW);
    SerialLog("Pump deactivated (valve purge complete)");
}

void testPump() {
    SerialLog("Testing pump...");
    pumpActivate();
    vTaskDelay(1000); // 1s test duration
    pumpDeactivate();
    SerialSuccess("Pump test complete");
}
