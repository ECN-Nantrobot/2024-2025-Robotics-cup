#include "powerSensorHandler.h"
#include "Adafruit_INA219.h"
#include "Config.h"
#include "debug.h"

// Create an instance of the Adafruit INA219 power sensor class
Adafruit_INA219 ina219;

/**
 * @brief Initializes the power sensor and starts a background task for battery updates.
 */
void initPowerSensor() {
    myWire.begin(SDAPin, SLCPin); // SDA on pin 23, SCL on pin 22

    if (!ina219.begin(&myWire)) {
        SerialCritical("INA219 power sensor not detected");
    } else {
        SerialLog("INA219 power sensor connected");

        xTaskCreatePinnedToCore(
            updateBattery,   // Task function
            "updateBattery", // Name
            2048,            // Stack size
            NULL,            // Parameters
            1,               // Priority
            NULL,            // Task handle
            0                // CPU core (0)
        );

        SerialSuccess("Power sensor initialization complete");
    }
}

/**
 * @brief Task function to periodically update the battery voltage.
 */
void updateBattery(void *pvParameters) {
    while (1) {
        float voltage = 0.0;

        if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){
            voltage = ina219.getBusVoltage_V();
            xSemaphoreGive(i2cMutex);
        }

        display.updateBatteryDisplay(voltage);
        SerialLog("Battery voltage updated: " + String(voltage) + " V");

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}