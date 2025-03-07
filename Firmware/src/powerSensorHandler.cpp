#include "powerSensorHandler.h"
#include "Adafruit_INA219.h"
#include "Config.h"

// Create an instance of the Adafruit INA219 power sensor class
Adafruit_INA219 ina219;

/**
 * @brief Initializes the power sensor and starts a background task for battery updates.
 * 
 * This function sets up the I2C communication for the INA219 sensor and checks if 
 * the sensor is detected. If successful, it starts an asynchronous FreeRTOS task 
 * (`updateBattery`) to periodically update battery readings.
 */
void initPowerSensor(){

    myWire.begin(SDAPin, SLCPin); // SDA on pin 23, SCL on pin 22


    if (!ina219.begin(&myWire)) {
        Serial.println("Failed to find INA219 chip");
    }
    else{
        // If the sensor is successfully initialized,
        // start a background task that updates battery voltage every 3 seconds.

        xTaskCreatePinnedToCore(
            updateBattery,   // Task function
            "updateBattery", // Name
            2048,          // Stack size
            NULL,          // Parameters
            1,             // Priority
            NULL,          // Task handle
            0              // CPU core (0)
        );
    }
}


/**
 * @brief Task function to periodically update the battery voltage.
 * 
 * This FreeRTOS task runs indefinitely, updating the battery voltage every 3 seconds.
 * It is intended to be executed on a separate CPU core to avoid blocking other operations.
 *
 * @param pvParameters Unused task parameters (required by FreeRTOS but not used here).
 */
void updateBattery(void *pvParameters){
    while(1){
        // Update the voltage variable in the display object, 
        // the display object has it's own freeRTOS task to periodiclly update the actual screen
        display.updateBatteryDisplay(ina219.getBusVoltage_V());

        // Serial.println("update voltage");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}