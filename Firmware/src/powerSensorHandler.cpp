#include "powerSensorHandler.h"
#include "Adafruit_INA219.h"
<<<<<<< HEAD
#include "config.h"
=======
#include "Config.h"
#include "debug.h"
>>>>>>> main

// Create an instance of the Adafruit INA219 power sensor class
Adafruit_INA219 ina219;

/**
 * @brief Initializes the power sensor and starts a background task for battery updates.
<<<<<<< HEAD
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
=======
 */
void initPowerSensor() {
    myWire.begin(SDAPin, SLCPin); // SDA on pin 23, SCL on pin 22

    if (!ina219.begin(&myWire)) {
        SerialCritical("INA219 power sensor not detected");
    } else {
        SerialLog("INA219 power sensor connected");
>>>>>>> main

        xTaskCreatePinnedToCore(
            updateBattery,   // Task function
            "updateBattery", // Name
<<<<<<< HEAD
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
        // display.updateBatteryDisplay(ina219.getBusVoltage_V());

        // Serial.println("update voltage");
=======
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

>>>>>>> main
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}