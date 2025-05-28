#include "controlPanelHandler.h"
#include "config.h"
#include "debug.h"
#include <Wire.h>

#include "stacking.h"

#define PCF8574_ADDRESS 0x20

void scanI2C()
{
    SerialInfo("Scanning I2C bus...");
    byte count = 0;

    for (byte address = 1; address < 127; address++)
    {
        myWire.beginTransmission(address);
        if (myWire.endTransmission() == 0)
        {
            Serial.print("I2C device found at 0x");
            Serial.println(address, HEX);
            count++;
        }
        delay(2);
    }

    if (count == 0)
    {
        SerialWarning("No I2C devices found.");
    }
    else
    {
        SerialSuccess("I2C scan complete.");
    }
}

void initControlPanel()
{

    myWire.begin(21, 22); // SDA, SCL

    scanI2C(); // Run I2C scan before initializing PCF8574

    myWire.beginTransmission(PCF8574_ADDRESS);
    if (myWire.endTransmission() != 0)
    {
        Serial.print(F("Le PCF8574 ne répond pas à l'adresse 0x"));
        Serial.println(PCF8574_ADDRESS, HEX);
    }

    myWire.beginTransmission(PCF8574_ADDRESS);
    myWire.write(0b11111111); // Chaque "1" représente respectivement P7, P6, P5, P4, P3, P2, P1, et P0
    myWire.endTransmission();

    myWire.setTimeOut(1000);
    delay(500);
    SerialSuccess("PCF8574 control panel initialized");

    xTaskCreatePinnedToCore(
        controlPanel,   // Task function
        "controlPanel", // Name
        2048,           // Stack size
        NULL,           // Parameters
        1,              // Priority
        NULL,           // Task handle
        0               // CPU core (0)
    );
}

byte reponseDuPCF8574;
bool isTestInProgress = false;
int testCurrentStep = 0;

void controlPanel(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
        {
            myWire.requestFrom(0x20, 1); // On interroge le PCF8574, en lui demandant de nous retourner 1 octet (qui contiendra l'état des broches P0 à P7)

            if (myWire.available())
            {
                reponseDuPCF8574 = myWire.read();
                xSemaphoreGive(i2cMutex);
            }

            // Sarter
            if (!(reponseDuPCF8574 & (1 << 5))){
                Serial.println(F("  ==> Interruption depuis la broche P5"));
            }
            // // Switch Fonction
            // if (!(reponseDuPCF8574 & (1 << 4))) Serial.println(F("  ==> Interruption depuis la broche P4"));
            // // Bouton YELLOW
            if (!(reponseDuPCF8574 & (1 << 3))) 
            {
                Serial.println(F("  ==> Interruption depuis la broche P3"));
                is_blue = false;
            }
            else {
                is_blue = true;
            }

            // Bouton RESET
            if (!(reponseDuPCF8574 & (1 << 2)))
            {
                if (isTestInProgress)
                {
                    SerialInfo("Test Restarted");
                    testCurrentStep -= 1;
                    testAll();
                }
                else
                {
                    ESP.restart();
                }
            }

            // Bouton STOP
            if (!(reponseDuPCF8574 & (1 << 1)))
            {
                if (isTestInProgress)
                {
                    isTestInProgress = false;
                    testCurrentStep = 0;
                    // TODO: Stop test on screen
                    SerialWarning("===== All test aborted =======");
                }
            };

            // Bouton TEST
            if (!(reponseDuPCF8574 & (1 << 0)))
            {

                // if we haven't started a test we start one
                if (!isTestInProgress)
                {
                    SerialInfo("===== Start all test =======");

                    // if switch function is UP just a stacking test
                    if (!(reponseDuPCF8574 & (1 << 3)))
                    {
                        Serial.println("quick test");

                        // else test all function with test button after each
                    }
                    else
                    {
                        isTestInProgress = true;
                        testAll();
                    }
                }

                // if the test as already started, go to the next
                else
                {
                    testAll();
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}