/**
 * @file displayHandler.h
 * @brief Header file for the DisplayHandler class.
 * 
 * Dependencies:
 * - TFT_eSPI library for display control.
 * - SPI library for communication.
 * - ecn_logo.h for the logo bitmap.
 * - displayConfig.h for display & class configuration constants.
 * 
 * Usage example:
 * @code
 * ecn::DisplayHandler display;
 * display.initDisplay();
 * display.updateDisplay(1, 12.5, 100.0);
 * @endcode
 * 
 * @note Currently displayHandler is only configured for the GC9A01 display controller.
 * Ensure that the TFT_eSPI library is properly configured for your hardware.
 * Remaining tasks are commented with the TODO tag.
 * 
 * @version 1.0
 * @date 02/02/2025
 * 
 * @author
 * Rohit Panikar
 */
#ifndef DISPLAYHANDLER_H
#define DISPLAYHANDLER_H

#include <TFT_eSPI.h>       // Hardware-specific library
#include <SPI.h>
#include <ecn_logo.h>
#include <displayConfig.h>
#include <Config.h>

namespace ecn
{
    class DisplayHandler
    {
        TFT_eSPI tft = TFT_eSPI();
        
        const int center[2] = {120, 120};
        const int logoPos[2] = {center[0]/2, center[1]/2};

        const char *statusText[4] = {"Status", "IDLE", "WKG", "ERR"};
        // Manual offsets for status text and indicator:
        const int statusPos[3][2] = {
            {center[0] + 45, center[1] - 60}, // Position for "Status:"
            {center[0] + 40, center[1] - 40}, // Position for "IDLE", "WKG", "ERR"
            {center[0] + 30, center[1] - 75}, // Position for status indicator
        };

        // Fix battery text and position
        const char *batteryText[4] = {"Battery", "V", "mA", "%"};
        const int batteryPos[5][2] = {
            {center[0] - 75, center[1] - 60}, // Position for "Battery:"
            {center[0] - 90, center[1] - 40}, // Position for "V: "
            {center[0] - 80, center[1] - 20}, // Position for "I: "
            {center[0] - 30, center[1] - 75}, // Position for battery indicator
        };

        // Fix points text and position
        const int pointsPos[2][2] = {
            {center[0]-25, center[1]}, // Position for "Points:"
            {center[0]-60, center[1]+15}, // Position for points value
            };

        // Fix version text and position
        const int versionPos[2] = {center[0]-20, center[1]+80};

        public:
        DisplayHandler(){};

        // Main display variables
        int status = 1;
        float voltage = 0.0;
        float points = 0.0;
        char version[10] = "v0.0";

        DisplayHandler(int status, float voltage, float points, char version[10])
        {
            this->status = status;
            this->voltage = voltage;
            this->points = points;
            strcpy(this->version, version); //TODO: too slow!
        }
        
        void initDisplay(bool flip = true, bool flash = false, int flashDelay = 2000)
        {
            wireDisplay.begin(displaySDA, displaySLC); // SDA on pin 19, SCL on pin 21

            tft.init();
            if(flip){
                tft.setRotation(6);
            }

            /****** Flash screen routine ******/
            if(flash){
                tft.fillScreen(TFT_BLACK);
                unsigned long start = millis();
                drawLogo();
                while(millis() - start < flashDelay){} // Stay here twiddling thumbs waiting
            }
            /*********************************/
            
            tft.fillScreen(TFT_BLACK);
            tft.drawCircle(120, 120, int32_t(tft.width()*0.47), TFT_WHITE); // Outer circle
            // Static prints:
            drawText(statusText[0], statusPos[0][0], statusPos[0][1], 2, TFT_WHITE); 
            drawText(batteryText[0], batteryPos[0][0], batteryPos[0][1], 2, TFT_WHITE); 
            drawText("Points: ", pointsPos[0][0], pointsPos[0][1], 2, TFT_WHITE, 1);
            drawText(version, versionPos[0], versionPos[1], 2, TFT_WHITE, 1);
            
            updateDisplay(status, voltage, points);

            /****** Automatically update the screen every 3s ******/

            xTaskCreatePinnedToCore(
                autoUpdateDisplay,   // Task function
                "autoUpdateDisplay", // Name
                2048,          // Stack size
                this,          // Parameters
                1,             // Priority
                NULL,          // Task handle
                0              // CPU core (0)
            );



        }

        // Draw bitmap with top left corner at x,y with foreground only color.
        // Bits set to 1 plot as the defined color, bits set to 0 are not plotted.
        void drawLogo()
        {
            //              x           y           xbm        xbmWidth  xbmHeight   color
            tft.drawXBitmap(logoPos[0], logoPos[1], ecn_logo, logoWidth, logoHeight, TFT_WHITE);
        }

        void drawText(const char *text, int x, int y, int font, uint16_t color, int font_size = 1){
            tft.setCursor(int16_t(x), int16_t(y), uint8_t(font));
            tft.setTextColor(color);
            tft.setTextSize(uint8_t(font_size));
            tft.println(text);
        }

        void updateStatusDisplay(int status){
            this->status = status;
        }

        void updateBatteryDisplay(float voltage){
            this->voltage = voltage;
        }

        void updatePointsDisplay(float points){
            this->points = points;
        }

        void _updateStatusDisplay(int status){
            // find length of statusText array
            if( status > sizeof(statusText)/sizeof(statusText[0]) or status < 1){
                status = 0; // TODO: implement error display instead
                return;
            }
            // Clear the previous status and update status using sprites
            tft.fillRect(statusPos[1][0], statusPos[1][1], 52, 40, TFT_BLACK);
            drawText(statusText[status], statusPos[1][0], statusPos[1][1], 2, TFT_WHITE, 2);

            // Update the status indicator
            if( status == 1){
                tft.fillCircle(statusPos[2][0], statusPos[2][1], 10, TFT_GREEN);
            }else if(status == 2){
                tft.fillCircle(statusPos[2][0], statusPos[2][1], 10, TFT_YELLOW);
            }else if(status == 3){
                tft.fillCircle(statusPos[2][0], statusPos[2][1], 10, TFT_RED);
            }
        }

        void _updateBatteryDisplay(float voltage)
        {
            char voltageStr[10];
            unsigned long start = millis();

            // TODO: maxVoltage is deined in displayConfig.h as #define maxVoltage 15.0,
            // here, we check if it's defined and update the battery indicator accordingly
            #ifdef maxVoltage
            // TODO: Alternate bet. battery percentage and voltage
            // float voltagePercentage = (voltage/maxVoltage)*100;
            
            if(voltage > 0.87*maxVoltage){
                tft.fillCircle(batteryPos[3][0], batteryPos[3][1], 10, TFT_GREEN);
            }else if(voltage < 0.87*maxVoltage){
                tft.fillCircle(batteryPos[3][0], batteryPos[3][1], 10, TFT_YELLOW);
            }else{
                tft.fillCircle(batteryPos[3][0], batteryPos[3][1], 10, TFT_RED);
            }
            #else
            tft.fillCircle(batteryPos[3][0], batteryPos[3][1], 10, TFT_GREEN);
            #endif

            // Draw the voltage text
            sprintf(voltageStr, "%0.1f%s", voltage, batteryText[1]); // Combine the string and float value such that it displays "12.6V"
            tft.fillRect(batteryPos[1][0], batteryPos[1][1], 90, 40, TFT_BLACK); // Clear the prev. voltage
            drawText(voltageStr, batteryPos[1][0], batteryPos[1][1], 2, TFT_WHITE, 2);
        }

        void _updatePointsDisplay(float points)
        {
            char pointsStr[10];
            sprintf(pointsStr, "%0.1f", points);
            tft.fillRect(pointsPos[1][0], pointsPos[1][1], 120, 60, TFT_BLACK);
            drawText(pointsStr, pointsPos[1][0], pointsPos[1][1], 2, TFT_WHITE, 4);
        }

        // TODO: Implement error display
        // void displayError(const char *error);
        
        void updateDisplay(int status, float voltage, float points)
        {
            this->voltage = voltage;
            this->status = voltage;
            this->points = points;
        }

        static void autoUpdateDisplay(void *pvParameters){
            if (pvParameters == NULL) {
                Serial.println("Error: NULL instance passed to autoUpdateDisplay!");
                vTaskDelete(NULL); // Terminate the task safely
                return;
            }
        
            DisplayHandler* instance = static_cast<DisplayHandler*>(pvParameters);
            while (true) {
                instance->_updateDisplay();
                Serial.println("update display");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        void _updateDisplay()
        {
            // Draw the status section
            _updateStatusDisplay(status);
            // Draw the battery section
            _updateBatteryDisplay(voltage);
            // Draw the points section
            _updatePointsDisplay(points);
        }
    };
}


#endif // DISPLAYHANDLER_H