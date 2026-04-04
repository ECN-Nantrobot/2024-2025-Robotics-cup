/**
 * @file powerSensorHandler.h
 * @brief Header file to use the ina219 power sensor
 * 
 * Dependencies:
 * 
 * Usage example:
 * @code
 * display.updateDisplay(1, 12.5, 100.0);
 * @endcode
 * 
 * @note 
 * Remaining tasks are commented with the TODO tag.
 * 
 * @version 1.0
 * @date 02/03/2025
 * 
 * @author
 * Alexis MORICE
 */
#ifndef POWERSENSORHANDLER_H
#define POWERSENSORHANDLER_H


void initPowerSensor();
void updateBattery(void *pvParameters);


#endif 

