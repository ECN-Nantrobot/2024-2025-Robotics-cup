#ifndef CONTROL_PANEL_HANDLER_H
#define CONTROL_PANEL_HANDLER_H

#include <Arduino.h>

void initControlPanel();
void controlPanel(void *pvParameters);
void startProcess(void *pvParameters);

#endif