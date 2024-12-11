# ReFillit
**RefillIt** is an IoT-enabled box designed to streamline the restocking process in workshops. By simply tapping an RFID tag, users can select the desired amount of an article, automatically generating an order for that item. This system aims to make refilling consumables like screws and other small items effortless and efficient.

This project was developed as a school assignment to explore the capabilities of IoT devices in automating everyday tasks.

## Features
- **RFID Scanning**: Detects RFID tags to identify items quickly.
- **Quantity Selection**: Users can select the desired amount using a potentiometer.
- **Order Generation**: Automatically creates orders based on selections.
- **Visual Feedback**: RGB LEDs and TFT display provide real-time status updates.
- **Bluetooth Communication**: Enables wireless data transfer between ESP32 and desktop application.
- **Local Database Management**: Stores and manages RFID tag data in a JSON file.
- **Modular Codebase**: Organized into separate files for easier maintenance and scalability.

## Hardware Components
- **Microcontroller**: ESP32
- **RFID Reader**: MFRC522
- **Display**: TFT Display (GC9A01)
- **LED Indicators**: RGB LEDs
- **Input Devices**: Touch Button, Potentiometer
- **Power Supply**: Battery (3x AAA battery)

## Schematic and Circuit Diagram

![full schematics](https://github.com/user-attachments/assets/8d81ea13-5f77-4dce-86d2-5da6df2af4c6)

Initially, the circuit was assembled on a breadboard. However, due to frequent cable disconnections, a more permanent solution was implemented by soldering components onto a solderable prototype board using JST connectors for interfacing.
| ![image général](https://github.com/user-attachments/assets/65dd4d9f-391e-4781-825a-f37d5b95f575) | ![closup jst](https://github.com/user-attachments/assets/1d5b700b-ae56-44e2-89a7-f66a063a41c2) | ![soudure](https://github.com/user-attachments/assets/151a79e5-68ec-46b8-9d7c-f06d943785f8) |
|---|---|---|
| _General view of the PCB_	| _Close-up of JST connectors_	| _Soldering process_|
## Software Components
### Overview

RefillIt consists of two primary software components:

1) **Firmware for ESP32**:
  - **Platform**: Arduino (via PlatformIO on VSCode)
  - **Languages**: C/C++
  - **Structure**: Modularized code with separate files handling Bluetooth, RFID, Display, LED control, and inputs.
2) **Desktop Application**:
  - **Platform**: Python
  - **Languages**: Python 3.x
  - **Functionality**: Handles serial communication with the ESP32, manages a local JSON database for RFID tags.
### Arduino Code Structure
**Main File**: ```main.cpp```

**Modules**:
- ```Config.h```: Configuration and pin definitions.
- ```BluetoothHandler.h/cpp```: Manages Bluetooth communication.
- ```RFIDHandler.h/cpp```: Handles RFID scanning.
- ```DisplayHandler.h/cpp```: Manages the TFT display.
- ```InputHandler.h/cpp```: Processes inputs from buttons and potentiometer.
- ```LEDHandler.h/cpp```: Controls RGB LEDs.
- ```Utils.h/cpp```: Utility functions.
  
**Libraries Used**:
- Wire.h
- BluetoothSerial.h
- SPI.h
- MFRC522.h
- TFT_eSPI.h
- SerialANSI.h ([Custom Library](https://github.com/py-Alexis/SerialANSI))
