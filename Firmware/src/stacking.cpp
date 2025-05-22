#include "Config.h"
#include "stacking.h"
#include "servoHandler.h"  // Assuming servoHandler is where servo control is managed
#include "pumpHandler.h"   // Assuming pumpHandler is where pump control is managed
#include "motorHandler.h"
#include "debug.h"


int AXE_Z_DEFAULT = -700;
int AXE_Z_LOADED = 0;
int AXE_Z_SUCTION = -1000;
int AXE_Z_STACKED = 2800;
int AXE_Z_DROP = 2600;

int ANGLE_X_DEPLOIED = 60;
int ANGLE_X_RETRACTED = 60;

void loadPlanks() {
    SerialTitle("Starting loading sequence");
    // Step 1/2: Axe Z UP
    SerialLog("Step 1/2: Moving Axe Z UP");
    // Your code to move Axe Z up
    moveAxeZ(AXE_Z_LOADED, 1);
    SerialLog("Axe Z moved UP");

    // Step 2/2: Engage all magnets (servo 1 to 4 at 90 degrees)
    SerialLog("Step 2/2: Engaging all magnets (servo 1 to 4 to 90 degrees)");
    setServo(0, 90);
    setServo(1, 90);
    setServo(2, 90);
    setServo(3, 90);
    SerialLog("Magnets engaged");
}

void stack() {
    SerialTitle("Starting stacking sequence");
    // Step 1/12: Disengage the magnets (servo 1 to 4 to 0 degrees)
    SerialLog("Step 1/12: Disengaging magnets (servo 1 to 4 to 0 degrees)");
    setServo(0, 0);
    setServo(1, 0);
    setServo(2, 0);
    setServo(3, 0);
    SerialLog("Magnets disengaged");

    // Step 2/12: Activate the pump
    SerialLog("Step 2/12: Activating pump");
    pumpActivate();

    // Step 3/12: Axe Z DOWN
    SerialLog("Step 3/12: Moving Axe Z DOWN");
    // Your code to move Axe Z down
    moveAxeZ(AXE_Z_SUCTION, 1);
    SerialLog("Axe Z moved DOWN");

    // Step 4/12: Axe Z UP
    SerialLog("Step 4/12: Moving Axe Z UP");
    // Your code to move Axe Z up
    moveAxeZ(AXE_Z_DEFAULT, 1);
    SerialLog("Axe Z moved UP");

    // Step 5/12: Middle 2 magnets (servo 3 and 4)
    SerialLog("Step 5/12: Engaging middle magnets (servo 3 and 4 to 90 degrees)");
    setServo(2, 90);
    setServo(3, 90);
    SerialLog("Middle magnets engaged");

    // Step 6/12: Axe X retreat (servo 5 to 160 degrees)
    SerialLog("Step 6/12: Retreating Axe X (servo 5 to 160 degrees)");
    setServo(4, 160);
    delay(1000);
    SerialLog("Axe X retreated");

    // Step 7/12: Axe Z UP
    SerialLog("Step 7/12: Moving Axe Z UP");
    // Your code to move Axe Z up
    moveAxeZ(AXE_Z_STACKED, 1);
    SerialLog("Axe Z moved UP");

    // Step 8/12: Axe X extend (servo 5 to 90 degrees)
    SerialLog("Step 8/12: Extending Axe X (servo 5 to 90 degrees)");
    delay(500);
    setServo(4, 90);
    delay(1000);
    SerialLog("Axe X extended");

    // Step 9/12: Disengage middle magnets (servo 3 and 4 to 0 degrees)
    SerialLog("Step 9/12: Disengaging middle magnets (servo 3 and 4 to 0 degrees)");
    setServo(2, 0);
    setServo(3, 0);
    SerialLog("Middle magnets disengaged");

    // Step 10/12: Axe Z DOWN
    SerialLog("Step 10/12: Moving Axe Z DOWN");
    // Your code to move Axe Z down
    moveAxeZ(AXE_Z_DROP, 1);
    SerialLog("Axe Z moved DOWN");

    // Step 11/12: Deactivate pump
    SerialLog("Step 11/12: Deactivating pump");
    pumpDeactivate();

    // Step 12/12: Axe X retreat (servo 5 to 160 degrees)
    SerialLog("Step 12/12: Retreating Axe X (servo 5 to 160 degrees)");
    setServo(4, 130);
    SerialLog("Axe X retreated");

    // Reset the position
    // resetStackingPosition();
}

void resetStackingPosition() {
    SerialTitle("Starting reset sequence");
    // Step 1/3: Retreat Axe X (servo 5 to 160 degrees)
    SerialLog("Step 1/3: Retreating Axe X (servo 5 to 160 degrees)");
    setServo(4, 125);
    delay(1000);
    SerialLog("Axe X retreated");

    // Step 2/3: Move Axe Z DOWN
    SerialLog("Step 2/3: Moving Axe Z DOWN");
    // Your code to move Axe Z down
    moveAxeZ(AXE_Z_DEFAULT, 1);
    SerialLog("Axe Z moved DOWN");

    // Step 3/3: Extend Axe X (servo 5 to 90 degrees)
    SerialLog("Step 3/3: Extending Axe X (servo 5 to 90 degrees)");
    setServo(4, 90);
    SerialLog("Axe X extended");
}

void testAll() {
    switch (testCurrentStep) {
        case 0:
            SerialInfo("Emergency stop removed");
            SerialLog("Step 0: Emergency stop removed");
            delay(2000);
            testCurrentStep++;
            break;

        case 1:
            SerialInfo("com valid");
            SerialLog("Step 1: Communication with ESP32 verified");
            delay(2000);
            testCurrentStep++;
            break;

        case 2:
            SerialInfo("Starter ?");
            SerialLog("Step 2: Awaiting starter initialization");
            testCurrentStep++;
            // Wait for operator to confirm
            break;

        case 3:
            SerialInfo("Starter initialized");
            SerialLog("Step 3: Starter initialized");
            delay(2000);
            testCurrentStep++;
            break;

        case 4:
            SerialInfo("Left wheel?");
            SerialLog("Step 4: Test left wheel (fwd & bwd)");
            // Add your code to test the left wheel

            delay(2000);
            testCurrentStep++;
            break;

        case 5:
            SerialInfo("Right wheel?");
            SerialLog("Step 5: Test right wheel (fwd & bwd)");
            // Add your code to test the right wheel
            delay(2000);
            testCurrentStep++;
            break;

        case 6:
            SerialInfo("Magnet left?");
            SerialLog("Step 6: Test left magnet");
            // Add code to test left magnet (servo 0)
            delay(2000);
            testCurrentStep++;
            break;

        case 7:
            SerialInfo("Magnet middle left?");
            SerialLog("Step 7: Test middle left magnet");
            // Add code to test middle left magnet (servo 1)
            delay(2000);
            testCurrentStep++;
            break;

        case 8:
            SerialInfo("Magnet middle right?");
            SerialLog("Step 8: Test middle right magnet");
            // Add code to test middle right magnet (servo 2)
            delay(2000);
            testCurrentStep++;
            break;

        case 9:
            SerialInfo("Magnet right?");
            SerialLog("Step 9: Test right magnet");
            // Add code to test right magnet (servo 3)
            delay(2000);
            testCurrentStep++;
            break;

        case 10:
            SerialInfo("Axe X?");
            SerialLog("Step 10: Testing Axe X full range");
            // Test Axe X full movement (servo 5)
            setServo(4, 160);
            delay(1000);
            setServo(4, 90);
            delay(1000);
            testCurrentStep++;
            break;

        case 11:
            SerialInfo("Axe Z?");
            SerialLog("Step 11: Testing Axe Z full range");
            moveAxeZ(AXE_Z_SUCTION, 1);
            delay(1000);
            moveAxeZ(AXE_Z_STACKED, 1);
            delay(1000);
            testCurrentStep++;
            break;

        case 12:
            SerialInfo("Suction cup?");
            SerialLog("Step 12: Testing suction cup");
            pumpActivate();
            delay(2000);
            pumpDeactivate();
            testCurrentStep++;
            break;

        case 13:
            SerialInfo("Select team?");
            SerialLog("Step 13: Awaiting team selection");
            // Wait for operator to select team (button or input)
            delay(2000);
            testCurrentStep++;
            break;

        case 14:
            // Team selected elsewhere in code
            // if (selectedTeam == TEAM_BLUE) {
            //     SerialInfo("Team blue selected");
            //     SerialLog("Step 14: Team blue selected");
            // } else {
            //     SerialInfo("Team yellow selected");
            //     SerialLog("Step 14: Team yellow selected");
            // }
            // delay(2000);
            // testCurrentStep++;
            delay(2000);
            testCurrentStep++;
            break;

        case 15:
            SerialInfo("Switching to default screen");
            SerialLog("Step 15: Test completed");
            delay(2000);
            testCurrentStep = 0;
            isTestInProgress = false;
            break;
    }
}