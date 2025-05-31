#include <Arduino.h>
#include "config.h"
#include "robot.h"
#include "motorHandler.h"
#include "point.h"
// #include "displayHandler.h"
#include "powerSensorHandler.h"
#include "controlPanelHandler.h"
#include "debug.h"
#include "stacking.h"
#include "servoHandler.h"

#include <chrono>

#include "Wire.h"
#include <math.h>

SemaphoreHandle_t i2cMutex = xSemaphoreCreateMutex();

TwoWire myWire(0);
TwoWire wireDisplay(1);

using namespace ecn;

Robot robot(0, 0, 0, 3, 5, 0.01, 0.1); // x, y, theta, speed, kp, ki, kd

// DisplayHandler display;

// // Robot state (in meters, radians)
float currentX = 0.0;
float currentY = 0.0;
float currentTheta = 0.0;

// Timing
unsigned long lastUpdateTime = 0;
unsigned long startTime = 0;

// const int internalLed = 2; // eingebaute LED auf GPIO 2

float distance_to_goal = 0.0;

bool emergencystop = false;
int emergency_counter = 0;

bool start_timer = false;
auto start_time = std::chrono::steady_clock::now();

enum RobotState
{
    WAIT,
    INIT,
    NAVIGATION,
    TURN_TO_GOAL,
    TURN_TO_PATH,
    GOAL_REACHED,
    PATH_PLANNING
};
RobotState state = WAIT;
RobotState last_sent_state = WAIT;

String pathBuffer = ""; // Buffer to store the incoming path data

float distance_to_travel_4 = -27.0;

bool continue_after_straight = false;

int straight_counter = 0;
int go_back_counter_limit = 20;

float max_straight_speed = 10.0; // Maximum speed for straight movement
float straight_speed = 0.0; // Current speed for straight movement

    void
    sendpath(const std::vector<Point> &path)
{
    Serial.print("PATH:");
    for (size_t i = 0; i < path.size(); ++i)
    {
        Serial.printf("%f,%f", path[i].x, path[i].y);
        if (i < path.size() - 1)
        {
            Serial.print(";"); // Separate points with semicolons
        }
    }
    Serial.print("\n"); // End the path message
}

void parseGoals(String data)
{
    robot.goals.clear();
    int lastIndex = 0;
    while (lastIndex < data.length())
    {
        int nextIndex = data.indexOf(';', lastIndex);
        if (nextIndex == -1)
            nextIndex = data.length();

        String segment = data.substring(lastIndex, nextIndex);
        lastIndex = nextIndex + 1;

        int comma1 = segment.indexOf(',');
        int comma2 = segment.lastIndexOf(',');

        if (comma1 == -1 || comma2 == -1)
            continue;

        float x = segment.substring(0, comma1).toFloat();
        float y = segment.substring(comma1 + 1, comma2).toFloat();
        float theta = segment.substring(comma2 + 1).toFloat();

        robot.goals.push_back(Pose(x, y, theta * M_PI / 180));
    }

    robot.setPose(robot.goals[0].point.x, robot.goals[0].point.y, robot.goals[0].theta);

    // Initialize the robot's position with the first goal's coordinates
    if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE)
    {
        _robotX = robot.goals[0].point.x;
        _robotY = robot.goals[0].point.y;
        _robotTheta = robot.goals[0].theta;
        xSemaphoreGive(robotXMutex);
    }

    robot.setTargetTheta(robot.goals[0].theta);

    Serial.print("Goals received: ");
    for (const auto &goal : robot.goals)
    {
        Serial.printf("(%f, %f, %f) ", goal.point.x, goal.point.y, goal.theta);
    }
    Serial.println();
}

// void parsePath(String data)
// {
//     robot.path_.clear();
//     int lastIndex = 0;
//     while (lastIndex < data.length())
//     {
//         int nextIndex = data.indexOf(';', lastIndex);
//         if (nextIndex == -1)
//             nextIndex = data.length();

//         String segment = data.substring(lastIndex, nextIndex);
//         lastIndex = nextIndex + 1;

//         int comma = segment.indexOf(',');
//         if (comma == -1)
//             continue;

//         float x = segment.substring(0, comma).toFloat();
//         float y = segment.substring(comma + 1).toFloat();
//         // Serial.printf("Parsedpath Point: (%f, %f)\n", x, y);
//         robot.path_.push_back(Point(x, y));
//     }
//     // Serial.print("Full Path received: ");
//     // for (const auto &point : robot.path_)
//     // {
//     //     Serial.printf("(%f, %f) ", point.x, point.y);
//     // }
//     // Serial.println();
//     Serial.print("Number of points in the path::: ");
//     Serial.println(robot.path_.size());
// }


void moveStraightsimple(bool forward){
    if (straight_counter < max_straight_speed)
    {
        if(forward)
            straight_speed += 1;
        else
            straight_speed -= 1;

        robot.setWheelSpeed(straight_speed, straight_speed);

    }
    if (straight_counter > max_straight_speed && straight_counter < (go_back_counter_limit - max_straight_speed))
    {
        if(forward)
            robot.setWheelSpeed(max_straight_speed, max_straight_speed);
        else
            robot.setWheelSpeed(-max_straight_speed, -max_straight_speed);
    }
    if (straight_counter >= (go_back_counter_limit - max_straight_speed) && straight_counter < go_back_counter_limit)
    {
        if (forward)
            straight_speed -= 1;
        else
            straight_speed += 1;

        robot.setWheelSpeed(straight_speed, straight_speed);
    }

    straight_counter++;

    if (straight_counter >= go_back_counter_limit)
    {
        straight_speed = 0;
        robot.setWheelSpeed(straight_speed, straight_speed);
        straight_counter = 0;
        continue_after_straight = true;
    }
}

void parseSpeedAndPID(String data)
{
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    int thirdComma = data.indexOf(',', secondComma + 1);

    if (firstComma == -1 || secondComma == -1 || thirdComma == -1)
        return;

    float speed = data.substring(0, firstComma).toFloat();
    float P = data.substring(firstComma + 1, secondComma).toFloat();
    float I = data.substring(secondComma + 1, thirdComma).toFloat();
    float D = data.substring(thirdComma + 1).toFloat();

    robot.setSpeed(speed);
    robot.setPID(P, I, D);

    Serial.print("Received Speed and PID: ");
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(", P: ");
    Serial.print(P);
    Serial.print(", I: ");
    Serial.print(I);
    Serial.print(", D: ");
    Serial.println(D);
}

void processCommand(String command)
{
    // Serial.println("RCCCC: " + command);

    if (command.startsWith("GOALS:"))
    {
        parseGoals(command.substring(6));
        Serial.println("ACK:GOALS_RECEIVED");

        state = INIT;
    }

    // else if (command.startsWith("PATH:"))
    // {
    //     pathBuffer += command.substring(5);
    // }

    // else if (command.startsWith("PT:")){
    //     pathBuffer += command.substring(3);
    // }

    // else if (command.startsWith("PATHEND:"))
    // {
    //     pathBuffer += command.substring(8);
    //     parsePath(pathBuffer);
    //     pathBuffer = "";
    //     Serial.println("ACK:PATH_RECEIVED");

    // }

    else if (command.startsWith("STATE:"))
    {
        // String stateStr = command.substring(6);
        // // if (stateStr == "INIT")
        // //     state = INIT;
        // // else if (stateStr == "NAVIGATION")
        // //     state = NAVIGATION;
        // // else if (stateStr == "TURN_TO_GOAL")
        // //     state = TURN_TO_GOAL;
        // // else if (stateStr == "TURN_TO_PATH")
        // //     state = TURN_TO_PATH;
        // // else if (stateStr == "GOAL_REACHED")
        // //     state = GOAL_REACHED;
        // Serial.print("ACK:STATE_RECEIVED:_");
        // Serial.print(stateStr);
        // Serial.println("_");
    }

    if (command == "EMERGENCYSTOP")
    {
        emergencystop = true;
        Serial.println("EMERGENCYSTOP_RECEIVED");
    }
    // else if (command == "RESET")
    // {
    //     Serial.println("ESP32 restarting...");
    //     ESP.restart(); // Software Reset
    // }

    else if (command.startsWith("SPEED_PID:"))
    {
        parseSpeedAndPID(command.substring(10));
    }
}

void sendPositionUpdate()
{
    if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE)
    {
        robot.setPose(_robotX, _robotY, _robotTheta);
        xSemaphoreGive(robotXMutex);

        // digitalWrite(internalLed, HIGH);
        Serial.print("X: ");
        Serial.print(robot.getX());
        Serial.print(", Y: ");
        Serial.print(robot.getY());
        Serial.print(", Theta: ");
        Serial.println(robot.getTheta());
        // digitalWrite(internalLed, LOW);
    }
}

void setup()
{
    Serial.begin(921600);
    while (!Serial)
    {
        delay(10);
    }
    delay(200);

    Serial.println("");
    Serial.print("RESET!\n");
    Serial.println("");

    SerialTitle("Initialisation du système");

    SerialInfo("1/6 Initialisation de la pompe...");
    // initPump();

    SerialInfo("2/6 Initialisation du servo...");
    initServo();

    SerialInfo("3/6 Initialisation du capteur de puissance...");
    // initPowerSensor();

    SerialInfo("4/6 Initialisation du moteur...");
    initMotor();

    SerialInfo("5/6 Initialisation de l'afficheur...");
    // display.initDisplay(true, false);

    SerialInfo("6/6 Initialisation de l'afficheur...");
    // initControlPanel();

    SerialInfo("Initialisation terminée avec succès !");

    setMotorSpeeds(0, 0);

    setServo(0, 0);
    vTaskDelay(200); // Hold position
    setServo(1, 0);
    vTaskDelay(200); // Hold position
    setServo(2, 0);
    vTaskDelay(200); // Hold position
    setServo(8, 0);

    vTaskDelay(600); // Hold position

    setServo(0, 90);
    vTaskDelay(200); // Hold position
    setServo(1, 90);
    vTaskDelay(200); // Hold position
    setServo(2, 90);
    vTaskDelay(200); // Hold position
    setServo(8, 90);

    vTaskDelay(600); // Hold position

    setServo(0, 0);
    vTaskDelay(200); // Hold position
    setServo(1, 0);
    vTaskDelay(200); // Hold position
    setServo(2, 0);
    vTaskDelay(200); // Hold position
    setServo(8, 0);

    // pinMode(internalLed, OUTPUT);
    // digitalWrite(internalLed, LOW);

    // Reset timer for control loop
    lastUpdateTime = millis();
    startTime = millis();

    // testServo();

    pinMode(starter_button, INPUT_PULLDOWN);
    pinMode(colour_button, INPUT_PULLDOWN);
    pinMode(save_start_button, INPUT_PULLDOWN);

    // while(true)
    //  {
    //     Serial.print(digitalRead(starter_button));
    //     Serial.print(", ");
    //     Serial.print(digitalRead(colour_button));
    //     Serial.print(", ");
    //     Serial.println(digitalRead(save_start_button));
    //  }

    while (digitalRead(save_start_button) == LOW)
    {
        Serial.print("Waiting for Save Start button...\n");
        vTaskDelay(200);
    }

    Serial.println("security button pressed, waiting for Start button...");

    while (digitalRead(starter_button) == HIGH)
    {
        Serial.println("Waiting for Start button to be pressed...");
        vTaskDelay(200);
    }

    Serial.println("ESP Initialized!");

    Serial.print("START!\n");

    if (digitalRead(colour_button) == LOW)
    {
        Serial.print("COLOUR:yellow\n");
    }
    else
    {
        Serial.print("COLOUR:blue\n");
    }
}

RobotState lastState = GOAL_REACHED; // Letzter ausgegebener Zustand

void loop()
{
    if (millis() - lastUpdateTime >= robot.getDtInMs())
    {
        lastUpdateTime = millis();

        if (emergencystop == true)
        {
            emergency_counter++;
            if (emergency_counter > 65)
            {
                emergencystop = false;
                emergency_counter = 0;
            }
        }

        while (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            processCommand(command);

            // // If a PATH message is received, wait until PATHEND message
            // if (command.startsWith("PATH:"))
            // {
            //     // Collect all parts of the path until we encounter PATHEND
            //     String fullPath = ""; // Initialize empty string to store the full path

            //     if (command.startsWith("PATH:"))
            //     {
            //         command = command.substring(5); // Remove "PATH:" prefix
            //     }

            //     fullPath += command;
            //     int counter = 0;
            //     // Keep collecting until "PATHEND:" is found
            //     while (true)
            //     {
            //         if (Serial.available())
            //         {
            //             command = Serial.readStringUntil('\n');
            //             // Serial.println("RCCCC: " + command);

            //             // counter++;
            //             // Serial.print("Received Path Count: ");
            //             // Serial.println(counter);

            //             // If we encounter "PATHEND:", we need to remove it before adding to fullPath
            //             if (command.startsWith("PATHEND:"))
            //             {
            //                 command = command.substring(8); // Remove "PATHEND:" prefix
            //                 fullPath += command;            // Add the final chunk to the full path
            //                 parsePath(fullPath);       // Process the complete path
            //                 break;                          // Exit the loop after processing
            //             }
            //             else
            //             {
            //                 // Otherwise, just add the chunk to the full path
            //                 fullPath += command.substring(3);
            //             }
            //         }
            //     }
            // }
        }

        // static String inputString = ""; // Buffer for incoming data

        // while (Serial.available())
        // {
        //     char c = Serial.read();
        //     inputString += c;

        //     if (c == '\n')
        //     { // Full message received
        //         processCommand(inputString);
        //         inputString = ""; // Reset buffer
        //     }
        // }

        switch (state)
        {
        case WAIT:

            while (Serial.available())
            {
                String command = Serial.readStringUntil('\n');
                processCommand(command);
            }

            Serial.print("CurrentState: WAIT\n");

            break;
        case INIT:
            Serial.println("CurrentState: INIT");

            Serial.printf("Current Goal: %f, %f, %f\n", robot.goals[robot.getCurrentGoalindex()].point.x, robot.goals[robot.getCurrentGoalindex()].point.y, robot.goals[robot.getCurrentGoalindex()].theta);

            robot.setIsStarting(true);

            if (start_timer)
            {
                start_time = std::chrono::steady_clock::now();
            }

            robot.generateStraightPath();

            sendpath(robot.getPath());

            state = TURN_TO_PATH;
            [[fallthrough]];

        case TURN_TO_PATH:
            Serial.println("CurrentState: TURN_TO_PATH");

            if (INIT != last_sent_state)
            {
                Serial.print("STATE:INIT\n");
                last_sent_state = INIT;
            }

            if (robot.turnToPathOrientation())
            {
                Serial.println("Robot is aligned to path orientation!");
                robot.setIsStarting(true);
                robot.start_turning = true;
                state = NAVIGATION;
            }

            break;

        case NAVIGATION:
            if (PATH_PLANNING != last_sent_state)
            {
                Serial.print("STATE:PATH_PLANNING\n");
                last_sent_state = PATH_PLANNING;
            }
            Serial.println("CurrentState: NAVIGATION");

            robot.followPath();

            // float start = Point(robot.getX(), robot.getY());
            distance_to_goal = robot.distanceToGoal(robot.goals[robot.getCurrentGoalindex()].point);
            if (distance_to_goal < 5.0)
            {
                // std::cout << "Distance to goal: " << distance_to_goal << std::endl;

                if (distance_to_goal < 0.5)
                {
                    Serial.printf("Goal reached: %f, %f, %f\n", robot.goals[robot.getCurrentGoalindex()].point.x, robot.goals[robot.getCurrentGoalindex()].point.y, robot.goals[robot.getCurrentGoalindex()].theta);
                    std::cout << "Robot has reached the goal -> TURN_TO_GOAL!" << std::endl;
                    state = TURN_TO_GOAL;
                }
            }

            break;

        case TURN_TO_GOAL:
            Serial.println("CurrentState: TURN_TO_GOAL");
            Serial.print("STATE:TURN_TO_GOAL\n");

            if (last_sent_state != WAIT)
            {
                Serial.print("STATE:WAIT\n");
                last_sent_state = WAIT;
                // delay(1000);
            }

            if (robot.turnToGoalOrientation())
            {
                std::cout << "Robot is aligned to goal orientation!" << std::endl;
                robot.start_turning = true;
                state = GOAL_REACHED;
            }

            break;

        case GOAL_REACHED:

            Serial.println("CurrentState: GOAL_REACHED");
            Serial.print("STATE:GOAL_REACHED\n");
            Serial.print("current_goal_index: ");
            Serial.println(robot.getCurrentGoalindex());

            vTaskDelay(200);

            if (robot.getCurrentGoalindex() < robot.goals.size() - 1)
            {

                if (robot.getCurrentGoalindex() == 2)
                {
                    setServo(0, 90);
                    setServo(1, 90);
                    setServo(2, 90);
                    setServo(3, 90);
                }

                if (robot.getCurrentGoalindex() == 4) // without start so real goal
                {
                    setServo(0, 0);
                    setServo(1, 0);
                    setServo(2, 0);
                    setServo(3, 0);

                    moveStraightsimple(forward = false);
                }
                if (robot.getCurrentGoalindex() == 8) // without start so real goal
                {
                    moveStraightsimple(forward = false);
                }
                if (robot.getCurrentGoalindex() == 10)
                {
                    setServo(0, 90);
                    setServo(1, 90);
                    setServo(2, 90);
                    setServo(3, 90);

                    moveStraightsimple(forward = false);
                }

                if (robot.getCurrentGoalindex() == 11) // without start so real goal
                {
                    setServo(0, 0);
                    setServo(1, 0);
                    setServo(2, 0);
                    setServo(3, 0);

                    moveStraightsimple(forward = false);
                }

                else
                {
                    robot.incrementCurrentGoalIndex();
                    Serial.print("New current_goal_index: ");
                    Serial.println(robot.getCurrentGoalindex());
                    Serial.printf("New Goal: %f, %f, %f\n", robot.goals[robot.getCurrentGoalindex()].point.x, robot.goals[robot.getCurrentGoalindex()].point.y, robot.goals[robot.getCurrentGoalindex()].theta);
                    robot.setTargetTheta(robot.goals[robot.getCurrentGoalindex()].theta);
                    std::cout << "Set Target theta to: " << robot.goals[robot.getCurrentGoalindex()].theta << std::endl;

                    state = INIT;
                }

                if (continue_after_straight == true)
                {
                    Serial.println("Robot is aligned to path orientation!");
                    robot.setIsStarting(true);
                    robot.start_turning = true;
                    robot.incrementCurrentGoalIndex();
                    Serial.print("New current_goal_index: ");
                    Serial.println(robot.getCurrentGoalindex());
                    Serial.printf("New Goal: %f, %f, %f\n", robot.goals[robot.getCurrentGoalindex()].point.x, robot.goals[robot.getCurrentGoalindex()].point.y, robot.goals[robot.getCurrentGoalindex()].theta);
                    robot.setTargetTheta(robot.goals[robot.getCurrentGoalindex()].theta);
                    std::cout << "Set Target theta to: " << robot.goals[robot.getCurrentGoalindex()].theta << std::endl;

                    continue_after_straight = false;

                    state = INIT;
                }

                // if(robot.getCurrentGoalindex() == 2){
                //     vTaskDelay(65000);
                // }
            }
            else
            {
                std::cout << "All goals have been reached. Mission complete!" << std::endl;
                state = WAIT;
                // End simulation
            }
            break;
        }

        // Measure execution time
        auto now_time = std::chrono::steady_clock::now();
        double time_duration = std::chrono::duration_cast<std::chrono::seconds>(now_time - start_time).count();

        if (time_duration > 100 && start_timer == true)
        {
            Serial.println("The exec time is over 100 sec");
            robot.stop();
        }

        // static auto last_print_time = std::chrono::steady_clock::now();
        // if (std::chrono::duration_cast<std::chrono::milliseconds>(now_time - last_print_time).count() >= 1000)
        // {
        //     last_print_time = now_time;

        //     Serial.print("ESPstate: ");
        //     Serial.println(state);
        // }

        // if (state == WAIT)
        //     {
        //         if (time_duration > 5 && start_timer == true)
        //         {
        //             Serial.println("The exec time is over 5 sec");
        //             state = INIT;
        //         }
        //     }

        if (emergencystop == true)
        {
            robot.stop();
        }

        if (state != WAIT)
        {
            setMotorSpeeds(robot.getLeftSpeed() / 100, robot.getRightSpeed() / 100);
            Serial.printf("LM Speed: %f, RM Speed: %f\n", robot.getLeftSpeed(), robot.getRightSpeed());

            if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE)
            {
                robot.setPose(_robotX, _robotY, _robotTheta);
                xSemaphoreGive(robotXMutex);

                // digitalWrite(internalLed, HIGH);
                Serial.print("X: ");
                Serial.print(robot.getX());
                Serial.print(", Y: ");
                Serial.print(robot.getY());
                Serial.print(", Theta: ");
                Serial.println(robot.getTheta());
                // digitalWrite(internalLed, LOW);

                // display.updatePointsDisplay(robotY);
            }

            // robot.setPose(_robotX, _robotY, _robotTheta);
            // sendPositionUpdate();
        }

        // display.updatePointsDisplay(0);

        // End of loop
    }
}
