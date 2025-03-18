#include <Arduino.h>
#include "config.h"
#include "robot.h"
#include "motorHandler.h"
#include "point.h"
#include "displayHandler.h"
#include "powerSensorHandler.h"

#include "Wire.h"
#include <math.h>
TwoWire myWire(0);
TwoWire wireDisplay(1);

using namespace ecn;

Robot robot(0, 0, 0, 3, 5, 0.01, 0.1); // x, y, theta, speed, kp, ki, kd

DisplayHandler display;

// // Robot state (in meters, radians)
float currentX = 0.0;
float currentY = 0.0;
float currentTheta = 0.0;

// Timing
unsigned long lastUpdateTime = 0;
unsigned long startTime = 0;

const int internalLed = 2; // eingebaute LED auf GPIO 2

int current_goal_index = 1;
float distance_to_goal = 0.0;

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

        robot.goals.push_back(Pose(x, y, theta));
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

void parsePath(String data)
{
    robot.path_.clear();
    int lastIndex = 0;
    while (lastIndex < data.length())
    {
        int nextIndex = data.indexOf(';', lastIndex);
        if (nextIndex == -1)
            nextIndex = data.length();

        String segment = data.substring(lastIndex, nextIndex);
        lastIndex = nextIndex + 1;

        int comma = segment.indexOf(',');
        if (comma == -1)
            continue;

        float x = segment.substring(0, comma).toFloat();
        float y = segment.substring(comma + 1).toFloat();
        // Serial.printf("Parsedpath Point: (%f, %f)\n", x, y);
        robot.path_.push_back(Point(x, y));
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
    if (command.startsWith("GOALS:"))
    {
        parseGoals(command.substring(6));
        Serial.println("ACK:GOALS_RECEIVED");
    }
    else if (command.startsWith("PATH:"))
    {
        parsePath(command.substring(5));
        Serial.println("ACK:PATH_RECEIVED");
    }
    else if (command.startsWith("STATE:"))
    {
        String stateStr = command.substring(6);
        if (stateStr == "INIT")
            state = INIT;
        // else if (stateStr == "NAVIGATION")
        //     state = NAVIGATION;
        // else if (stateStr == "TURN_TO_GOAL")
        //     state = TURN_TO_GOAL;
        // else if (stateStr == "TURN_TO_PATH")
        //     state = TURN_TO_PATH;
        // else if (stateStr == "GOAL_REACHED")
        //     state = GOAL_REACHED;
        Serial.print("ACK:STATE_RECEIVED: ");
        Serial.println(stateStr);
    }
    else if (command == "RESET")
    {
        Serial.println("ESP32 restarting...");
        ESP.restart(); // Software Reset
    }

    else if (command.startsWith("SPEED_PID:"))
    {
        parseSpeedAndPID(command.substring(10));
    }
}

void sendPositionUpdate()
{
    if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE)
    {
        // currentX = robot.getX();
        // currentY = robot.getY();
        // currentTheta = robot.getTheta();
        robot.setPose(_robotX, _robotY, _robotTheta);
        xSemaphoreGive(robotXMutex);

        digitalWrite(internalLed, HIGH);
        Serial.print("X: ");
        Serial.print(robot.getX());
        Serial.print(", Y: ");
        Serial.print(robot.getY());
        Serial.print(", Theta: ");
        Serial.println(robot.getTheta());
        digitalWrite(internalLed, LOW);

        // display.updatePointsDisplay(robotY);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    delay(500);

    // Initialize the other modules (motors, display, power sensor)
    initMotor();

    // For testing, we start with a low constant speed; pure pursuit will adjust individual wheel speeds.
    setMotorSpeeds(0, 0);
    display.initDisplay(false, false);
    initPowerSensor();

    pinMode(internalLed, OUTPUT);
    digitalWrite(internalLed, LOW);

    // Reset timer for control loop
    lastUpdateTime = millis();
    startTime = millis();

    Serial.println("ESP Initialized!");
}

RobotState lastState = GOAL_REACHED; // Letzter ausgegebener Zustand

void loop()
{
    if (millis() - lastUpdateTime >= robot.getDt())
    {
        lastUpdateTime = millis();

        while (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            processCommand(command);
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

            Serial.println("CurrentState: WAIT");

            break;
        case INIT:
            // if (state != lastState)
            // {
            Serial.println("CurrentState: INIT");
            //     lastState = state;
            // }

            Serial.printf("Current Goal: %f, %f, %f\n", robot.goals[current_goal_index].point.x, robot.goals[current_goal_index].point.y, robot.goals[current_goal_index].theta);

            state = TURN_TO_PATH;
            [[fallthrough]];

        case TURN_TO_PATH:

            // if (state != lastState)
            // {
            Serial.println("CurrentState: TURN_TO_PATH");

            //     lastState = state;
            // }

            if (robot.turnToPathOrientation())
            {
                Serial.println("Robot is aligned to path orientation!");
                robot.setIsStarting(true);
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
            distance_to_goal = robot.distanceToGoal(robot.goals[current_goal_index].point);
            if (distance_to_goal < 5.0)
            {
                std::cout << "Distance to goal: " << distance_to_goal << std::endl;

                if (distance_to_goal < 0.5)
                {
                    std::cout << "Robot has reached the goal -> TURN_TO_GOAL!" << std::endl;
                    state = TURN_TO_GOAL;
                }
            }

            break;

        case TURN_TO_GOAL:
            Serial.println("CurrentState: TURN_TO_GOAL");

            if (robot.turnToGoalOrientation())
            {
                std::cout << "Robot is aligned to goal orientation!" << std::endl;
                state = GOAL_REACHED;
            }

            break;

        case GOAL_REACHED:

            Serial.println("CurrentState: GOAL_REACHED");

            if (current_goal_index++ < robot.goals.size())
            {
                current_goal_index++;
                robot.setTargetTheta(robot.goals[current_goal_index].theta);
                state = INIT;
            }
            else
            {
                std::cout << "All goals have been reached. Mission complete!" << std::endl;
                // End simulation
            }
            break;
        }

        if (state != WAIT)
        {
            setMotorSpeeds(robot.getLeftSpeed(), robot.getRightSpeed());
            // Serial.printf("LM Speed: %f, RM Speed: %f\n", robot.getLeftSpeed(), robot.getRightSpeed());

            if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE)
            {
                robot.setPose(_robotX, _robotY, _robotTheta);
                xSemaphoreGive(robotXMutex);

                digitalWrite(internalLed, HIGH);
                Serial.print("X: ");
                Serial.print(robot.getX());
                Serial.print(", Y: ");
                Serial.print(robot.getY());
                Serial.print(", Theta: ");
                Serial.println(robot.getTheta());
                digitalWrite(internalLed, LOW);

                // display.updatePointsDisplay(robotY);
            }

            // robot.setPose(_robotX, _robotY, _robotTheta);
            // sendPositionUpdate();
        }

        display.updatePointsDisplay(0);

        // End of loop
    }
}
