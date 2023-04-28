#include <AccelStepper.h>
#include <MultiStepper.h>
#include "enums.h"

#define motor1_direction 26
#define motor1_step 5
// motor2
// #define motor2_direction 24
// #define motor2_step 25
// // motor3
// #define motor3_direction 26
// #define motor3_step 27
// // motor4
// #define motor4_direction 28
// #define motor4_step 29
AccelStepper motor1(AccelStepper::DRIVER, motor1_step, motor1_direction);
// AccelStepper motor2(AccelStepper::DRIVER, motor2_step, motor2_direction);
// AccelStepper motor3(AccelStepper::DRIVER, motor3_step, motor3_direction);
// AccelStepper motor4(AccelStepper::DRIVER, motor4_step, motor4_direction);

float ROBOTDIAMETER = 30.0;                             // cm
#define stepsPerRevolution 200.0                        // in steps
const float wheelDiameters = 10.0;                      // cm
const float wheelCircumference = (PI * wheelDiameters); // in cm
const float STEPSPERCM = (wheelCircumference / stepsPerRevolution);

long position[4] = {0, 0, 0, 0};

MultiStepper steppers;
const float MAX_SPEED = 3000.0;
const float ACCELERATION = 1000.0;

void setup()
{
    motor1.setMaxSpeed(MAX_SPEED);
    motor1.setAcceleration(ACCELERATION);
    // motor1.runToNewPosition(1000);
    // motor2.setMaxSpeed(MAX_SPEED);
    // motor2.setAcceleration(ACCELERATION);
    // motor3.setMaxSpeed(MAX_SPEED);
    // motor3.setAcceleration(ACCELERATION);
    // motor4.setMaxSpeed(MAX_SPEED);
    // motor4.setAcceleration(ACCELERATION);

    // Then give them to MultiStepper to manage
    steppers.addStepper(motor1);
    // steppers.addStepper(motor2);
    // steppers.addStepper(motor3);
    // steppers.addStepper(motor4);
}
int pos = 10000;
void loop()
{
    // if (motor1.distanceToGo() == 0)
    // {
    //     delay(500);
    //     pos = -pos;
    //     motor1.moveTo(pos);
    // }
    // // motor1.moveTo(pos);
    // motor1.run();
    digitalWrite(5, HIGH);
    delayMicroseconds(400);
    digitalWrite(5, LOW);
    delayMicroseconds(400);
    // moveInDirection(north, 100);
    // moveInDirection(east, 100);
    // moveInDirection(south, 100);
    // moveInDirection(west, 100);
    // moveInDirection(forward, 100);
    // moveInDirection(backward, 100);
    // moveInDirection(left, 100);
    // moveInDirection(right, 100);
    // moveInDirection(0, 100);
    // moveInDirection(90, 100);
    // moveInDirection(180, 100);
    // moveInDirection(270, 100);
    // moveInDirection(45, 100);
    // moveInDirection(135, 100);
    // moveInDirection(225, 100);
    // moveInDirection(315, 100);
    // turnInDirection(east);
    // turnInDirection(south);
    // turnInDirection(west);
    // turnInDirection(north);
    // turnToDirection(90);
    // turnToDirection(180);
    // turnToDirection(270);
    // turnToDirection(0);
}

void moveInDirection(absolute_direction dir, int distance)
{
    // get the direction of the robot
    int angleOfRobot = getMagData();
    // convert to absolute direction to degrees from north
    float direction;
    if (dir == north)
    {
        direction = 0.0;
    }
    else if (dir == east)
    {
        direction = 90.0;
    }
    else if (dir == south)
    {
        direction = 180.0;
    }
    else if (dir == west)
    {
        direction = 270.0;
    }
    // float direction = 0.0;

    float directionWheel1 = angleOfRobot + 45.0;
    float directionWheel2 = angleOfRobot + 135.0;
    float weight1 = cos((direction - directionWheel1 + 90.0) * 1000 / 57296);
    float weight2 = cos((direction - directionWheel2 + 90.0) * 1000 / 57296);

    int stepsWheel1 = round(weight1 * distance);
    int stepsWheel2 = round(weight2 * distance);
    int stepsWheel3 = -stepsWheel1;
    int stepsWheel4 = -stepsWheel2;

    // change their goals
    position[0] += stepsWheel1;
    position[1] += stepsWheel2;
    position[2] += stepsWheel3;
    position[3] += stepsWheel4;

    // now we have to use the multistepper to move all the steps at once
    steppers.moveTo(position);
    steppers.runSpeedToPosition();
}

void moveInDirection(relative_direction dir, int distance)
{
    // get the direction of the robot
    int angleOfRobot = getMagData();
    // convert to absolute direction to degrees from north
    float direction;
    if (dir == forward)
    {
        direction = angleOfRobot;
    }
    else if (dir == right)
    {
        direction = angleOfRobot + 90.0;
    }
    else if (dir == backward)
    {
        direction = angleOfRobot + 180.0;
    }
    else if (dir == left)
    {
        direction = angleOfRobot + 270.0;
    }
    // float direction = 0.0;

    float directionWheel1 = angleOfRobot + 45.0;
    float directionWheel2 = angleOfRobot + 135.0;
    float weight1 = cos((direction - directionWheel1 + 90.0) * 1000 / 57296);
    float weight2 = cos((direction - directionWheel2 + 90.0) * 1000 / 57296);

    int stepsWheel1 = round(weight1 * distance);
    int stepsWheel2 = round(weight2 * distance);
    int stepsWheel3 = -stepsWheel1;
    int stepsWheel4 = -stepsWheel2;

    // change their goals
    position[0] += stepsWheel1;
    position[1] += stepsWheel2;
    position[2] += stepsWheel3;
    position[3] += stepsWheel4;

    // now we have to use the multistepper to move all the steps at once
    steppers.moveTo(position);
    steppers.runSpeedToPosition();
}

void moveInDirection(float degrees, int distance)
{
    float angleOfRobot = getMagData();
    float direction = angleOfRobot + degrees;
    float directionWheel1 = angleOfRobot + 45.0;
    float directionWheel2 = angleOfRobot + 135.0;
    float weight1 = cos((direction - directionWheel1 + 90.0) * 1000 / 57296);
    float weight2 = cos((direction - directionWheel2 + 90.0) * 1000 / 57296);

    int stepsWheel1 = round(weight1 * distance);
    int stepsWheel2 = round(weight2 * distance);
    int stepsWheel3 = -stepsWheel1;
    int stepsWheel4 = -stepsWheel2;

    // change their goals
    position[0] += stepsWheel1;
    position[1] += stepsWheel2;
    position[2] += stepsWheel3;
    position[3] += stepsWheel4;

    // now we have to use the multistepper to move all the steps at once
    steppers.moveTo(position);
    steppers.runSpeedToPosition();
}

// relative to the robot direction
void turnInDirection(float degrees)
{
    // float angleOfRobot = getGyroData();
    // convert to absolute direction to radians
    degrees = degrees * 1000 / 57296;
    // get the distance the wheels need to travel
    float distance = ROBOTDIAMETER / 2 * degrees;
    // now get the amount of steps
    int steps = distance * STEPSPERCM;
    // change their goals
    position[0] += steps;
    position[1] += steps;
    position[2] += steps;
    position[3] += steps;

    // now we have to use the multistepper to move all the steps at once in the same direction
    steppers.moveTo(position);
    steppers.runSpeedToPosition();
}

// absolute direction ex: 0 = north, 180 = south
void turnToDirection(float dir)
{
    float angleOfRobot = getMagData();
    float degrees = dir - angleOfRobot;
    turnInDirection(degrees);
}

void turnInDirection(absolute_direction dir)
{
    float angleOfRobot = getMagData();
    float degrees;
    if (dir == north)
    {
        degrees = 0.0;
    }
    else if (dir == east)
    {
        degrees = 90.0;
    }
    else if (dir == south)
    {
        degrees = 180.0;
    }
    else if (dir == west)
    {
        degrees = 270.0;
    }
    turnToDirection(degrees);
}

float getMagData()
{
    return 0;
}