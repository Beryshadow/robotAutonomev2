// magnetometer
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float initialDirection = 0;
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);
const int magAvg = 5;
/*
Skills ontario autonomous robot
court size = 609.6mm x 3733.6mm
robot size = 45cm x 45cm
*/

// #define 

const int WIDTH = 45;
const int HEIGHT = 45;
// steppers
// https://dronebotworkshop.com/big-stepper-motors/
// https://forum.arduino.cc/t/sabertooth-2x12-guidance/661660
#include "enums.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#define stepsPerRevolution 200.0                        // in steps
const float wheelDiameters = 10.0;                      // cm
const float wheelCircumference = (PI * wheelDiameters); // in cm
const float STEPSPERCM = (wheelCircumference / stepsPerRevolution);

// disable stepper
const int enable1 = 8;
const int enable2 = 42;
const int enable3 = 10;
const int enable4 = 11;

// motor1
#define motor1_direction 30
#define motor1_step 2
// motor2
#define motor2_direction 31
#define motor2_step 3
// motor3
#define motor3_direction 32
#define motor3_step 4
// motor4
#define motor4_direction 33
#define motor4_step 5

AccelStepper motor1(AccelStepper::DRIVER, motor1_step, motor1_direction);
AccelStepper motor2(AccelStepper::DRIVER, motor2_step, motor2_direction);
AccelStepper motor3(AccelStepper::DRIVER, motor3_step, motor3_direction);
AccelStepper motor4(AccelStepper::DRIVER, motor4_step, motor4_direction);

long position[4] = {0, 0, 0, 0};

MultiStepper steppers;

const float MAX_SPEED = 2000.0;
const float ACCELERATION = 100.0;

// servo
// https://www.instructables.com/Arduino-Servo-Motors/
#include <Servo.h>
#define servoPin 7
Servo trapDoor;
const int OPEN = 10;
const int CLOSE = 40;

// motor
// https://forum.arduino.cc/t/sabertooth-2x12-guidance/661660
// https://se.inf.ethz.ch/people/wei/robots/arduino_sabertooth2x12/sabertooth.html
#define motorPin 6
Servo pallete; // 45 = forward 90 = stop 135 = backward

// camera
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
#include <Pixy2.h>
Pixy2 pixy;

// buttons
#define button1Pin 28 // side button
#define button2Pin 40 // back button

// misc
float ROBOTDIAMETER = 40.0; // cm

void setup()
{
    /// init everything

    // serial monitor
    Serial.begin(115200);
    // while (!Serial)
    // { // Waiting for USB Serial COM port to open.
    // }

    if (!mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1)
            ;
    }

    // mag init directions
    // sensors_event_t event;
    // mag.getEvent(&event);

    // float Pi = 3.14159;

    // Calculate the angle of the vector y,x

    // camera
    pixy.init();

    // servo
    trapDoor.attach(servoPin);
    trapDoor.write(OPEN);

    // buttons
    // pinMode(button1Pin, INPUT);
    // pinMode(button2Pin, INPUT);

    // motor
    pallete.attach(motorPin);
    pallete.write(90);
    // 30 = forward
    // 90 = stop
    // 160 = backward

    // steppers
    motor1.setMaxSpeed(MAX_SPEED);
    motor1.setAcceleration(ACCELERATION);
    motor2.setMaxSpeed(MAX_SPEED);
    motor2.setAcceleration(ACCELERATION);
    motor3.setMaxSpeed(MAX_SPEED);
    motor3.setAcceleration(ACCELERATION);
    motor4.setMaxSpeed(MAX_SPEED);
    motor4.setAcceleration(ACCELERATION);

    // Then give them to MultiStepper to manage
    steppers.addStepper(motor1);

    steppers.addStepper(motor2);
    steppers.addStepper(motor3);
    steppers.addStepper(motor4);

    // now that everything is set up, we need to go in the corner
    // and make sure the gyro is aligned

    // moveInDirection(south, 20);
    // moveInDirection(west, 100);
    // moveInDirection(180 + 45, 100);
    // check gyro is within 1 degree of 0, else reset gyro

    // while (true) {
    //   Serial.println("icite");
    //   delay(500);
    // }
    // turn the pallete to 45
    // pallete.write(45);
    // delay(10000);
    // pallete.write(90);
    zeroOutMagData();
}

void loop()
{
    // motor1.move(5000);
    // motor1.run();
    moveInDirection(forward, 12000);
    delay(1000);
    moveInDirection(left, 12000); 
    delay(1000);
    moveInDirection(backward, 12000);
    delay(1000);
    moveInDirection(right, 12000);
    delay(1000);

    // standby();       // the robot goes in a corner and looks for ball
    // getball();       // the robot seeks the oldest ball, then the next, until it dosent find one
    // alignAndShoot(); // the robot aligns itself with the goal and shoots the amount of balls it has seeked
    // alignAndGyro();  // the robot aligns itself with the the corner and makes sure the gyro is aligned (if not then it does it again)
    // testtrapdoor();
}

void testtrapdoor()
{
    trapDoor.write(OPEN);
    delay(1000);
    trapDoor.write(CLOSE);
    delay(1000);
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
    // convert to absolute direction to degrees from north
    float direction;
    if (dir == forward)
    {
        direction = 0.0;
    }
    else if (dir == right)
    {
        direction = 90.0;
    }
    else if (dir == backward)
    {
        direction = 180.0;
    }
    else if (dir == left)
    {
        direction = 270.0;
    }
    // float direction = 0.0;

    float directionWheel1 = 45.0;
    float directionWheel2 = 135.0;
    float directionWheel3 = 225.0;
    float directionWheel4 = 315.0;
    float weight1 = cos((direction - directionWheel1 + 90.0) * 1000 / 57296);
    float weight2 = cos((direction - directionWheel2 + 90.0) * 1000 / 57296);
    float weight3 = sin((direction - directionWheel3 + 90.0) * 1000 / 57296);
    float weight4 = sin((direction - directionWheel4 + 90.0) * 1000 / 57296);

    int stepsWheel1 = round(weight1 * distance);
    int stepsWheel2 = round(weight2 * distance);
    int stepsWheel3 = round(weight3 * distance);
    int stepsWheel4 = -round(weight4 * distance);

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
    // do it 5 times
    float headingSum = 0.0;
    for (int i = 0; i < magAvg; i++)
    {
        sensors_event_t event;
        mag.getEvent(&event);

        float Pi = 3.14159;
        float heading = (atan2(mapfloat(event.magnetic.x, -12, 49.27, -180, 180), mapfloat(event.magnetic.z, 50, 113.91, -180, 180)) * 180) / Pi;

        // Normalize to 0-360
        if (heading < 0)
        {
            heading = 360 + heading;
        }
        headingSum += heading;
    }

    return headingSum / magAvg;
}

float zeroOutMagData()
{
    // do it 5 times
    float headingSum = 0.0;
    for (int i = 0; i < magAvg; i++)
    {
        sensors_event_t event;
        mag.getEvent(&event);

        float Pi = 3.14159;
        float heading = (atan2(mapfloat(event.magnetic.x, -12, 49.27, -180, 180), mapfloat(event.magnetic.z, 50, 113.91, -180, 180)) * 180) / Pi;

        // Normalize to 0-360
        if (heading < 0)
        {
            heading = 360 + heading;
        }
        headingSum += heading;
    }
    float heading = headingSum / magAvg;

    initialDirection = heading;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}