// sans gyro
/*
Skills ontario autonomous robot
court size = 609.6mm x 3733.6mm
robot size = 45cm x 45cm
*/

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
// motor1
#define motor1_direction 22
#define motor1_step 23
// motor2
#define motor2_direction 24
#define motor2_step 25
// motor3
#define motor3_direction 26
#define motor3_step 27
// motor4
#define motor4_direction 28
#define motor4_step 29

AccelStepper motor1(AccelStepper::DRIVER, motor1_step, motor1_direction);
AccelStepper motor2(AccelStepper::DRIVER, motor2_step, motor2_direction);
AccelStepper motor3(AccelStepper::DRIVER, motor3_step, motor3_direction);
AccelStepper motor4(AccelStepper::DRIVER, motor4_step, motor4_direction);

long position[4] = {0, 0, 0, 0};

MultiStepper steppers;

const float MAX_SPEED = 200.0;
const float ACCELERATION = 100.0;

// servo
// https://www.instructables.com/Arduino-Servo-Motors/
#include <Servo.h>
#define servoPin 37
Servo trapDoor;
const int OPEN = 90;
const int CLOSE = 0;

// motor
// https://forum.arduino.cc/t/sabertooth-2x12-guidance/661660
// https://se.inf.ethz.ch/people/wei/robots/arduino_sabertooth2x12/sabertooth.html
#define motorPin 35
Servo pallete;

// camera
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
#include <Pixy2.h>
Pixy2 pixy;

// buttons
#define button1Pin 31 // side button
#define button2Pin 33 // back button

// misc
float ROBOTDIAMETER = 40.0; // cm

void setup()
{
    /// init everything

    // serial monitor
    Serial.begin(115200);
    while (!Serial)
    { // Waiting for USB Serial COM port to open.
    }
    // camera
    pixy.init();

    // servo
    trapDoor.attach(servoPin);
    trapDoor.write(0);

    // buttons
    pinMode(button1Pin, INPUT);
    pinMode(button2Pin, INPUT);

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

    moveInDirection(south, 20);
    moveInDirection(west, 100);
    moveInDirection(180 + 45, 100);
    // check gyro is within 1 degree of 0, else reset gyro
}

void loop()
{
    standby();       // the robot goes in a corner and looks for ball
    getball();       // the robot seeks the oldest ball, then the next, until it dosent find one
    alignAndShoot(); // the robot aligns itself with the goal and shoots the amount of balls it has seeked
    alignAndGyro();  // the robot aligns itself with the the corner and makes sure the gyro is aligned (if not then it does it again)
}
