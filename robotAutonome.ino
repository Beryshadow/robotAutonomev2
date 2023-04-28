/*
Skills ontario autonomous robot
court size = 609.6mm x 3733.6mm
robot size = 40cm x 40cm
*/

const int WIDTH = 40;
const int HEIGHT = 40;

// gyro
// https://github.com/DFRobot/DFRobot_ICG20660L
// https://github.com/DFRobot/DFRobot_ICG20660L/blob/master/examples/getGyroData/getGyroData.ino
#include <DFRobot_ICG20660L.h>
DFRobot_ICG20660L_IIC icg(/*addr=*/IIC_ADDR_SDO_H, &Wire);
float DPS = 3.1415926 / 180.0;

#ifdef ARDUINO_BBC_MICROBIT
#define CS_PIN 8 // The CS pin of sensor which is connected to the 8 digital io pin of micro:bit,and also can connected to other pin.
#else
#define CS_PIN 5 // The CS pin of sensor which is connected to the 5 digital io pin of MCU,and also can connected to other pin.
#endif

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
float ROBOTDIAMETER = 30.0; // cm
int oldestSide = 0;         // 0 left, 1 right

// ball struct
struct ball
{
    int x;
    int y;
    int width;
    int height;
    int color; // 1 = red, 2 = blue;
};

ball balls[10]; // the balls that are in the robot

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
    // gyroscope
    while (icg.begin(/*mode=*/icg.eRegMode) != 0)
    {
        Serial.println("failed. Please check whether the hardware connection is wrong.");
        delay(1000);
        Serial.print("Initialization sensor...");
    }
    Serial.println("done.");
    Serial.print("ICG20660L Device ID: 0x");
    Serial.println(icg.readID(), HEX);
    icg.enableSensor(icg.eGyroAxisXYZ);
    icg.configGyro(icg.eFSR_G_250DPS, icg.eGyro_DLPF_8173_32KHZ);
    icg.setSampleDiv(19);
    // magnetometer
    // https://github.com/adafruit/Adafruit_LSM303DLH_Mag

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

void gyroTest()
{
    // we know the gyro is aligned, so we test if its still within 1 degree of 0 else reset it
    int angleOfRobot = getMagData();
    if (abs(angleOfRobot) < 1)
    {
        gyroReset();
    }
}

void gyroReset()
{
    icg.reset();
}

void standby()
{
    // move side to side and scan with pixycam
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks)
    {
        for (int i = 0; i < pixy.ccc.numBlocks; i++)
        {
            // check if their is one older than 100
            if (pixy.ccc.blocks[i].m_age > 100)
            {
                break;
            }
        }
    }
}

void getball()
{
    //--------------------------Need to add exit condition based on time----------------------------
    // get the biggest ball according to the camera (the closest one)
    pixy.ccc.getBlocks();
    int x_biggest_ball = 0;
    int biggest_size = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
        if (pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height > biggest_size)
        {
            biggest_size = pixy.ccc.blocks[i].m_x * pixy.ccc.blocks[i].m_y;
            x_biggest_ball = pixy.ccc.blocks[i].m_x;
        }
    }
    // we now have the x_position of the biggest ball
    // we want to turn until its x_position is within 4 of 158
    int ajustable_turn = 4;
    while (x_biggest_ball > 158 + ajustable_turn || x_biggest_ball < 158 - ajustable_turn)
    {
        if (x_biggest_ball > 158 + ajustable_turn) // the ball is to the right
        {
            turnInDirection(-10.0);
        }
        else if (x_biggest_ball < 158 - ajustable_turn)
        {
            turnInDirection(10.0);
        }
    }

    // now we open the trap door
    trapDoor.write(OPEN);

    // roll forward until the ball is no longer seen
    pixy.ccc.getBlocks();
    while (pixy.ccc.numBlocks)
    {
        moveInDirection(forward, 20);
        pixy.ccc.getBlocks();
    }

    // close the trap door
    trapDoor.write(CLOSE);
}

void alignAndShoot()
{
    // align with goal
    // to do that we need to turn north
    // then go to a corner move specific distance turn, open the trap door and shoot

    turnInDirection(north);
    // go forwards a bit
    moveInDirection(forward, 10);
    turnInDirection(north); // wiggle a bit
    // move sideways until the button is pressed
    while (!digitalRead(button1Pin))
    {
        moveInDirection(left, 10);
    }
    moveInDirection(right, 10);
    // backwards until the back button is pressed
    while (!digitalRead(button2Pin))
    {
        moveInDirection(backward, 10);
    }
    moveInDirection(forward, 60 - HEIGHT);
    // turn a bit right
    turnToDirection(26.0);
    // move backwards
    moveInDirection(backward, 60 - HEIGHT);
    // open the trap door
    trapDoor.write(OPEN);
    // shoot
    shoot();
}

void shoot()
{
    pallete.write(30);
    // move forward
    moveInDirection(forward, 20);
    // close the trap door
    pallete.write(90);
    trapDoor.write(CLOSE);
    // move backwards
    moveInDirection(backward, 20);
    // open the trap door
    trapDoor.write(OPEN);
    pallete.write(30);
    // move forward
    moveInDirection(forward, 20);
    // close the trap door
    trapDoor.write(CLOSE);
    pallete.write(90);
}

void align()
{
    turnInDirection(north);
    // go forwards a bit
    moveInDirection(forward, 10);
    turnInDirection(north); // wiggle a bit
    // move sideways until the button is pressed
    while (!digitalRead(button1Pin))
    {
        moveInDirection(left, 10);
    }
    moveInDirection(right, 10);
    // backwards until the back button is pressed
    while (!digitalRead(button2Pin))
    {
        moveInDirection(backward, 10);
    }
    moveInDirection(forward, 10);

    // go in the corner
    moveInDirection(225, 15);
    gyroTest();
    // now depending on which was the oldest side we want to go to the opposite one
    if (oldestSide == 0)
    {
        // turn to the west and go east until the button is pressed
        turnInDirection(west);
        while (!digitalRead(button2Pin))
        {
            moveInDirection(east, 10);
        }
        moveInDirection(forward, 5);
        oldestSide = 1;
    }
    else
    {
        // turn to the east and go west until the button is pressed
        turnInDirection(east);
        while (!digitalRead(button1Pin))
        {
            moveInDirection(west, 10);
        }
        moveInDirection(forward, 5);
        oldestSide = 0;
    }
}

void alignAndGyro()
{
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
    return icg.getGyroDataX();
}