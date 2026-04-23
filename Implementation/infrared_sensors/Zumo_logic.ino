//Owen Keller
//Gabe Caldwell
//block code snips XD


//imports
#include  <Wire.h>
#include <Zumo32U4.h>
#include <math.h>


#define LEFT 0
#define RIGHT 1

//construct zumo datatype
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

//construct math datatypes
const float pointOneXY = 0.0;
const uint8_t sensorThreshold = 1;

//degree static speeds
const uint16_t turnSpeedMax = 400;
const uint16_t turnSpeedMin = 100;

//speeds multiplier
const uint16_t deceleration = 10;
const uint16_t acceleration = 10;

bool senseDir = RIGHT;
// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;

// The time, in milliseconds, when an object was last seen.
uint16_t lastTimeObjectSeen = 0;

//structures custom
struct CenterPoint {
  float x;
  float y;

};


void setup() {
  // put your setup code here, to run once:
  //wait for button press
  proxSensors.initFrontSensor();
  display.clear();
  display.print(F("Press A"));
  buttonA.waitForButton();
  display.clear();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;

  if (objectSeen){  
    turnSpeed -= deceleration;

  } else { 
    turnSpeed += acceleration;
  }


  turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);

  if (objectSeen){
    ledYellow(1);

    lastTimeObjectSeen = millis();

    if (leftValue < rightValue){
        motor_Right();
        senseDir = RIGHT;
    }
    else if (leftValue > rightValue)
    {
      motor_Left();
      senseDir = LEFT;
    }
    else {  stop();}
  }
  else{
    ledYellow(0);

    if (senseDir == RIGHT)
    {
      motor_Right();
    }
    else
    {
      motor_Left();
    }
    }
}

//function controls left motor
void motor_Left(){  
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turningLeft = true;
  turningRight = false;
}
//function controls right motor 
void motor_Right(){
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turningLeft = false;
  turningRight = true;  
}

void stop(){
  motors.setSpeeds(0, 0);
  turningLeft = false;
  turningRight = false;
}

//math helpers
float degreesToRadians(float degrees) {
  return degrees * (PI / 180.0);
}
//used for finding the point_x
float findX(float distanceTraveled, float previousX, float currentAngle, float previousAngle) {
    return previousX + (distanceTraveled * cos(currentAngle)) - (5.0 * cos(previousAngle));
}

//used for finding point_y
float findY(float distanceTraveled, float previousY, float currentAngle, float previousAngle) {
    return previousY + (distanceTraveled * sin(currentAngle)) - (5.0 * sin(previousAngle));
}
//returns the x and y position of the center of a circle from 3 points
CenterPoint find_Center(float x_2, float y_2, float x_3, float y_3) {
  float a = (x_2 * y_3) - (x_3 * y_2);
  float b = (y_2 * ((x_3 * x_3) + (y_3 * y_3))) - (y_3 * ((x_2 * x_2) + (y_2 * y_2)));
  float c = (x_3 * ((x_2 * x_2) + (y_2 * y_2))) - (x_2 * ((x_3 * x_3) + (y_3 * y_3)));

  return {-b / (2 * a), -c / (2 * a)};
}