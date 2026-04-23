//Owen Keller
//Gabe Caldwell
//block code snips XD


//imports
#include  <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

//construct zumo datatype
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
//construct math datatypes
const float pointOneXY = 0.0;

//structures custom
struct CenterPoint {
  float x;
  float y;

};


void setup() {
  // put your setup code here, to run once:
  //wait for button press
  buttonA.waitForButton();

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
 
}

//function controls left motor
void motor_Left(int speed, bool reverse){
  if (reverse == true){
      motors.setLeftSpeed(speed);  

  }

  else{
      motors.setLeftSpeed(speed*(-1));
  }
  
}
//function controls right motor 
void motor_Right(int speed, bool reverse){
  if (reverse == true){
      motors.setRightSpeed(speed);  

  }
  else{
      motors.setRightSpeed(speed*(-1));
  }
  
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