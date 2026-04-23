#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;

#include "TurnSensor.h"

// Exact gearbox gear ratio: 100.37:1
// Distance from center of motor shaft to ground: ~1.9cm

// Counts per revolution: 100.37 (exact gear ratio) * 12 (encoder resolution) = 1204.44 (drive sprockets CPR)
const float countsPerRevolution = 1204.44;

// Circumference = 2π * radius
const float wheelCircumferenceMm = 2 * PI * 19; // ~119.38052083 mm
const float wheelCircumferenceCm = 11.566; //2 * PI * 1.9; // ~11.938052083 cm

// How many mm/cm in one count (distance reading)
const float mmPerCount = wheelCircumferenceMm / countsPerRevolution;
const float cmPerCount = wheelCircumferenceCm / countsPerRevolution;
// How many counts in one mm/cm (movement planning)
const float countsPerMm = countsPerRevolution / wheelCircumferenceMm;
const float countsPerCm = countsPerRevolution / wheelCircumferenceCm;

// Mapping Variables
const float P1[2] = {0, 0};
float P2[3];
float P3[2];
float center[2];
float radius;

// Moves continually at a set speed, until a border is detected
float moveToLine(int16_t leftMotorSpeed, int16_t rightMotorSpeed, uint16_t threshold) {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  uint16_t sensorValues[3];
  bool middleOnLine = false;

  while (!middleOnLine) {
    lineSensors.read(sensorValues);
    // < white line, > for black line
    middleOnLine = sensorValues[1] < threshold;

    int16_t countsL = encoders.getCountsLeft();
    int16_t countsR = encoders.getCountsRight();
    float averageCounts = (countsL + countsR) / 2.0;
    distanceTraveled = (float)averageCounts * cmPerCount;

    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
  motors.setSpeeds(0, 0);
  return distanceTraveled;
}

// Moves a set distance at a set speed
void move(float distance, int16_t leftMotorSpeed, int16_t rightMotorSpeed) {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveledCm = 0;
  // Reverse
  if(distance < 0) {
    while(distanceTraveledCm > distance) {
      int16_t countsL = encoders.getCountsLeft();
      int16_t countsR = encoders.getCountsRight();
      float averageCounts = (countsL + countsR) / 2.0;
      distanceTraveledCm = (float)averageCounts * cmPerCount;

      // Debugging
      Serial.print("Total Distance: ");
      Serial.print(distanceTraveledCm);
      Serial.println(" cm");

      motors.setSpeeds(-leftMotorSpeed, -rightMotorSpeed);
    }
  }
  // Forwards
  else if (distance > 0) {
    while(distanceTraveledCm < distance) {
      int16_t countsL = encoders.getCountsLeft();
      int16_t countsR = encoders.getCountsRight();
      float averageCounts = (countsL + countsR) / 2.0;
      distanceTraveledCm = (float)averageCounts * cmPerCount;

      // Debugging
      Serial.print("Total Distance: ");
      Serial.print(distanceTraveledCm);
      Serial.println(" cm");

      motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
    }
  }
  motors.setSpeeds(0, 0);
}

// Turns a specified amount at a specified speed
void pivot(int32_t degrees, int16_t leftMotorSpeed, int16_t rightMotorSpeed, int16_t brakePulseMs) {
  turnSensorReset();
  int32_t targetAngle = degrees * turnAngle1;

  if (degrees > 0) { // Counter Clockwise
    while ((int32_t)turnAngle < targetAngle) {
      turnSensorUpdate();
      motors.setSpeeds(-leftMotorSpeed, rightMotorSpeed);
    }
    motors.setSpeeds(leftMotorSpeed, -rightMotorSpeed);
  } 
  else { // Clockwise
    while ((int32_t)turnAngle > targetAngle) {
      turnSensorUpdate();
      motors.setSpeeds(leftMotorSpeed, -rightMotorSpeed);
    }
    motors.setSpeeds(-leftMotorSpeed, rightMotorSpeed);
  }
  delay(brakePulseMs); // brake time
  motors.setSpeeds(0, 0);
}

// Helper
float degreesToRadians(float degrees) {
  return degrees * (PI / 180.0);
}

float findX(float distanceTraveled, float previousX, float currentAngle, float previousAngle, float reverseDistance) {
  return previousX + (distanceTraveled * cos(degreesToRadians(currentAngle))) - (reverseDistance * cos(degreesToRadians(previousAngle)));
}

float getLiveDegrees() {
  // turnAngle is defined in TurnSensor.h
  // 0x20000000 (or 2^29) represents 45 degrees
  return (float)((int32_t)turnAngle) * 45.0 / 0x20000000;
}

float findY(float distanceTraveled, float previousY, float currentAngle, float previousAngle, float reverseDistance) {
  return previousY + (distanceTraveled * sin(degreesToRadians(currentAngle))) - (reverseDistance * sin(degreesToRadians(previousAngle)));
}

// Find the location of the center using the x and y values of points 2 and 3
void findCenter(float x_2, float y_2, float x_3, float y_3) {
  float a = (x_2 * y_3) - (x_3 * y_2);
  float b = (y_2 * ((x_3 * x_3) + (y_3 * y_3))) - (y_3 * ((x_2 * x_2) + (y_2 * y_2)));
  float c = (x_3 * ((x_2 * x_2) + (y_2 * y_2))) - (x_2 * ((x_3 * x_3) + (y_3 * y_3)));

  center[0] = -b / (2 * a);
  center[1] = -c / (2 * a);
}

// r = √h + k
void findRadius() {
  radius = sqrt((center[0]*center[0]) + (center[1]*center[1]));
}

int8_t mappingStep= 0;
void maneuver(int16_t reverseDistance, int16_t turnAngle, int16_t threshold) {
  float distanceTraveled = 0;

  move(reverseDistance, 209, 200);
  delay(200);
  pivot(turnAngle, 109, 100, 5);
  delay(200);
  distanceTraveled = moveToLine(216, 200, 500);
  delay(200);
  float heading = getLiveDegrees();

  if(mappingStep == 0) {
    P2[0] = findX(distanceTraveled, P1[0], heading + 90, 90, -abs(reverseDistance));
    P2[1] = findY(distanceTraveled, P1[1], heading + 90, 90, -abs(reverseDistance));
    P2[2] = heading + 90;
    mappingStep = 1;
  }
  else if(mappingStep == 1) {
    P3[0] = findX(distanceTraveled, P2[0], heading + P2[2], P2[2], -abs(reverseDistance));
    P3[1] = findY(distanceTraveled, P2[1], heading + P2[2], P2[2], -abs(reverseDistance));
    P2[2] = 0;
    mappingStep = 0;
  }
}

void align(int16_t alignSpeed, uint16_t threshold) {
  uint16_t sensorValues[3];

  while (true) {
    lineSensors.read(sensorValues);
    
    // 0 = Left sensor, 1 = Middle sensor, 2 = Right sensor
    bool leftOnLine = sensorValues[0] < threshold;
    bool middleOnLine = sensorValues[1] < threshold;
    bool rightOnLine = sensorValues[2] < threshold;

    if (leftOnLine && rightOnLine) { // Perfect alignment
      motors.setSpeeds(0, 0);
      break; 
    }
    else if (leftOnLine && !rightOnLine) { // Left sensor is on the line
      motors.setSpeeds(-alignSpeed, alignSpeed);
    } 
    else if (!leftOnLine && rightOnLine) { // Right sensor is on the line
      motors.setSpeeds(alignSpeed, -alignSpeed);
    } 
    else { // Neither sensor is on the line
      motors.setSpeeds(100, 100);
    }
  }
  delay(100);
}

void setup() {
  Serial.begin(115200);

  lineSensors.initThreeSensors();
  
  turnSensorSetup();
  delay(500);
  turnSensorReset();
}

void loop() {
  if(buttonA.getSingleDebouncedPress()) {
    // Reset Values
    P2[0] = 0; P2[1] = 0; P2[2] = 0;
    P3[0] = 0; P3[1] = 0;
    center[0] = 0; center[1] = 0;
    radius = 0;
    mappingStep = 0;

    delay(1000);

    // Find values
    align(100, 500);
    delay(200);
    maneuver(-10, 45, 500);
    maneuver(-10, 45, 500);
    findCenter(P2[0], P2[1], P3[0], P3[1]);
    findRadius();

    // Realign
    move(-4, 209, 200);
    align(100, 500);

    // Drive to center
    move(-radius + 10, 209, 200);
  }
}

// forward (L:216 R:200)
// back (L:209 R:200)
// pivot(45, 109, 100, 5);