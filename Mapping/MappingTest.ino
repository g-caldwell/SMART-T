#include <math.h>

const float pointOneXY = 0.0;

struct CenterPoint {
  float x;
  float y;
};

float degreesToRadians(float degrees) {
  return degrees * (PI / 180.0);
}

float findX(float distanceTraveled, float previousX, float currentAngle, float previousAngle) {
    return previousX + (distanceTraveled * cos(currentAngle)) - (5.0 * cos(previousAngle));
}

float findY(float distanceTraveled, float previousY, float currentAngle, float previousAngle) {
    return previousY + (distanceTraveled * sin(currentAngle)) - (5.0 * sin(previousAngle));
}

CenterPoint findCenter(float x_2, float y_2, float x_3, float y_3) {
  float a = (x_2 * y_3) - (x_3 * y_2);
  float b = (y_2 * ((x_3 * x_3) + (y_3 * y_3))) - (y_3 * ((x_2 * x_2) + (y_2 * y_2)));
  float c = (x_3 * ((x_2 * x_2) + (y_2 * y_2))) - (x_2 * ((x_3 * x_3) + (y_3 * y_3)));

  return {-b / (2 * a), -c / (2 * a)};
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  float p2x = findX(6.363961, pointOneXY, degreesToRadians(135), degreesToRadians(90));
  float p2y = findY(6.363961, pointOneXY, degreesToRadians(135), degreesToRadians(90));

  float p3x = findX(11.035591, p2x, degreesToRadians(180), degreesToRadians(135));
  float p3y = findY(11.035591, p2y, degreesToRadians(180), degreesToRadians(135));

  CenterPoint center = findCenter(p2x, p2y, p3x, p3y);
  
  Serial.print("P2: "); Serial.print(p2x, 4); Serial.print(", "); Serial.println(p2y, 4);
  Serial.print("P3: "); Serial.print(p3x, 4); Serial.print(", "); Serial.println(p3y, 4);
  Serial.print("Center: "); Serial.print(center.x, 4); Serial.print(", "); Serial.println(center.y, 4);
}

void loop() {

}