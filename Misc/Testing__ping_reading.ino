const int sens_1 = 11;

void setup() {
  // put your setup code here, to run once:
  Serial.print("Started");
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  long duration, inches, cm;
  

  pinMode(sens_1, OUTPUT);
  digitalWrite(sens_1, LOW);
  delayMicroseconds(2);
  digitalWrite(sens_1, HIGH);
  delayMicroseconds(5);
  digitalWrite(sens_1, LOW);

  pinMode(sens_1, INPUT);
  duration = pulseIn(sens_1, HIGH);



  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(1000);


}



long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}