#include <Servo.h>

const int trigPin = 11;
const int echoPin = 10;
const int motorPin = 2;

long duration;
int distance;

Servo myServo;

int calculateDistance(){

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration*0.034/2;
  return distance;
}

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  myServo.attach(6);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  for(int i=15;i<=165;i++){
    myServo.write(i);
    delay(30);
    distance = calculateDistance();

    Serial.print("Angle: ");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(".");
  }

  for(int i=165; i>=15;i--){
    myServo.write(i);
    delay(30);
    distance = calculateDistance();

    Serial.print("Angle: ");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(".");
  }

  digitalWrite(motorPin, HIGH);
  delay(1000);
  digitalWrite(motorPin, LOW);
  delay(1000);
}