#include <Servo.h>

const int trigPin = 11;
const int echoPin = 10;
const int motorPin = 3;

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

void controlVibration(int dist){
  if(dist >= 213){
    analogWrite(motorPin, 0);
  }
  else if(dist < 200){
    analogWrite(motorPin, 60);
  }
  else if(dist < 120){
    analogWrite(motorPin, 120);
  }
  else if(dist < 60){
    analogWrite(motorPin, 200);
  }
}

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  myServo.attach(6);
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
    Serial.println(distance);
    
    controlVibration(distance);
  }

  for(int i=165; i>=15;i--){
    myServo.write(i);
    delay(30);
    distance = calculateDistance();

    Serial.print("Angle: ");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Distance: ");
    Serial.println(distance);

    controlVibration(distance);
  }
}