#include <Servo.h>

const int trigPin = 11;
const int echoPin = 10;
const int motorPin = 6;
const int motorPin_2 = 5;

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

void controlVibration(int dist, int angle){
  int intensity = 0;

  if(dist <= 160){
    intensity = map(dist, 0, 150, 200, 50);
    intensity = constrain(intensity, 50, 200);
  }
  else{
    intensity = 0;
  }

  if(angle >= 85 && angle <= 95){
    analogWrite(motorPin, intensity);
    analogWrite(motorPin_2, intensity);
  }
  else if(angle < 85){
    analogWrite(motorPin, intensity);
    analogWrite(motorPin_2, 0);
  }
  else{
    analogWrite(motorPin, 0);
    analogWrite(motorPin_2, intensity);
  }
}

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin_2, OUTPUT);
  Serial.begin(9600);
  myServo.attach(9);
}

void loop() {
  for(int i = 15;i <= 165; i++){
    myServo.write(i);
    delay(30);

    distance = calculateDistance();
    controlVibration(distance, i);

    Serial.print("Angle: ");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Distance: ");
    Serial.println(distance);

    if(i == 15 || i == 90 || i == 165){
      delay(800);
    }
  }

  for(int i=165; i>=15;i--){
    myServo.write(i);
    delay(30);

    distance = calculateDistance();
    controlVibration(distance, i);

    Serial.print("Angle: ");
    Serial.print(i);
    Serial.print(",");
    Serial.print("Distance: ");
    Serial.println(distance);

    if(i == 165 || i == 90 || i == 15){
      delay(800);
    }
  }
}