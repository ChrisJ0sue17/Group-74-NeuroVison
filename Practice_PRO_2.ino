#include <Servo.h>

const int trigPin = 11;
const int echoPin = 10;
const int servoPin = 5;

Servo servo;

float duration, distance;
int angle = 0;



void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(servoPin);

}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = (duration*0.0343)/2;

  Serial.println("Distance: ");
  Serial.println(distance);

  delay(80);

  for(angle = 0; angle < 180; angle++){
    servo.write(angle);
    delay(10);
  }
  for(angle = 180; angle > 0; angle--){
    servo.write(angle);
    delay(10);
  }

}
