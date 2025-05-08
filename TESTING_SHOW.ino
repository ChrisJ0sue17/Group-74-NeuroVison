#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

const int trigPin = 11;
const int echoPin = 10;
const int motorPin = 6;
const int motorPin_2 = 5;
const int buzzerPin = 8;
const int buttonPin = 13;

Servo myServo;
MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;
float ax_g, ay_g, az_g;
float vx = 0, vy = 0, vz = 0;
float totalDistance = 0;
float dt;

const float ACCEL_THRESHOLD = 1.2;
const float ACCEL_CEILING = 3.5;
const float VELOCITY_DAMPING = 0.75;
const float ALPHA = 0.98;
float angleX = 0, angleY = 0;

bool lastButtonState = HIGH;

unsigned long lastScanTime = 0;
unsigned long scanInterval = 20;
unsigned long lastMPUTime = 0;
unsigned long mpuInterval = 50;
unsigned long lastBuzzerTime = 0;

int currentAngle = 15;
int direction = 1;
bool atCenterPause = false;
unsigned long centerPauseStart = 0;
const unsigned long centerPauseDuration = 800;

int distance;
bool isBuzzing = false;
int beepDigitsArray[10];
int beepCount = 0;
int beepIndex = 0;
int beepRepeat = 0;
bool beepOn = false;
unsigned long beepStartTime = 0;
unsigned long beepDelay = 500;
unsigned long digitPause = 1000;

int calculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void controlVibration(int dist, int angle) {
  int intensity = 0;
  if (dist <= 160) {
    intensity = map(dist, 0, 100, 160, 70);
    intensity = constrain(intensity, 50, 150);
  } else {
    intensity = 0;
  }

  if (angle >= 85 && angle <= 95) {
    analogWrite(motorPin, intensity);
    analogWrite(motorPin_2, intensity);
  } else if (angle < 85) {
    analogWrite(motorPin, intensity);
    analogWrite(motorPin_2, 0);
  } else {
    analogWrite(motorPin, 0);
    analogWrite(motorPin_2, intensity);
  }
}

void updateMPU() {
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  ax_g = ax / 16384.0;
  ay_g = ay / 16384.0;
  az_g = az / 16384.0;

  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;

  float accelAngleX = atan2(ay_g, az_g) * (180 / PI);
  float accelAngleY = atan2(ax_g, az_g) * (180 / PI);

  angleX = ALPHA * (angleX + gx_dps * dt) + (1 - ALPHA) * accelAngleX;
  angleY = ALPHA * (angleY + gy_dps * dt) + (1 - ALPHA) * accelAngleY;

  float gravityX = sin(angleX * PI / 180.0);
  float gravityY = sin(angleY * PI / 180.0);
  ax_g -= gravityX;
  ay_g -= gravityY;

  if (fabs(ax_g) < ACCEL_THRESHOLD) ax_g = 0;
  if (fabs(ay_g) < ACCEL_THRESHOLD) ay_g = 0;
  if (fabs(az_g) < ACCEL_THRESHOLD) az_g = 0;

  bool inAccelCeiling = (fabs(ax_g) > ACCEL_CEILING || fabs(ay_g) > ACCEL_CEILING || fabs(az_g) > ACCEL_CEILING);
  if (!inAccelCeiling) {
    vx += ax_g * dt;
    vy += ay_g * dt;
    vz += az_g * dt;

    if (ax_g == 0 && ay_g == 0 && az_g == 0) {
      vx *= VELOCITY_DAMPING;
      vy *= VELOCITY_DAMPING;
      vz *= VELOCITY_DAMPING;
    }

    float dx = vx * dt;
    float dy = vy * dt;
    float dz = vz * dt;

    totalDistance += sqrt(dx * dx + dy * dy + dz * dz);
  } else {
    Serial.println("Acceleration too high — paused tracking.");
  }

  Serial.print("Total Distance: ");
  Serial.print(totalDistance * 6);
  Serial.println(" m");
}

void handleServoScan() {
  unsigned long now = millis();

  if (atCenterPause) {
    if (now - centerPauseStart >= centerPauseDuration) {
      atCenterPause = false;
    }
    return;
  }

  if (now - lastScanTime >= scanInterval) {
    lastScanTime = now;

    myServo.write(currentAngle);
    distance = calculateDistance();
    controlVibration(distance, currentAngle);

    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print(", Distance: ");
    Serial.println(distance);

    currentAngle += direction;

    if (currentAngle >= 165 || currentAngle <= 15) {
      direction = -direction;
    }

    if (currentAngle == 90) {
      atCenterPause = true;
      centerPauseStart = now;
    }
  }
}

void startBeepSequence(int number) {
  beepIndex = 0;
  beepRepeat = 0;
  beepCount = 0;
  isBuzzing = true;
  do {
    beepDigitsArray[beepCount++] = number % 10;
    number /= 10;
  } while (number > 0);
}

void updateBuzzer() {
  unsigned long now = millis();
  if (!isBuzzing) return;

  if (beepRepeat < beepDigitsArray[beepCount - 1 - beepIndex]) {
    if (!beepOn && now - beepStartTime >= beepDelay) {
      tone(buzzerPin, 700, 100);
      beepOn = true;
      beepStartTime = now;
    } else if (beepOn && now - beepStartTime >= 100) {
      beepOn = false;
      beepRepeat++;
      beepStartTime = now;
    }
  } else {
    if (now - beepStartTime >= digitPause) {
      beepRepeat = 0;
      beepIndex++;
      beepStartTime = now;
    }
  }

  if (beepIndex >= beepCount) {
    noTone(buzzerPin);
    isBuzzing = false;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin_2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  myServo.attach(9);
}

void loop() {
  handleServoScan();

  if (millis() - lastMPUTime >= mpuInterval) {
    lastMPUTime = millis();
    updateMPU();
  }

  bool currentButtonState = digitalRead(buttonPin);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    int result = round((totalDistance * 6) / 0.8);
    startBeepSequence(result);
  }
  lastButtonState = currentButtonState;

  updateBuzzer();
}
