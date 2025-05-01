#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int buzzerPin = 8;
const int buttonPin = 7;

int16_t ax, ay, az, gx, gy, gz;
float ax_g, ay_g, az_g;
float vx = 0, vy = 0, vz = 0;
float totalDistance = 0;
unsigned long lastTime;
float dt;

const float ACCEL_THRESHOLD = 1.2;
const float ACCEL_CEILING = 3.5; // NEW: ignore if accel is too high
const float VELOCITY_DAMPING = 0.75;
const float ALPHA = 0.98;

float angleX = 0, angleY = 0;

bool lastButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  lastTime = millis();
}

void loop() {
  // Read sensor data
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  ax_g = ax / 16384.0;
  ay_g = ay / 16384.0;
  az_g = az / 16384.0;

  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;

  float accelAngleX = atan2(ay_g, az_g) * (180 / 3.14159265);
  float accelAngleY = atan2(ax_g, az_g) * (180 / 3.14159265);

  angleX = ALPHA * (angleX + gx_dps * dt) + (1 - ALPHA) * accelAngleX;
  angleY = ALPHA * (angleY + gy_dps * dt) + (1 - ALPHA) * accelAngleY;

  float gravityX = sin(angleX * 3.14159265 / 180.0);
  float gravityY = sin(angleY * 3.14159265 / 180.0);
  ax_g -= gravityX;
  ay_g -= gravityY;

  // Thresholds
  if (fabs(ax_g) < ACCEL_THRESHOLD) ax_g = 0;
  if (fabs(ay_g) < ACCEL_THRESHOLD) ay_g = 0;
  if (fabs(az_g) < ACCEL_THRESHOLD) az_g = 0;

  // Check for ceiling condition
  bool inAccelCeiling = (fabs(ax_g) > ACCEL_CEILING || fabs(ay_g) > ACCEL_CEILING || fabs(az_g) > ACCEL_CEILING);

  if (!inAccelCeiling) {
    // Integrate acceleration to velocity
    vx += ax_g * dt;
    vy += ay_g * dt;
    vz += az_g * dt;

    // Apply damping
    if (ax_g == 0 && ay_g == 0 && az_g == 0) {
      vx *= VELOCITY_DAMPING;
      vy *= VELOCITY_DAMPING;
      vz *= VELOCITY_DAMPING;
    }

    // Integrate velocity to distance
    float dx = vx * dt;
    float dy = vy * dt;
    float dz = vz * dt;

    totalDistance += sqrt(dx * dx + dy * dy + dz * dz);
  } else {
    Serial.println("Acceleration too high — paused tracking.");
  }

  // Print distance
  Serial.print("Total Distance: ");
  Serial.print(totalDistance * 6);
  Serial.println(" m");

  // Button handling
  bool currentButtonState = digitalRead(buttonPin);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    int result = round((totalDistance * 6) / 0.8);
    beepDigits(result);
  }
  lastButtonState = currentButtonState;

  delay(100);
}

void beepDigits(int number) {
  int digits[10];
  int count = 0;

  // Split number into digits
  do {
    digits[count++] = number % 10;
    number /= 10;
  } while (number > 0);

  // Play digits in correct order
  for (int i = count - 1; i >= 0; i--) {
    for (int j = 0; j < digits[i]; j++) {
      tone(buzzerPin, 1000, 100);
      delay(500);
    }
    delay(1000);
  }
  noTone(buzzerPin);
}