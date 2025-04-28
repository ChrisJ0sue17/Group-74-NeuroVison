#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;
float ax_g, ay_g, az_g;
float vx = 0, vy = 0, vz = 0;
float totalDistance = 0;
unsigned long lastTime;
float dt;

// Threshold to ignore small accelerations (reduce drift)
const float ACCEL_THRESHOLD = 1.2;  // Adjust based on noise level
const float VELOCITY_DAMPING = 0.75; // Damping factor to reduce velocity when stationary

// Complementary filter weight
const float ALPHA = 0.98;

float angleX = 0, angleY = 0; // Estimated angles for gravity correction

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Use default SDA (A4) and SCL (A5) on Arduino Uno
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    lastTime = millis();
}

void loop() {
    // Read accelerometer and gyroscope data
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Convert raw acceleration data to g (assuming ±2g range)
    ax_g = ax / 16384.0;
    ay_g = ay / 16384.0;
    az_g = az / 16384.0;

    // Time difference in seconds
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Convert gyroscope readings to degrees per second
    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;

    // Compute angles using complementary filter
    float accelAngleX = atan2(ay_g, az_g) * (180 / 3.14159265);
    float accelAngleY = atan2(ax_g, az_g) * (180 / 3.14159265);

    angleX = ALPHA * (angleX + gx_dps * dt) + (1 - ALPHA) * accelAngleX;
    angleY = ALPHA * (angleY + gy_dps * dt) + (1 - ALPHA) * accelAngleY;

    // Adjust acceleration readings based on estimated tilt
    float gravityX = sin(angleX * 3.14159265 / 180.0);
    float gravityY = sin(angleY * 3.14159265 / 180.0);
    ax_g -= gravityX;
    ay_g -= gravityY;

    // Ignore small movements to reduce error accumulation
    if (fabs(ax_g) < ACCEL_THRESHOLD) ax_g = 0;
    if (fabs(ay_g) < ACCEL_THRESHOLD) ay_g = 0;
    if (fabs(az_g) < ACCEL_THRESHOLD) az_g = 0;

    // Integrate acceleration to get velocity
    vx += ax_g * dt;
    vy += ay_g * dt;
    vz += az_g * dt;

    // Apply damping when no significant acceleration is detected
    if (ax_g == 0 && ay_g == 0 && az_g == 0) {
        vx *= VELOCITY_DAMPING;
        vy *= VELOCITY_DAMPING;
        vz *= VELOCITY_DAMPING;
    }

    // Integrate velocity to get displacement
    float dx = vx * dt;
    float dy = vy * dt;
    float dz = vz * dt;

    // Calculate total distance traveled
    totalDistance += sqrt(dx * dx + dy * dy + dz * dz);

    // Print total distance traveled
    Serial.print("Total Distance: ");
    Serial.print(totalDistance * 6);
    Serial.println(" m");

    delay(100);  // Sampling delay
}
