#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define CONVERT_G_TO_MS2    9.80665f
#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

static unsigned long last_interval_ms = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Started");
  
  while (!Serial) delay(10); // Wait for serial

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  // Optional: configure accelerometer/gyro ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 ready.");
}

void loop() {
  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print accelerometer values converted to m/s²
    Serial.print(a.acceleration.x * CONVERT_G_TO_MS2);
    Serial.print('\t');
    Serial.print(a.acceleration.y * CONVERT_G_TO_MS2);
    Serial.print('\t');
    Serial.println(a.acceleration.z * CONVERT_G_TO_MS2);
  }
}
