#include <Arduino.h>

// Basic demo for accelerometer readings from Adafruit ICM20649

#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>

Adafruit_ICM20649 icm;

#define PIN 12
#define NUM_PIXELS 30

Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, PIN, NEO_GRB);

void setup(void) {
    Serial.begin(115200);

    strip.begin();
    strip.setBrightness(32);

    if (!icm.begin_I2C()) {
        Serial.println("Failed to find ICM20649 chip");
        while (1) { delay(10); }
    }
    Serial.println("ICM20649 Found!");

    icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    Serial.print("Accelerometer range set to: ");
    switch (icm.getAccelRange()) {
        case ICM20649_ACCEL_RANGE_4_G: Serial.println("+-4G"); break;
        case ICM20649_ACCEL_RANGE_8_G: Serial.println("+-8G"); break;
        case ICM20649_ACCEL_RANGE_16_G: Serial.println("+-16G"); break;
        case ICM20649_ACCEL_RANGE_30_G: Serial.println("+-30G"); break;
    }

    icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
    Serial.print("Gyro range set to: ");
    switch (icm.getGyroRange()) {
        case ICM20649_GYRO_RANGE_500_DPS: Serial.println("500 degrees/s"); break;
        case ICM20649_GYRO_RANGE_1000_DPS: Serial.println("1000 degrees/s"); break;
        case ICM20649_GYRO_RANGE_2000_DPS: Serial.println("2000 degrees/s"); break;
        case ICM20649_GYRO_RANGE_4000_DPS: Serial.println("4000 degrees/s"); break;
    }

    //  icm.setAccelRateDivisor(4095);
    uint16_t accel_divisor = icm.getAccelRateDivisor();
    float    accel_rate    = 1125 / (1.0 + accel_divisor);

    Serial.print("Accelerometer data rate divisor set to: ");
    Serial.println(accel_divisor);
    Serial.print("Accelerometer data rate (Hz) is approximately: ");
    Serial.println(accel_rate);

    icm.setGyroRateDivisor(10);
    uint8_t gyro_divisor = icm.getGyroRateDivisor();
    float   gyro_rate    = 1100 / (1.0 + gyro_divisor);

    Serial.print("Gyro data rate divisor set to: ");
    Serial.println(gyro_divisor);
    Serial.print("Gyro data rate (Hz) is approximately: ");
    Serial.println(gyro_rate);
    Serial.println();
}

float    heading = 0.0;
uint32_t last    = 0;

void loop() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp);

    float z = gyro.gyro.z;
    heading += z;

    uint32_t now = millis();
    if (now - last > 100) {
        last = now;
        Serial.println(heading);
    }

    uint16_t hue = (uint16_t) (abs(heading) * 100);

    for (uint32_t i = 0; i < NUM_PIXELS; i++) { strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue))); }
    strip.show();
    Serial.println(hue);

    delay(10);
}
