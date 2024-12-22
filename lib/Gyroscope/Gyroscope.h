#pragma once
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"

class Gyroscope {
public:
  Gyroscope() {
    init();

  }

  void begin() {

    //initialize mpu 
    mpu.initialize();

    calibrate();
  }

  void calibrate() {
    // Serial.println(mpu.getXAccelOffset());
    // Serial.println(mpu.getYAccelOffset());
    // Serial.println(mpu.getZAccelOffset());
    // Serial.println(mpu.getXGyroOffset());
    // Serial.println(mpu.getYGyroOffset());
    // Serial.println(mpu.getZGyroOffset());

    // mpu.setXAccelOffset(1494);
    // mpu.setYAccelOffset(2163);
    // mpu.setZAccelOffset(1100);
    // mpu.setXGyroOffset(88);
    // mpu.setYGyroOffset(25);
    // mpu.setZGyroOffset(-6);
  }

  float get_angle() {
    int16_t ax = mpu.getAccelerationX();  // ускорение по оси Х
    int16_t ay = mpu.getAccelerationY();

    ax = constrain(ax, -16384, 16384);  // ограничиваем +-1g
    float angle_x = ax / 16384.0;       // переводим в +-1.0

    ay = constrain(ay, -16384, 16384);  // ограничиваем +-1g
    float angle_y = ay / 16384.0;       // переводим в +-1.0

    int result = degrees(atan2f(angle_y, angle_x));

    return result;
  }

private:
  MPU6050 mpu;
};