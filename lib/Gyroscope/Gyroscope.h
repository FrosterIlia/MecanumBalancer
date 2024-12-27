#pragma once
#include <Arduino.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

class Gyroscope {
public:

    volatile bool mpuFlag = false;

    Gyroscope() {
        init();

    }

    void begin() {

        //initialize mpu 
        mpu.initialize();

        mpu.dmpInitialize();
        mpu.setDMPEnabled(true);
        mpu.CalibrateGyro(6);


    }

    bool tick(){
        if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpuFlag = false;
            return true;
        }
        return false;
    }

    float get_angle(){
        return degrees(ypr[1]);
    }



private:

    MPU6050 mpu;
    
    uint8_t fifoBuffer[45]; //buffer
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
};