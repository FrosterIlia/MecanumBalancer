#include "Arduino.h"
#include <Wire.h>

#include <Gyroscope.h>

Gyroscope gyro;

void setup(){
    Serial.begin(9600);
    Wire.begin(21, 22);
    gyro.begin();
}

void loop(){
    Serial.print(">angle:");
    Serial.println(gyro.get_angle());
    delay(100);
}