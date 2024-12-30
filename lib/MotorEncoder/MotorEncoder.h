#pragma once
#include <Arduino.h>

#define HOLES_NUMBER 20

#define AVERAGING_BUFFER 5

float midArifm(float newVal);

class MotorEncoder{
    public:

    MotorEncoder() {}

    MotorEncoder(uint8_t pin){
        init();
        _pin = pin;
        _period = 100;
        _timer = millis();
        _prevTime = millis();
        pinMode(_pin, INPUT);
    }

    uint8_t getPin(){
        return _pin;
    }

    void tick(){


        if (millis() - _timer >= _period || _tickCounter >= 2){
            _speed = midArifm((float)_tickCounter / ((float)millis() - (float)_prevTime) * 10000.0);
            _tickCounter = 0;
            _prevTime = millis();
            _timer = millis();
            
        }
    }

    void incTick(){
        _tickCounter++;
    }



    uint16_t getSpeed(){
        return _speed;
    }

    private:

    float midArifm(float newVal) {
        static byte counter = 0; 
        static float prevResult = 0;
        static float sum = 0;  
        sum += newVal; 
        counter++;     
        if (counter == AVERAGING_BUFFER) {    
            prevResult = sum / AVERAGING_BUFFER; 
            sum = 0; 
            counter = 0; 
        }
        return prevResult;
    }

    uint8_t _pin;
    volatile uint16_t _tickCounter;
    float _speed;
    uint32_t _timer;
    uint16_t _period;
    uint32_t _prevTime;
};





template < uint8_t motorsNumber >
class MotorEncoders{
    public:
    MotorEncoders(const uint8_t* pins){
        init();
        for (uint8_t i = 0; i < motorsNumber; i++){
            encoders[i] = MotorEncoder(pins[i]);
        }
    }

    void attach(void (*function)()){
        for (uint8_t i = 0; i < motorsNumber; i++){
            attachInterrupt(encoders[i].getPin(), function, RISING);
        }
    }

    void tickISR(){
        for (uint8_t i = 0; i < motorsNumber; i++){
            if (digitalRead(encoders[i].getPin())){
                encoders[i].incTick();
                break;
            }
        }
    }

    void tick(){
        for (uint8_t i = 0; i < motorsNumber; i++){
            encoders[i].tick();
        }
    }

    uint16_t getSpeed(uint8_t encNumber){
        return encoders[encNumber].getSpeed();
    }

    MotorEncoder encoders[motorsNumber];
};
