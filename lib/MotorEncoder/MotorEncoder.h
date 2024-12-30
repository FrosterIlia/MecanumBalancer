#pragma once
#include <Arduino.h>

#define HOLES_NUMBER 20

#define AVERAGING_BUFFER 5



class MotorEncoder{
    public:



    MotorEncoder(uint8_t pin){
        init();
        _pin = pin;
        _period = 100;
        _timer = millis();
        _prevTime = millis();
        pinMode(_pin, INPUT);
        
    }

    void attach(void (*function)()){
        attachInterrupt(_pin, function, CHANGE);
    }

    uint8_t getPin(){
        return _pin;
    }

    void tick(){

        if (millis() - _timer >= _period || _tickCounter >= 2){
            _speed = median(midArifm(((float)_tickCounter / ((float)millis() - (float)_prevTime) * 10000.0)));
            _tickCounter = 0;
            _prevTime = millis();
            _timer = millis();
            
        }
    }

    void incTick(){
        _tickCounter++;
    }



    float getSpeed(){
        return _speed;
    }

    private:

    float median(float newVal) {
        _medianBuf[_medianCount] = newVal;
        if (++_medianCount >= 3) _medianCount = 0;
        return (max(_medianBuf[0], _medianBuf[1]) == max(_medianBuf[1], _medianBuf[2])) ? max(_medianBuf[0], _medianBuf[2]) : max(_medianBuf[1], min(_medianBuf[0], _medianBuf[2]));
    }

    float midArifm(float newVal) {

        _sum += newVal; 
        _counter++;     
        if (_counter == AVERAGING_BUFFER) {    
            _prevResult = _sum / AVERAGING_BUFFER; 
            _sum = 0; 
            _counter = 0; 
        }
        return _prevResult;
    }

    uint8_t _pin;
    volatile uint16_t _tickCounter;
    float _speed;
    uint32_t _timer;
    uint16_t _period;
    uint32_t _prevTime;
    byte _counter = 0;
    float _prevResult = 0;
    float _sum = 0;
    float _medianBuf[3];
    byte _medianCount;
};
