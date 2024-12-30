#include "Arduino.h"
#include <Wire.h>
#include <GyverPortal.h>

#include "Motor.h"
#include "Gyroscope.h"
#include "Timer.h"
#include "PID.h"
#include "MotorEncoder.h"


#define AIN1 GPIO_NUM_5
#define AIN2 GPIO_NUM_18
#define PWM1 GPIO_NUM_23
#define ENC1 GPIO_NUM_36

#define BIN1 GPIO_NUM_26
#define BIN2 GPIO_NUM_33
#define PWM2 GPIO_NUM_25
#define ENC2 GPIO_NUM_39

#define CIN1 GPIO_NUM_2
#define CIN2 GPIO_NUM_4
#define PWM3 GPIO_NUM_19
#define ENC3 GPIO_NUM_35

#define DIN1 GPIO_NUM_15
#define DIN2 GPIO_NUM_13
#define PWM4 GPIO_NUM_12
#define ENC4 GPIO_NUM_32

#define MOTORS_NUMBER 4

#define MAIN_PID_DT 1

#define NUM_READ 6


float SETPOINT = -3.4;

Gyroscope gyro;

GyverPortal portal;

Motor motor1(AIN1, AIN2, PWM1);
Motor motor2(BIN1, BIN2, PWM2);
Motor motor3(CIN1, CIN2, PWM3);
Motor motor4(DIN1, DIN2, PWM4);


Timer pidTimer(MAIN_PID_DT);
Timer plotterTimer(100);


//Pid mainPID(43.3, 181.56, 0.19, MAIN_PID_DT);
Pid mainPID(0, 0, 0, MAIN_PID_DT);

uint32_t timerPID;

void setupMotors();
void setupPortal();
void IRAM_ATTR encoderISR1();
void IRAM_ATTR encoderISR2();
void IRAM_ATTR encoderISR3();
void IRAM_ATTR encoderISR4();
void attachEncoders();
void tickEncoders();
float getFilterSpeed();
float median(float newVal);



MotorEncoder encoder1(ENC1);
MotorEncoder encoder2(ENC2);
MotorEncoder encoder3(ENC3);
MotorEncoder encoder4(ENC4);

// void build() {
//   GP.BUILD_BEGIN();
//   GP.THEME(GP_DARK);
  
//   GP.SLIDER("slider_p", 0, 0, 60, 0.1, 1);
//   GP.SLIDER("slider_i", 0, 0, 250, 0.01, 3);
//   GP.SLIDER("slider_d", 0, 0, 5, 0.01, 3);
//   GP.SLIDER("slider_setpoint", SETPOINT, -20, 20, 0.1, 1);

//   GP.BUTTON("start", "Start");
//   GP.BUTTON("stop", "Stop");
  
//   GP.BUILD_END();
// }

// void action() {
//     if (portal.click()) {
//         if (portal.click("slider_p")){
//             mainPID.kp = portal.getFloat();
//             Serial.println(mainPID.kp);
//         } 
//         if (portal.click("slider_i")){
//             mainPID.ki = portal.getFloat();
//             Serial.println(mainPID.ki);
//         } 
//         if (portal.click("slider_d")){
//             mainPID.kd = portal.getFloat() / 10;
//             Serial.println(mainPID.kd);
//         }
//         if (portal.click("slider_setpoint")){
//             SETPOINT = portal.getFloat();
//             mainPID.setpoint = SETPOINT;
//             Serial.println(SETPOINT);
//         }

//         if (portal.click("start")) {
//             motor1.setMode(AUTO);
//             motor2.setMode(AUTO);
//             motor3.setMode(AUTO);
//             motor4.setMode(AUTO);

//             Serial.println("START");
//         }

//         if (portal.click("stop")) {
//             motor1.setMode(STOP);
//             motor2.setMode(STOP);
//             motor3.setMode(STOP);
//             motor4.setMode(STOP);

//             Serial.println("STOP");
//         }
//     }
// }

void IRAM_ATTR dmpReady() {
  gyro.mpuFlag = true;
}


void setup(){
    // noInterrupts();
    Serial.begin(9600);
    Serial.setTimeout(5);

    Wire.begin(21, 22);
    Wire.setClock(1000000UL);
    gyro.begin();
    attachInterrupt(GPIO_NUM_34, dmpReady, RISING);

    setupMotors();

    mainPID.set_limits(-1023, 1023);
    mainPID.setpoint = SETPOINT;
    mainPID.set_direction(REVERSE);

    // setupPortal();

    attachEncoders();
}

int speed = 0;

void loop(){
    portal.tick();
    tickEncoders();
    motor1.tick();
    motor2.tick();
    motor3.tick();
    motor4.tick();

    // if (pidTimer.isReady()){
    //     if (gyro.tick()){
    //         mainPID.input = gyro.get_angle();
        
    //         mainPID.compute();



    //         motor1.setSpeed(mainPID.get_output());
    //         motor2.setSpeed(mainPID.get_output());
    //         motor3.setSpeed(mainPID.get_output());
    //         motor4.setSpeed(mainPID.get_output());    

    //     }
    // }
    

    if (Serial.available() > 1){
        char key = Serial.read();

        switch(key){
            case 's':
            if (Serial.parseInt() == 0){
                motor1.setMode(STOP);
                motor2.setMode(STOP);
                motor3.setMode(STOP);
                motor4.setMode(STOP);
            }
            else{
                motor1.setMode(AUTO);
                motor2.setMode(AUTO);
                motor3.setMode(AUTO);
                motor4.setMode(AUTO);
            }
            break;

            case 'v':

                speed = Serial.parseInt();
                motor1.setSmoothSpeed(speed);
                motor2.setSmoothSpeed(speed);
                motor3.setSmoothSpeed(speed);
                motor4.setSmoothSpeed(speed); 
            break;
            
        }
        
    }

    if (plotterTimer.isReady()){
        Serial.print("{MyPlot(ENC1:");
        Serial.print(encoder1.getSpeed());
        Serial.print(",ENC2:");
        Serial.print(encoder2.getSpeed());
        Serial.print(",ENC3:");
        Serial.print(encoder3.getSpeed());
        Serial.print(",ENC4:");
        Serial.print(encoder4.getSpeed());
        Serial.print(")}");
    }

}

void IRAM_ATTR encoderISR1(){
    encoder1.incTick();
}
void IRAM_ATTR encoderISR2(){
    encoder2.incTick();

}
void IRAM_ATTR encoderISR3(){
    encoder3.incTick();
}
void IRAM_ATTR encoderISR4(){
    encoder4.incTick();
}

void attachEncoders(){
    encoder1.attach(encoderISR1);
    encoder2.attach(encoderISR2);
    encoder3.attach(encoderISR3);
    encoder4.attach(encoderISR4);
}

void tickEncoders(){
    encoder1.tick();
    encoder2.tick();
    encoder3.tick();
    encoder4.tick();
}




void setupMotors(){

    motor1.setResolution(10);
    motor1.setMinSignal(305);
    motor1.setSmooth(1);
    motor1.setMode(AUTO);

    
    motor2.setResolution(10);
    motor2.setMinSignal(299);
    motor2.setSmooth(1);
    motor2.setMode(AUTO);

    motor3.setResolution(10);
    motor3.setMinSignal(280);
    motor3.setSmooth(1);
    motor3.setMode(AUTO);

    motor4.setResolution(10);
    motor4.setMinSignal(305);
    motor4.setSmooth(1);
    motor4.setMode(AUTO);
}

// void setupPortal(){
//     WiFi.mode(WIFI_STA);
//     WiFi.begin("dlink-7850", "wsusa58776");
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println(WiFi.localIP());

//     portal.attachBuild(build);
//     portal.attach(action);
//     portal.start();
// }