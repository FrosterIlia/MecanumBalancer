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
#define ENC1 GPIO_NUM_32

#define BIN1 GPIO_NUM_26
#define BIN2 GPIO_NUM_33
#define PWM2 GPIO_NUM_25
#define ENC2 GPIO_NUM_35

#define CIN1 GPIO_NUM_2
#define CIN2 GPIO_NUM_4
#define PWM3 GPIO_NUM_19
#define ENC3 GPIO_NUM_39

#define DIN1 GPIO_NUM_15
#define DIN2 GPIO_NUM_13
#define PWM4 GPIO_NUM_12
#define ENC4 GPIO_NUM_36

#define MOTORS_NUMBER 4

#define MAIN_PID_DT 1

#define NUM_READ 6

// #define ENC_KP 0.3
// #define ENC_KI 1.7
// #define ENC_KD 0.7

#define ENC_KP 0.45
#define ENC_KI 2.96
#define ENC_KD 1.01


float SETPOINT = -7;

Gyroscope gyro;

GyverPortal portal;

Motor motor1(AIN1, AIN2, PWM1);
Motor motor2(BIN1, BIN2, PWM2);
Motor motor3(CIN1, CIN2, PWM3);
Motor motor4(DIN1, DIN2, PWM4);


Timer pidTimer(MAIN_PID_DT);
Timer plotterTimer(100);


Pid mainPID(37.81, 169.37, 0.02, MAIN_PID_DT);
// Pid mainPID(0, 0, 0, MAIN_PID_DT);

Pid motor1PID(ENC_KP, ENC_KI, ENC_KD);
Pid motor2PID(ENC_KP, ENC_KI, ENC_KD);
Pid motor3PID(ENC_KP, ENC_KI, ENC_KD);
Pid motor4PID(ENC_KP, ENC_KI, ENC_KD);

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

int speed = 0;

MotorEncoder encoder1(ENC1);
MotorEncoder encoder2(ENC2);
MotorEncoder encoder3(ENC3);
MotorEncoder encoder4(ENC4);

void build() {
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);
  
  GP.SLIDER("slider_p", mainPID.kp, 0, 60, 0.01, 3);
  GP.SLIDER("slider_i", mainPID.ki, 0, 250, 0.01, 3);
  GP.SLIDER("slider_d", mainPID.kd, 0, 5, 0.01, 3);
  GP.SLIDER("slider_setpoint", SETPOINT, -10, 5, 0.1, 2);

  GP.BUTTON("start", "Start");
  GP.BUTTON("stop", "Stop");


  GP.SLIDER("motor_slider_p", motor1PID.kp, 0, 10, 0.01, 3);
  GP.SLIDER("motor_slider_i", motor1PID.ki, 0, 100, 0.01, 3);
  GP.SLIDER("motor_slider_d", motor1PID.kd, 0, 50, 0.01, 3);
  GP.SLIDER("motor_slider_setpoint", motor1PID.setpoint, -1050, 1050);
  
  GP.BUILD_END();
}

void action() {
    if (portal.click()) {
        if (portal.click("slider_p")){
            mainPID.kp = portal.getFloat();
            // Serial.println(mainPID.kp);
        } 
        if (portal.click("slider_i")){
            mainPID.ki = portal.getFloat();
            // Serial.println(mainPID.ki);
        } 
        if (portal.click("slider_d")){
            mainPID.kd = portal.getFloat() / 10;
            // Serial.println(mainPID.kd);
        }
        if (portal.click("slider_setpoint")){
            SETPOINT = portal.getInt();
            mainPID.setpoint = SETPOINT;
            // Serial.println(SETPOINT);
        }

        if (portal.click("motor_slider_p")){
            motor3PID.kp = portal.getFloat();
            // Serial.println(mainPID.kp);
        } 
        if (portal.click("motor_slider_i")){
            motor3PID.ki = portal.getFloat();
            // Serial.println(mainPID.ki);
        } 
        if (portal.click("motor_slider_d")){
            motor3PID.kd = portal.getFloat() / 10;
            // Serial.println(mainPID.kd);
        }
        if (portal.click("motor_slider_setpoint")){

            motor3PID.setpoint = portal.getInt();
            // Serial.println(SETPOINT);
        }

        if (portal.click("start")) {
            motor1.setMode(AUTO);
            motor2.setMode(AUTO);
            motor3.setMode(AUTO);
            motor4.setMode(AUTO);

            // Serial.println("START");
        }

        if (portal.click("stop")) {
            motor1.setMode(STOP);
            motor2.setMode(STOP);
            motor3.setMode(STOP);
            motor4.setMode(STOP);

            // Serial.println("STOP");
        }
    }
}

void IRAM_ATTR dmpReady() {
  gyro.mpuFlag = true;
}


void setup(){
    Serial.begin(9600);
    Serial.setTimeout(5);

    Wire.begin(21, 22);
    Wire.setClock(1000000UL);
    gyro.begin();
    attachInterrupt(GPIO_NUM_34, dmpReady, RISING);

    setupMotors();

    mainPID.set_limits(-1050, 1050);
    mainPID.setpoint = SETPOINT;
    mainPID.set_direction(REVERSE);

    motor1PID.set_limits(-1023, 1023);
    motor2PID.set_limits(-1023, 1023);
    motor3PID.set_limits(-1023, 1023);
    motor4PID.set_limits(-1023, 1023);

    setupPortal();

    attachEncoders();

    motor1.setMode(STOP);
    motor2.setMode(STOP);
    motor3.setMode(STOP);
    motor4.setMode(STOP);
}

float resultPID;

void loop(){
    portal.tick();
    // tickEncoders();
    // motor1.tick();
    // motor2.tick();
    // motor3.tick();
    // motor4.tick();



    if (pidTimer.isReady()){
        if (gyro.tick()){
            mainPID.input = gyro.get_angle();
        
            mainPID.compute();

            // resultPID = mainPID.get_output();

            // if (resultPID > 0){
            //     resultPID = map(resultPID, 0, 1050, 370, 1050);
            // }
            // else if (resultPID == 0) resultPID = 0;
            // else{
            //     resultPID = map(-resultPID, 0, 1050, 370, 1050) * -1;
            // }

            motor1.setSpeed(mainPID.get_output());
            motor2.setSpeed(mainPID.get_output());
            motor3.setSpeed(mainPID.get_output());
            motor4.setSpeed(mainPID.get_output()); 

            // motor1PID.setpoint = resultPID;  
            // motor2PID.setpoint = resultPID;  
            // motor3PID.setpoint = resultPID;   
            // motor4PID.setpoint = resultPID;  


        }
    }
    

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
                motor1PID.setpoint = speed;
                // motor1.setSmoothSpeed(Serial.parseInt());
                // motor2.setSmoothSpeed(speed);
                // motor3.setSmoothSpeed(speed);
                // motor4.setSmoothSpeed(speed); 
            break;
            
        }
    }



    if (plotterTimer.isReady()){
        //Motor graph
        // Serial.print("{Motor PID(Measured Speed:");
        // Serial.print(encoder3.getSpeed());
        // Serial.print(",Setpoint:");
        // Serial.print(motor3PID.setpoint);
        // Serial.print(",Output:");
        // Serial.print(motor3PID.get_output());
        // Serial.print(")}");

        // main PID graph
        Serial.print("{PID_Graph(Angle:");
        Serial.print(gyro.get_angle());
        Serial.print(",Setpoint:");
        Serial.print(mainPID.setpoint);
        Serial.print(")Output_Graph(Output:");
        Serial.print(mainPID.get_output());
        Serial.print(")}");
    }


}

void IRAM_ATTR encoderISR1(){
    if (motor1.getCurrentDirection()) encoder1.incTick();
    else encoder1.decTick();
    
    encoder1.setTimerOn();
}
void IRAM_ATTR encoderISR2(){
    if (motor2.getCurrentDirection()) encoder2.incTick();
    else encoder2.decTick();
    
    encoder2.setTimerOn();

}
void IRAM_ATTR encoderISR3(){
    if (motor3.getCurrentDirection()) encoder3.incTick();
    else encoder3.decTick();
    
    encoder3.setTimerOn();
}
void IRAM_ATTR encoderISR4(){
    if (motor4.getCurrentDirection()) encoder4.incTick();
    else encoder4.decTick();
    
    encoder4.setTimerOn();
}

void attachEncoders(){
    encoder1.attach(encoderISR1);
    encoder2.attach(encoderISR2);
    encoder3.attach(encoderISR3);
    encoder4.attach(encoderISR4);
}

void tickEncoders(){

    encoder1.tick();
    motor1PID.input = encoder1.getSpeed();
    motor1PID.compute();
    // if (motor1PID.setpoint == 0) motor1.setSpeed(0);
    // else motor1.setSpeed(motor1PID.get_output());

    encoder2.tick();
    motor2PID.input = encoder2.getSpeed();
    motor2PID.compute();
    // if (motor2PID.setpoint == 0) motor2.setSpeed(0);
    // else motor2.setSpeed(motor2PID.get_output());

    encoder3.tick();
    motor3PID.input = encoder3.getSpeed();
    motor3PID.compute();
    if (motor3PID.setpoint == 0) motor3.setSpeed(0);
    else motor3.setSpeed(motor3PID.get_output());

    encoder4.tick();
    motor4PID.input = encoder4.getSpeed();
    motor4PID.compute();
    // if (motor4PID.setpoint == 0) motor4.setSpeed(0);
    // else motor4.setSpeed(motor4PID.get_output());

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

void setupPortal(){
    WiFi.mode(WIFI_STA);
    WiFi.begin("BCIT Robotics Club", "IWillBuildARobot");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(WiFi.localIP());

    portal.attachBuild(build);
    portal.attach(action);
    portal.start();
}