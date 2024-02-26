#include "StringSplitter.h" //https://github.com/aharshac/StringSplitter

#include "Motor.h"
#include "MotorConfig.h"
#include "Commands.h"
//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/

#define BAUDRATE 115200

#define ENC_COUNT_REV 105 // Motor encoder output pulses per 360 degree revolution (measured manually)

MotorConfig motorConfig(ENC_COUNT_REV, .08, 0.0, 0.0);
Motor m1(motorConfig,
         6,
         11,
         12,
         2,
         4);
Motor m2(motorConfig,
         9,
         15,
         16,
         3,
         5);

//bool stopTune = false;
//double Kp = 0;
//double Ki = 0;
//double Kd = 0;

byte command = NO_OP;
double v1, v2;


void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    attachInterrupt(digitalPinToInterrupt(m1.getVelocityEncoder()), firstEncoderCallback, RISING);
    attachInterrupt(digitalPinToInterrupt(m2.getVelocityEncoder()), secondEncoderCallback, RISING);
}

void loop() {
/*
 *
 * {"command":1,"params":{"m1":15,"m2":8}}
 * */
    readCommand();
    executeCommand();
//    tune(6.28);
//    m1.move(6.28);
}

void firstEncoderCallback() { m1.interruptCallback(true); }

void secondEncoderCallback() { m2.interruptCallback(false); }

void readCommand() {
    if (!Serial.available()) {
        return;
    }
    String input = Serial.readStringUntil('\n');
    if (input.length() == 1) {
        command = atoi(input.c_str());
    } else {
        command = MOVE_MOTORS;
        //m1,m2
        //15.20
        StringSplitter *splitter = new StringSplitter(input, ',', 2);
        v1 = atof(splitter->getItemAtIndex(0).c_str());
        v2 = atof(splitter->getItemAtIndex(1).c_str());
        delete splitter;
    }
}

void executeCommand() {
    switch (command) {
        case PING:
            Serial.println("PONG");
            command = NO_OP;
            break;
        case MOVE_MOTORS:
            m1.move(v1);
            m2.move(v2);
            break;
        case GET_MOTOR_1_VELOCITY:
            Serial.println(m1.calculateAngularVelocity());
            break;
        case GET_MOTOR_2_VELOCITY:
            Serial.println(m2.calculateAngularVelocity());
            break;
        case GET_MOTOR_1_POSITION:
            Serial.println(m1.getPosition());
            break;
        case GET_MOTOR_2_POSITION:
            Serial.println(m2.getPosition());
            break;
        default:
            break;
    }
}

void logOutput() {
    auto actual = m1.calculateAngularVelocity();
    Serial.println(actual);
}

//void tune(Motor &m, double target) {
//    m.move(target);
//    m.tunePID(Kp, Ki, Kd);
//    if (Serial.available() > 0) {
//        // read the incoming byte:
//        String input = Serial.readStringUntil('\n');
//        if (input == "p" || input == "i" || input == "d") {
//            if (input == "p") { Kp += .1; }
//            else if (input == "i") { Ki += .1; }
//            else if (input == "d") { Kd += .1; }
//        } else if (input == "s") {
//            stopTune = true;
//        }
//        input == "";
//    }
//    if (stopTune) {
//        Serial.print(" Kp:");
//        Serial.print(Kp);
//        Serial.print(" Ki:");
//        Serial.print(Ki);
//        Serial.print(" Kd:");
//        Serial.print(Kd);
//        Serial.println();
//    } else {
//        auto actual = m.calculateAngularVelocity();
//        Serial.print(actual);
//        Serial.print(" ");
//        Serial.print(target);
//        Serial.println();
//    }
//}
