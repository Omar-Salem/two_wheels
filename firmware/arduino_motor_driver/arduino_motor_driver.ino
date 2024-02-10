#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>

#include "Motor.h"
#include "MotorConfig.h"
#include "Commands.h"
//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/

#define BAUDRATE 115200

#define ENC_COUNT_REV 105 // Motor encoder output pulses per 360 degree revolution (measured manually)

MotorConfig _25GA_370_MotorConfig(ENC_COUNT_REV, .08, 0.0, 0.0);
Motor m1(_25GA_370_MotorConfig,
         5,
         6,
         7,
         2,
         4);
Motor m2(_25GA_370_MotorConfig,
         9,
         10,
         11,
         3,
         8);

//bool stopTune = false;
//double Kp = 0;
//double Ki = 0;
//double Kd = 0;

byte command;
double velocity = 0;


void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    attachInterrupt(digitalPinToInterrupt(m1.getVelocityEncoder()), firstEncoderCallback, RISING);
    attachInterrupt(digitalPinToInterrupt(m2.getVelocityEncoder()), secondEncoderCallback, RISING);
}

void loop() {
/*
 *
 * {"command":1,"params":{"velocity":6.28}}
 * */
    readCommand();
    executeCommand();
//    tune(6.28);
//    m1.move(6.28);
}

void firstEncoderCallback() { m1.interruptCallback(); }

void secondEncoderCallback() { m2.interruptCallback(); }

void readCommand() {
    if (!Serial.available()) {
        return;
    }
    String json = Serial.readStringUntil('\n');
    JsonDocument doc;
    deserializeJson(doc, json);
    command = doc["command"];
    if (command == MOVE_MOTOR_1 || command == MOVE_MOTOR_2) {
        velocity = doc["params"]["velocity"];
    }
}

void writeCommand(double value) {
    JsonDocument doc;
    doc["value"] = value;
    serializeJson(doc, Serial);
    Serial.println();
}

void executeCommand() {
    switch (command) {
        case MOVE_MOTOR_1:
            m1.move(velocity);
            break;
        case MOVE_MOTOR_2:
            m2.move(velocity);
            break;
        case GET_MOTOR_1_VELOCITY:
            writeCommand(m1.calculateAngularVelocity());
            break;
        case GET_MOTOR_2_VELOCITY:
            writeCommand(m2.calculateAngularVelocity());
            break;
        case GET_MOTOR_1_POSITION:
            writeCommand(m1.getPosition());
            break;
        case GET_MOTOR_2_POSITION:
            writeCommand(m2.getPosition());
            break;
        default:
            break;
    }
}

void logOutput() {
    auto actual = m1.calculateAngularVelocity();
    Serial.print(actual);
    Serial.println();
}

//void tune(double target) {
//    m1.move(target);
//    m1.tunePID(Kp, Ki, Kd);
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
//        auto actual = m1.calculateAngularVelocity();
//        Serial.print(actual);
//        Serial.print(" ");
//        Serial.print(target);
//        Serial.println();
//    }
//}
