#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>

#include "Motor.h"
#include "MotorConfig.h"
#include "Commands.h"
//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/

#define BAUDRATE 115200

#define ENC_COUNT_REV 105 // Motor encoder output pulses per 360 degree revolution (measured manually)

MotorConfig _25GA_370_MotorConfig(ENC_COUNT_REV, 1.0, 0, 0);
Motor m1(_25GA_370_MotorConfig,
         5,
         6,
         7,
         2,
         3);
Motor m2(_25GA_370_MotorConfig,
         5,
         6,
         7,
         2,
         3);

//bool stopTune = false;
byte command;
double velocity;

unsigned long moveCommandLastReceivedOn;
#define DEAD_MAN_SWITCH_INTERVAL_MILLI_SEC 500

void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    attachInterrupt(digitalPinToInterrupt(m1.getEncoderPin()), firstEncoderCallback, RISING);
    attachInterrupt(digitalPinToInterrupt(m2.getEncoderPin()), secondEncoderCallback, RISING);
}

void loop() {
    readCommand();
    Serial.println(command);
    executeCommand();
    if (millis() - moveCommandLastReceivedOn >= DEAD_MAN_SWITCH_INTERVAL_MILLI_SEC) {
        m1.move(0);
        m2.move(0);
    }
//    logOutput();
}

void firstEncoderCallback() { m1.interruptCallback(); }

void secondEncoderCallback() { m2.interruptCallback(); }

/*
 *
 * {"command":1,"params":{"velocity":6.28}}
 * */
void readCommand() {
    if (Serial.available() > 0) {
        String json = Serial.readStringUntil('\n');
        JsonDocument doc;
        deserializeJson(doc, json);
        command = doc["command"];
        if (command == MOVE_MOTOR_1 || command == MOVE_MOTOR_2) {
            moveCommandLastReceivedOn = millis();
            velocity = doc["params"]["velocity"];
        }
    }
}

void writeCommand(double value) {
    JsonDocument doc;
    doc["value"] = value;
    serializeJson(doc, Serial);
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
            writeCommand(m1.getAngle());
            break;
        case GET_MOTOR_2_POSITION:
            writeCommand(m2.getAngle());
            break;
        default:
            //TODO throw exception?  https://stackoverflow.com/a/10229439/801743
            break;
    }
}

void logOutput() {
    auto actual = m1.calculateAngularVelocity();
    Serial.print(actual);
    Serial.print(" ");
    Serial.print(velocity);
    Serial.println();
}

//void tune(double target) {
//    if (Serial.available() > 0) {
//        // read the incoming byte:
//        String json = Serial.readStringUntil('\n');
//        if (json == "p") { m1.Kp += 1; }
//        else if (json == "i") { m1.Ki += .1; }
//        else if (json == "d") { m1.Kd += .1; }
//        else if (json == "s") {
//            stopTune = true;
//        }
//    }
//    if (stopTune) {
//        Serial.print(" Kp:");
//        Serial.print(m1.Kp);
//        Serial.print(" Ki:");
//        Serial.print(m1.Ki);
//        Serial.print(" Kd:");
//        Serial.print(m1.Kd);
//        Serial.println();
//    } else {
//        auto actual = m1.calculateAngularVelocity();
//        Serial.print(actual);
//        Serial.print(" ");
//        Serial.print(target);
//        Serial.println();
//    }
//}
