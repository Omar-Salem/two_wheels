#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>

#include "Motor.h"
//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/

#define BAUDRATE 9600

#define ENC_COUNT_REV 105 // Motor encoder output pulses per 360 degree revolution (measured manually)


Motor m1(ENC_COUNT_REV,
         5,
         6,
         7,
         2,
         3);

//bool stopTune = false;
unsigned int command;
double velocity;
#define MOVE_MOTOR_1 1
#define MOVE_MOTOR_2 2

void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(m1.getEncoderPin()), firstEncoderCallback, RISING);
}

void loop() {
    readCommand();
    Serial.println(command);
    executeCommand();
//    auto target = 6.28;
//    if (!stopTune) {
//        m1.move(target);
//        stopTune = true;
//    }
//    tune(target);
//    Serial.println("");
//    m1.movePWM(200);
}

void firstEncoderCallback() { m1.interruptCallback(); }

void readCommand() {
    if (Serial.available() > 0) {
        //{"command":1,"params":{"velocity":6.28}}
        String json = Serial.readStringUntil('\n');
        JsonDocument doc;
        deserializeJson(doc, json);
        command = doc["command"];
        velocity = doc["params"]["velocity"];
        //TODO check if speed is over 0.6
    }
}

void writeCommand() {
    // JsonDocument doc;

    // doc["sensor"] = "gps";
    // doc["time"] = 1351824120;
    // doc["data"][0] = 48.756080;
    // doc["data"][1] = 2.302038;

    // serializeJson(doc, Serial);
}

void executeCommand() {
    if (command == MOVE_MOTOR_1) {
        m1.move(velocity);
    } else if (command == MOVE_MOTOR_2) {
//            m1.move(velocity);
    }
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
//        auto actual = m1.getAngularVelocity();
//        Serial.print(actual);
//        Serial.print(" ");
//        Serial.print(target);
//        Serial.println();
//    }
//}
