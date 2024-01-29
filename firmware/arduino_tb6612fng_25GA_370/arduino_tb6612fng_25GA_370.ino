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

void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(m1.getEncoderPin()), firstEncoderCallback, RISING);
}

void loop() {
    auto target = 6.28;
    m1.move(target);
//    m1.movePWM(255);
    Serial.println("");
//    tune(target);
}

void firstEncoderCallback() { m1.interruptCallback(); }

void readCommand() {
    if (Serial.available() > 0) {
        // read the incoming byte:
        String json = Serial.readStringUntil('\n');  //{"command":"move_motor_1","params":{"velocity":0.4}}
        JsonDocument doc;
        deserializeJson(doc, json);
        String command = doc["command"];
        int velocity = doc["params"]["velocity"];

        if (command == "move_motor_1") {
            m1.move(velocity);
        } else if (command == "move_motor_2") {
//            m1.move(velocity);
        }
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

void tune(double target) {
//    if (Serial.available() > 0) {
//        // read the incoming byte:
//        String json = Serial.readStringUntil('\n');
//        if (json == "p") { m1.Kp += 10; }
//        else if (json == "i") { m1.Ki += .1; }
//        else if (json == "d") { m1.Kd += .1; }
//        else if (json == "s") {
//            stopTune = true;
//        }
//    }
//    if (!stopTune) {
//        auto actual = m1.getAngularVelocity();
//        Serial.print(actual);
//        Serial.print(" ");
//        Serial.print(target);
//        Serial.println();
//    } else {
//        Serial.print(" Kp:");
//        Serial.print(m1.Kp);
//        Serial.print(" Ki:");
//        Serial.print(m1.Ki);
//        Serial.print(" Kd:");
//        Serial.print(m1.Kd);
//        Serial.println();
//    }
}
