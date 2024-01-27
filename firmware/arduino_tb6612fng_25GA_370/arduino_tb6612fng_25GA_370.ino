#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>

#include "Motor.h"
//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/

#define BAUDRATE 9600

#define ENC_COUNT_REV 105 // Motor encoder output pulses per 360 degree revolution (measured manually)
#define WHEEL_RADIUS_METERS 0.0349

Motor m1(ENC_COUNT_REV,
         WHEEL_RADIUS_METERS,
         5,
         6,
         7,
         2);

void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(m1.getEncoderPin()), firstEncoderCallback, RISING);
}

void loop() {
    auto target = 0.4;
    m1.move(target);

    m1.odom();
//    m1.movePWM(255);
//    readCommand();

    auto actual = m1.getLinearVelocity();
//    Serial.println();
    Serial.print(target);
    Serial.print(" ");
    Serial.print(actual);
    Serial.println();
//    Serial.println(" meters per second");
}

void firstEncoderCallback() { m1.interruptCallback(); }

void writeCommand() {
    // JsonDocument doc;

    // doc["sensor"] = "gps";
    // doc["time"] = 1351824120;
    // doc["data"][0] = 48.756080;
    // doc["data"][1] = 2.302038;

    // serializeJson(doc, Serial);
}

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

//        Serial.println("OK");
    }
}
