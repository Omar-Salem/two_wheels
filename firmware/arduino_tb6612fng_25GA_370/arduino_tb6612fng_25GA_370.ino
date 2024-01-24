#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>

#include "Motor.h"
//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/
/**https://mikroelectron.com/Product/25GA-370-12V-400RPM-DC-Reducer-Gear-Motor-with-Encoder
Red Wire - positive power supply of motor(+)(change positive and negative of motor the rotation will change)

White Wire - negative power supply of motor(-)(change positive and negative of motor the rotation will change))

Blue Wire - positive of encoder power supply(+)(3.3-5V),cannot be wrong

Black Wire - negative of encoder power supply(-)(3.3-5V),cannot be wrong

Yellow Wire - signal feedback (motor one turn has 11 signals)

Green Wire - signal feedback (motor one turn has 11 signals)

**/
//https://automaticaddison.com/how-to-calculate-the-velocity-of-a-dc-motor-with-encoder/

#define BAUDRATE 9600

#define ENC_COUNT_REV 105 // Motor encoder output pulses per 360 degree revolution (measured manually)
#define WHEEL_RADIUS_METERS 0.0349

Motor m1(ENC_COUNT_REV,
         WHEEL_RADIUS_METERS,
         3,
         8,
         9,
         2);

void setup() {
    Serial.begin(BAUDRATE);
    m1.initialize();
    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(m1.getEncoderPin()), firstMotorInterruptCallback, RISING);
}

void loop() {

    // JsonDocument doc;

    // doc["sensor"] = "gps";
    // doc["time"] = 1351824120;
    // doc["data"][0] = 48.756080;
    // doc["data"][1] = 2.302038;

    // serializeJson(doc, Serial);


    if (Serial.available() > 0) {
        // read the incoming byte:
        String json = Serial.readStringUntil('\n');  //{"command":"move_motor_1","params":{"velocity":255}}
        JsonDocument doc;
        deserializeJson(doc, json);
        String command = doc["command"];
        int velocity = doc["params"]["velocity"];

        if (command == "move_motor_1") {
            m1.move(velocity);
        } else if (command == "move_motor_2") {
//            m1.move(velocity);
        }

        Serial.println("OK");
    }
    m1.odom();

    Serial.print(" Linear Velocity: ");
    Serial.print(m1.getLinearVelocity());
    Serial.println(" meters per second");
}

void firstMotorInterruptCallback() { m1.interruptCallback(); }
