#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>
//TB6612FNG

//https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/
/**https://mikroelectron.com/Product/25GA-370-12V-400RPM-DC-Reducer-Gear-Motor-with-Encoder
Red Wire - positive power supply of motor(+)(change positive and negative of motor the rotation will change)

White Wire - negative power supply of motor(-)(change positive and negative of motor the rotation will change))

Blue Wire - positive of encoder power supply(+)(3.3-5V),cannot be wrong

Black Wire - negative of encoder power supply(-)(3.3-5V),cannot be wrong

Yellow Wire - signal feedback (motor one turn has 11 signals)

Green Wire - signal feedback (motor one turn has 11 signals)

**/

//motor A connected between A01 and A02
//motor B connected between B01 and B02


#define BAUDRATE 9600
//Motor A
int PWMA = 3;  //Speed control
int AIN1 = 9;  //Direction
int AIN2 = 8;  //Direction

//Motor B
int PWMB = 5;   //Speed control
int BIN1 = 11;  //Direction
int BIN2 = 12;  //Direction

void setup() {
    Serial.begin(BAUDRATE);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
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
        Serial.print("command: ");
        Serial.println(command);


        Serial.print("velocity: ");
        Serial.println(velocity);

        if (command == "move_motor_1") {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);

            analogWrite(PWMA, velocity);
        } else if (command == "move_motor_2") {
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);

            analogWrite(PWMB, velocity);
        }

        Serial.println("OK");
    }
}
