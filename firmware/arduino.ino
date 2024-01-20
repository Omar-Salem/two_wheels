#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
//TB6612FNG


//motor A connected between A01 and A02
//motor B connected between B01 and B02

// int STBY = 10; //standby
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

  // char json[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
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
    Serial.print("command");
    Serial.println(command);


    Serial.print("velocity");
    Serial.println(velocity);

    if (command == "move_motor_1") {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);

      analogWrite(PWMA, velocity);
    }
    if (command == "move_motor_2") {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);

      analogWrite(PWMB, velocity);
    }

    Serial.println("OK");
  }
}
