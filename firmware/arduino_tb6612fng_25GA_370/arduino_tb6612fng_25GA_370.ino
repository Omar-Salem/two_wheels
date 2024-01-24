#include <ArduinoJson.h>    //https://arduinojson.org/
#include <ArduinoJson.hpp>


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

//motor A connected between A01 and A02
//motor B connected between B01 and B02


#define BAUDRATE 9600
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 105
#define WHEEL_RADIUS_METERS 0.0349

#define rpm_to_radians 0.10471975512


//Motor A
#define PWMA 3  //Speed control
#define AIN1 8  //Direction
#define AIN2 9  //Direction
#define ENC_IN_RIGHT_A 2
volatile long motor_A_pulse_count = 0;

//Motor B
#define PWMB 5   //Speed control
#define BIN1 11  //Direction
#define BIN2 12  //Direction


// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
float rpm_motor_A = 0;

// Variable for angular velocity measurement
float ang_velocity_motor_A = 0;
float ang_velocity_motor_A_deg = 0;

void setup() {
    Serial.begin(BAUDRATE);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    // Set pin states of the encoder
    pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), motor_A_pulse, RISING);

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

    // Record the time
    currentMillis = millis();

    // If one second has passed, print the number of pulses
    if (currentMillis - previousMillis > interval) {

        previousMillis = currentMillis;

        // Calculate revolutions per minute
        rpm_motor_A = (float) (motor_A_pulse_count * 60 / ENC_COUNT_REV);
        ang_velocity_motor_A = rpm_motor_A * rpm_to_radians;

        Serial.print(" Pulses: ");
        Serial.println(motor_A_pulse_count);

        Serial.print(" RPM: ");
        Serial.print(rpm_motor_A);

        Serial.print(" Angular Velocity: ");
        Serial.print(ang_velocity_motor_A);
        Serial.print(" rad per second");

        Serial.print(" Linear Velocity: ");
        Serial.print(WHEEL_RADIUS_METERS * ang_velocity_motor_A);
        Serial.print(" meters per second");

        motor_A_pulse_count = 0;

    }
}

void motor_A_pulse() {
    motor_A_pulse_count++;
}
