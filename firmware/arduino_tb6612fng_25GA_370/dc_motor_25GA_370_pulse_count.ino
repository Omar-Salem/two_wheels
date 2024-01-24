//#include <ArduinoJson.h>    //https://arduinojson.org/
//#include <ArduinoJson.hpp>
//
////https://automaticaddison.com/calculate-pulses-per-revolution-for-a-dc-motor-with-encoder/
////https://www.phippselectronics.com/using-the-dual-dc-stepper-motor-drive-tb6612fng-with-arduino/
///**https://mikroelectron.com/Product/25GA-370-12V-400RPM-DC-Reducer-Gear-Motor-with-Encoder
//Red Wire - positive power supply of motor(+)(change positive and negative of motor the rotation will change)
//
//White Wire - negative power supply of motor(-)(change positive and negative of motor the rotation will change))
//
//Blue Wire - positive of encoder power supply(+)(3.3-5V),cannot be wrong
//
//Black Wire - negative of encoder power supply(-)(3.3-5V),cannot be wrong
//
//Yellow Wire - signal feedback (motor one turn has 11 signals)
//
//Green Wire - signal feedback (motor one turn has 11 signals)
//
//**/
//
//
//#define BAUDRATE 9600
//#define ENC_IN_RIGHT_A 2
//volatile long right_wheel_pulse_count = 0;
//
//void setup() {
//    Serial.begin(BAUDRATE);
//    // Set pin states of the encoder
//    pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
//
//    // Every time the pin goes high, this is a pulse
//    attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
//}
//
//void loop() {
//    Serial.print(" Pulses: ");
//    Serial.println(right_wheel_pulse_count);
//}
//
//// Increment the number of pulses by 1
//void right_wheel_pulse() {
//    right_wheel_pulse_count++;
//}
