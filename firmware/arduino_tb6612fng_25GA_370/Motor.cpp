//
// Created by omar on 1/24/24.
//
#include "Motor.h"

Motor::Motor(int encCountRev, int pwmPin, int firstBridgePin, int secondBridgePin,
             int encoderPin)
        : encCountRev(encCountRev),
          pwmPin(pwmPin),
          firstBridgePin(firstBridgePin),
          secondBridgePin(secondBridgePin),
          encoderPin(encoderPin) {}

void Motor::initialize() {
    pinMode(pwmPin, OUTPUT);
    pinMode(firstBridgePin, OUTPUT);
    pinMode(secondBridgePin, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);

    pid.begin();          // initialize the PID instance
    pid.limit(0, 255);
}

void Motor::odom() {
    float velocity2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        velocity2 = velocity_i;
    }
    //    // Low-pass filter (25 Hz cutoff)
    v2Filt = 0.854 * v2Filt + 0.0728 * velocity2 + 0.0728 * v2Prev;
    v2Prev = velocity2;

//    Serial.print(velocity2);
//    Serial.print(" ");
//    Serial.print(v2Filt);
//    Serial.println();

    rpm = v2Filt * 60 / encCountRev;
    angVelocity = rpm * RPM_TO_RADIANS;
}

double Motor::getAngularVelocity() {
    return angVelocity;
}

void Motor::move(double targetVelocity) {
    pid.tune(Kp, Ki, Kd);    // Tune the PID, arguments: kP, kI, kD
    pid.setpoint(targetVelocity);    // The "goal" the PID controller tries to "reach"
    int pwm = pid.compute(getAngularVelocity());
    movePWM(pwm);
}

void Motor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
    setDirectionForward();

    odom();
}

void Motor::interruptCallback() {
    // Compute velocity with method 2
    long currT = micros();
    float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
    velocity_i = 1 / deltaT;
    prevT_i = currT;
}

void Motor::setDirectionForward() {
    digitalWrite(firstBridgePin, HIGH);
    digitalWrite(secondBridgePin, LOW);
}

int Motor::getEncoderPin() {
    return encoderPin;
}

int Motor::getRPM() {
    return rpm;
}
