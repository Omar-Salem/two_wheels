//
// Created by omar on 1/24/24.
//
#include "Motor.h"

Motor::Motor(int encCountRev, double wheelRadiusMeters, int pwmPin, int firstBridgePin, int secondBridgePin,
             int encoderPin)
        : encCountRev(encCountRev),
          wheelRadiusMeters(wheelRadiusMeters),
          pwmPin(pwmPin),
          firstBridgePin(firstBridgePin),
          secondBridgePin(secondBridgePin),
          encoderPin(encoderPin) {}

void Motor::initialize() {
    pinMode(pwmPin, OUTPUT);
    pinMode(firstBridgePin, OUTPUT);
    pinMode(secondBridgePin, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);
    pid.begin();
    pid.tune(Kp, Ki, Kd);    // Tune the PID, arguments: kP, kI, kD
    pid.limit(0, 255);
    setDirectionForward();
}

void Motor::odom() {
    float velocity2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        velocity2 = velocity_i;
    }
    double rpm = velocity2 * 60 / encCountRev;
    angVelocity = rpm * RPM_TO_RADIANS;
}

double Motor::getLinearVelocity() {
    return wheelRadiusMeters * angVelocity;
}

void Motor::tune(double p, double i, double d) {
    pid.tune(p, i, d);
}

void Motor::move(double velocity) {
    pid.setpoint(velocity);
    auto pwm = pid.compute(this->getLinearVelocity());
    analogWrite(pwmPin, pwm);
}

void Motor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
}

void Motor::interruptCallback() {
    // Compute velocity with method 2
    long currT = micros();
    float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
    velocity_i = 1 / deltaT;
    prevT_i = currT;
}

void Motor::setDirectionForward() {
    digitalWrite(firstBridgePin, LOW);
    digitalWrite(secondBridgePin, HIGH);
}

int Motor::getEncoderPin() {
    return encoderPin;
}
