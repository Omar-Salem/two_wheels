#include "PIDController.h"


void PIDController::tune(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

double PIDController::compute(double current, double target) {
    if (lastTarget != target) {
        lastTarget = target;
        outputSum = 0;
    }
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
//    if (timeChange >= SampleTime) {
    /*Compute all the working error variables*/
    double error = target - current;
//    Serial.print("error:");
//    Serial.println(error);
    double dInput = (current - lastTarget);
    outputSum += (Ki_ * error);

    outputSum -= Kp_ * dInput;

    if (outputSum > outMax) {
        outputSum = outMax;
    } else if (outputSum < outMin) {
        outputSum = outMin;
    }

    /*Add Proportional on Error, if P_ON_E is specified*/
    double output = 0;
//        if (pOnE) output = kp * error;

    /*Compute Rest of PID Output*/
    output += outputSum - Kd_ * dInput;

    if (output > outMax) {
        outputSum -= output - outMax; // backcalculate integral to feasability
        output = outMax;
    } else if (output < outMin) {
        outputSum += outMin - output; // backcalculate integral to feasability
        output = outMin;
    }

    lastTime = now;
    return output;
}