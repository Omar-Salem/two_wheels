#include "PIDController.h"


void PIDController::begin() {
    Kp = 1;
    Ki = 1;
    Kd = 1;
    divisor = 10;
    doLimit = false;
    init = true;
}

void PIDController::tune(double _Kp, double _Ki, double _Kd) {
    if (_Kp < 0 || _Ki < 0 || _Kd < 0) return;
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
}

void PIDController::limit(double min, double max) {
    minOut = min;
    maxOut = max;
    doLimit = true;
}


double PIDController::compute(double setPoint, double sensor) {
    // Return false if it could not execute;
    // This is the actual PID algorithm executed every loop();

    // Failsafe, return if the begin() method hasn't been called
    if (!init) return 0;

    // Calculate time difference since last time executed
    unsigned long now = millis();
    double timeChange = (double) (now - lastTime);

    // Calculate error (P, I and D)
    double error = setPoint - sensor;
    errSum += error * timeChange;
    if (doLimit) {
        errSum = constrain(errSum, minOut * 1.1, maxOut * 1.1);
    }
    double dErr = (error - lastErr) / timeChange;

    // Calculate the new output by adding all three elements together
    double newOutput = (Kp * error + Ki * errSum + Kd * dErr) / divisor;
    double output;
    // If limit is specifyed, limit the output
    if (doLimit) {
        output = constrain(newOutput, minOut, maxOut);
    } else {
        output = newOutput;
    }

    // Update lastErr and lastTime to current values for use in next execution
    lastErr = error;
    lastTime = now;

    // Return the current output
    return output;
}
