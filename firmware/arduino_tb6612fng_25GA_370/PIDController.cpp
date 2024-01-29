#include "PIDController.h"

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


double PIDController::compute(double goal, double current) {
    if (currentGoal != goal) { //goal changed, reset
        errSum = 0;
        lastTime = 0;
        lastErr = 0;
        currentGoal = goal;
    }
    unsigned long now = millis();
    double timeChange = (double) (now - lastTime);

    // Calculate error (P, I and D)
    double error = goal - current;
    errSum += error * timeChange;
    if (doLimit) {
        errSum = constrain(errSum, minOut * 1.1, maxOut * 1.1);
    }
    double dErr = (error - lastErr) / timeChange;

    // Calculate the new output by adding all three elements together
    double newOutput = (Kp * error + Ki * errSum + Kd * dErr) / divisor;
    double output = newOutput;
    // If limit is specifyed, limit the output
    if (doLimit) {
        output = constrain(newOutput, minOut, maxOut);
    }

    // Update lastErr and lastTime to current values for use in next execution
    lastErr = error;
    lastTime = now;

    // Return the current output
    return output;
}
