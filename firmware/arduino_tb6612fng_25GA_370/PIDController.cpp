#include "PIDController.h"

void PIDController::init(double Kp, double Ki, double Kd, double min, double max) {
    pid_ = new PID(&current_, &output_, &target_, Kp, Ki, Kd, DIRECT);
    pid_->SetOutputLimits(min, max);
}


double PIDController::compute(double current, double target) {
    current_ = current;
    target_ = target;
    pid_->Compute();
    return output_;
}