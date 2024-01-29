#ifndef PIDControllerLib
#define PIDControllerLib

#if (ARDUINO >= 100)
#include "Arduino.h"
#else

#include "WProgram.h"

#endif


class PIDController {
public:

    // Methods - double
    double compute(double goal, double current);

    void tune(double _Kp, double _Ki, double _Kd);

    void limit(double min, double max);

private:

    // Variables - long
    unsigned long lastTime;

    // Variables - double
    double lastErr;

    // Variables - double, error variables
    double errSum;

    // Variables - bool
    bool doLimit;

    // Variables - double - tuning
    double Kp;
    double Ki;
    double Kd;
    double divisor = 10;
    double minOut;
    double maxOut;
    double currentGoal;
};

#endif
