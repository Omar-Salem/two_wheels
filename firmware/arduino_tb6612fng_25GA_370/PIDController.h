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
    double compute(double setPoint, double input);

    // Methods - void
    void begin();

    void tune(double _Kp, double _Ki, double _Kd);

    void limit(double min, double max);

private:

    // Variables - long
    unsigned long lastTime;

    // Variables - double
    double lastErr;
    double timeChanged;

    // Variables - double, error variables
    double error;
    double errSum;
    double dErr;

    // Variables - bool
    bool doLimit;
    bool init;

    // Variables - double - tuning
    double Kp;
    double Ki;
    double Kd;
    double divisor;
    double minOut;
    double maxOut;
};

#endif
