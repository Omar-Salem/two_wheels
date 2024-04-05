#ifndef MOTOR_h
#define MOTOR_h

class Motor
{

protected:
bool isLeft_;

public:
    Motor(bool isLeft);

    virtual void initialize(void (*interruptCallback)())=0;
    
    virtual void interruptCallback()=0;

    virtual double getAngularVelocity()=0;

    virtual double getPosition() const=0 ;

    virtual void move(double angularVelocity) = 0;
};
#endif