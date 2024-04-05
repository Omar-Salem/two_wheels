//
// Created by omar on 2/4/24.
//

#ifndef TWO_WHEELS_MOTORCONFIG_H
#define TWO_WHEELS_MOTORCONFIG_H


class DCMotorConfig {
public:
    DCMotorConfig(int encCountRev,
                double Kp,
                double Ki,
                double Kd);

    int getEncCountRev() const;

    double getKp() const;

    double getKi() const;

    double getKd() const;

private:
    int encCountRev;
    double Kp;
    double Ki;
    double Kd;
};


#endif //TWO_WHEELS_MOTORCONFIG_H
