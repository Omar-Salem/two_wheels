//
// Created by omar on 2/4/24.
//

#include "DCMotorConfig.h"

DCMotorConfig::DCMotorConfig(int encCountRev,
                         double Kp,
                         double Ki,
                         double Kd) : encCountRev(encCountRev),
                                      Kp(Kp),
                                      Ki(Ki),
                                      Kd(Kd) {}

int DCMotorConfig::getEncCountRev() const { return encCountRev; }

double DCMotorConfig::getKp() const { return Kp; }

double DCMotorConfig::getKi() const { return Ki; }

double DCMotorConfig::getKd() const { return Kd; }