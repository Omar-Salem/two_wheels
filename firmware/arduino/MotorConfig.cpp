//
// Created by omar on 2/4/24.
//

#include "MotorConfig.h"

MotorConfig::MotorConfig(int encCountRev,
                         double Kp,
                         double Ki,
                         double Kd) : encCountRev(encCountRev),
                                      Kp(Kp),
                                      Ki(Ki),
                                      Kd(Kd) {}

int MotorConfig::getEncCountRev() const { return encCountRev; }

double MotorConfig::getKp() const { return Kp; }

double MotorConfig::getKi() const { return Ki; }

double MotorConfig::getKd() const { return Kd; }