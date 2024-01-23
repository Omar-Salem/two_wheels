//
// Created by omar on 1/20/24.
//

#ifndef TWO_WHEELS_WHEEL_H
#define TWO_WHEELS_WHEEL_H
#include <string>

class Wheel {
  public:
    double position_state;
    double velocity_state;
    double velocity_command;
    std::string name;

    explicit Wheel(const std::string &name) : name(name) {}
};


#endif
