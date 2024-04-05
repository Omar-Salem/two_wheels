//
// Created by omar on 1/20/24.
//


#include <string>

class Wheel {
public:
    double position_state;
    double velocity_state;
    double velocity_command;
    std::string name;

    explicit Wheel(const std::string &name) : name(name) {}
};
