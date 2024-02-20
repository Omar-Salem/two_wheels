//
// Created by omar on 2/10/24.
//

#include "SerialOps.h"
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <utility>

SerialOps::SerialOps(string port, unsigned int baudRate) : port(std::move(port)), baudRate(baudRate) {

}

void SerialOps::write(const std::string &firmwareCommand) {
    const string cmd = "echo '" + firmwareCommand + "' > " + port;
    exec(cmd);
}

std::string SerialOps::read(const std::string &firmwareCommand) {
    // cat < /dev/ttyUSB0 && echo '{"command":0}' > /dev/ttyUSB0
    const std::string cmd = "cat < " + port + " && echo '" + firmwareCommand + "' > " + port;
    return exec(cmd);
}

std::string SerialOps::exec(const std::string &cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}
