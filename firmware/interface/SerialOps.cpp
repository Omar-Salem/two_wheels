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

void SerialOps::write(const std::string &line) {
    exec("echo '{\"command\":5}' > /dev/ttyUSB0");
}

std::string SerialOps::read() {
    return exec("cat /dev/ttyUSB0");
}

std::string SerialOps::exec(const char *cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}
