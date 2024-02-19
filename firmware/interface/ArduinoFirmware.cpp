//
// Created by omar on 2/8/24.
//


#include <iostream>
#include "ArduinoFirmware.h"

void ArduinoFirmware::configure() {
    serial = make_unique<SerialOps>(SERIAL_PORT, BAUD);
}

double ArduinoFirmware::getFirstMotorPosition() {
    return readCommand(GET_MOTOR_1_POSITION);
}

double ArduinoFirmware::getFirstMotorVelocity() {
    return readCommand(GET_MOTOR_1_VELOCITY);
}

void ArduinoFirmware::setFirstMotorVelocity(double v) {
    string command = std::regex_replace(WRITE_COMMAND_TEMPLATE,
                                        std::regex("#command"),
                                        to_string(MOVE_MOTOR_1));
    command = std::regex_replace(command,
                                 std::regex("#velocity"),
                                 to_string(v));
    writeCommand(command);
}


void ArduinoFirmware::writeCommand(const string &command) {
    serial->write(command);
}

double ArduinoFirmware::readCommand(int commandNumber) {
    const string command = regex_replace(READ_COMMAND_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));
    writeCommand(command);
    return readCommandUtil();
}

double ArduinoFirmware::readCommandUtil() {
    const string buffer = serial->read();
    cout << "*************************** buffer" << buffer << endl;
    if (buffer.empty()) {
        throw runtime_error("************************ SerialBufferEmpty");
    }
    const auto data = json::parse(buffer);
    if (data.is_null()) {
        throw runtime_error("************************ SerialReadNull");
    }
    const auto &value = data["value"];
    if (value.is_null() || !value.is_number()) {
        throw runtime_error("************************ SerialInvalidValue");
    }
    return value.get<nlohmann::json::number_float_t>();
}
