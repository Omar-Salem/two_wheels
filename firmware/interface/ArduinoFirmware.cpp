//
// Created by omar on 2/8/24.
//


#include "ArduinoFirmware.h"

void ArduinoFirmware::configure() {
    auto errorOpening = serial.openDevice(SERIAL_PORT, BAUD);
    if (errorOpening != 1) {
        throw std::invalid_argument("SerialDeviceException: Could not open device");
    }
}

double ArduinoFirmware::getFirstMotorPosition() {
    serial.writeString("{\"command\":5}");
    char buffer[1000];
    serial.readString(buffer, '\n', 2000, 1000);
    printf("String read: %s\n", buffer);
    return 0;
}

double ArduinoFirmware::getFirstMotorVelocity() { return 0; }

void ArduinoFirmware::setFirstMotorVelocity(double v) {
    serial.writeString("{\"command\":1,\"params\":{\"velocity\":6.28}}");
}
