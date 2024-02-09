//
// Created by omar on 2/10/24.
//

#ifndef TWO_WHEELS_SERIALOPS_H
#define TWO_WHEELS_SERIALOPS_H

#include <string>

using namespace std;

class SerialOps {
public:
    SerialOps(std::string port, unsigned int baudRate);

    std::string read();

    void write(const std::string &line);

private:
    std::string exec(const char *cmd);

    std::string port;
    unsigned int baudRate;

};


#endif //TWO_WHEELS_SERIALOPS_H
