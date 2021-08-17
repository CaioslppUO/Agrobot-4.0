#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for usleep
#include <iostream>

#include "bldc.h"
#include "motortypes.h"
#include "client.h"
#include <thread>
#include <cstring>
#include <string.h>

using namespace std;

float direction = 1;

void keepReceiving(BLDC motor) {
    while (true) {
        motor.set_Duty(direction*stof(Client::receive()));
    }
}

int main(int argc, char* argv[]) {

    direction = atoi((char*)argv[argc - 2]);
    // Initialize the Serial interface
    BLDC::init((char*)argv[argc - 1]);
    BLDC leftMotor(VESC1, motor1);

    // Initialize server
    Client::connect();

    std::thread read_t(keepReceiving, leftMotor);
    while (true)
        ;
    leftMotor.apply_Brake(3);
    BLDC::close();
    return 0;
}
