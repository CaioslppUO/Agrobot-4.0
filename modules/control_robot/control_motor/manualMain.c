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
int input;

void keepReceiving(BLDC motor) {
    while (true) {
        // motor.set_Duty(direction*stof(Client::receive()));
        // motor.set_Duty(stof(Client::receive()));
        motor.set_Current_Unscaled(stof(Client::receive()));
    }
}

int main(int argc, char* argv[]) {
    // Initialize the Serial interface
    BLDC::init((char*)"/dev/ttyUSB0");
    BLDC leftMotor(VESC1, motor3);

    
    while (true){
      std::cin >> input;
      motor.set_Current_Unscaled(intput);
    }
    leftMotor.apply_Brake(3);
    BLDC::close();
    return 0;
}
