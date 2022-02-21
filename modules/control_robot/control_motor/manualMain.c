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
float input;

int main(int argc, char* argv[]) {
    // Initialize the Serial interface
    BLDC::init((char*)"/dev/ttyACM0");
    BLDC leftMotor(VESC1, motor1);


    while (true) {
        std::cin >> input;
        leftMotor.set_Current_Unscaled(input);
    }
    leftMotor.apply_Brake(3);
    BLDC::close();
    return 0;
}
