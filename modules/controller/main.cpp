#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for usleep

#include "bldc.h"
#include "motortypes.h"
#include "client.cpp"
#include <thread>
#include <string.h>
#include <vector>


using namespace std;

vector<string> input;

vector<string> split(char *str,char *regex){
    vector<string> split;
    char *ptr;
    ptr = strtok(str, regex);   
    while (ptr != NULL){
        split.push_back(ptr);
        ptr = strtok (NULL, regex);  
    }  
    split.push_back(ptr);
    return split;  
}


void keepReceiving(){
    while (true){
        input = split(Client::receive(), ";");
    }
}

int main(void){
    // variables
    int reads = 0;
    bool loop = true;

    float val = 0;
    float pos = 0;

    // Initialize the Serial interface
    BLDC::init((char*)"/dev/ttyACM0");

    BLDC leftMotor(VESC1, motor1);
    Client::connect();
    char aux[10000];
    string msg2;
    std::thread read_t(keepReceiving);
    try{
        while (true){
            cout << "digite: " << endl;
            cin >> aux;
            Client::emit(aux);
            scanf("%f", &val);
            leftMotor.set_Duty(val);
            reads = 0;
        }
    } catch (exception e){
        Client::closeConnection();
        read_t.join();
        cout << "except" << endl;
    }
    leftMotor.apply_Brake(3);
    BLDC::close();
    return 0;

}
