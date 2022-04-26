#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for usleep
#include <iostream>

#include "bldc.h"
#include "motortypes.h"
#include <thread>
#include <cstring>
#include <string.h>

#include <mosquitto.h>

using namespace std;

BLDC* motor;

void on_connect(struct mosquitto* mosq, void* obj, int rc) {
    if (rc) {
        printf("Error with result code: %d\n", rc);
        exit(-1);
    }
    mosquitto_subscribe(mosq, NULL, "#", 0);
}

void on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg) {
    motor->set_Duty(stof((char*)msg->payload));
}

int main(int argc, char* argv[]) {
    // Initialize the Serial interface
    BLDC::init((char*)argv[argc - 1]);
    motor = new BLDC(VESC1, motor1);

    int rc, id = 12;

    mosquitto_lib_init();

    struct mosquitto* mosq;

    mosq = mosquitto_new(nullptr, true, &id);//id null para não ter problema nas várias instancias 
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if (rc) {
        printf("Could not connect to Broker with return code %d\n", rc);
        return -1;
    }

    mosquitto_loop_start(mosq);
    while (1)
        ;

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    motor->apply_Brake(3);
    BLDC::close();
    return 0;
}