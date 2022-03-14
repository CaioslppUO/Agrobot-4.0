#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for usleep
#include <iostream>

#include "bldc.h"
#include "motortypes.h"
#include <thread>
#include <cstring>
#include <string.h>
#include <boost/asio.hpp>

using namespace boost::asio;
using namespace std;


using ip::tcp;
using std::string;
using std::cout;
using std::endl;


void keepReceiving(BLDC motor, tcp::socket* sock) {
    boost::system::error_code errorCode;
    boost::asio::streambuf receive_buffer;
    while (true) {
        boost::asio::read(*sock, receive_buffer, boost::asio::transfer_all(), errorCode);
        motor.set_Current_Unscaled(stof(boost::asio::buffer_cast<const char*>(receive_buffer.data())));
    }
}



int main(int argc, char* argv[]) {
    // Initialize the Serial interface
    BLDC::init((char*)argv[argc - 1]);
    BLDC leftMotor(VESC1, motor1);

    boost::asio::io_service io_service;
    tcp::socket socketServer(io_service);
    socketServer.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 3000));

    std::thread read_t(keepReceiving, leftMotor, &socketServer);
    while (true)
        ;
    leftMotor.apply_Brake(3);
    BLDC::close();
    return 0;
}
