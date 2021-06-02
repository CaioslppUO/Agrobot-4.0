#ifndef CLIENT_CPP
#define CLIENT_CPP

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

// C++ client for communication between ROS python nodes and Agrobot core C++ program.
namespace Client {
    int BUFFER_SIZE = 10000; // MAX size of read/write characters.
    char IP[] = "localhost"; // LOCAL machine IP
    char PORT[] = "3000"; // LOCAL Port
    int sock = socket(AF_INET,SOCK_STREAM,0);
    struct sockaddr_in server_address;

    /* Connect to the local server if it is running.
     * Return 0 if connection is successfully and 1 if it is not.
     */
    int connect() {
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(atoi(PORT));
        inet_pton(AF_INET, IP, &(server_address.sin_addr.s_addr));

        int conn_success = connect(sock, (struct sockaddr*)&server_address, sizeof(server_address));

        if (conn_success < 0)
            return 1;
        return 0;
    }

    /* Send a message to the python server.
     * Throws buffer overflow if msg_size is bigger than BUFFER_SIZE.
    */
    void emit(char * msg) {
        int msg_size = strlen(msg),i,j;
        char aux[1];
        if(msg_size > BUFFER_SIZE) throw "buffer overflow";
        for(i = 0; i < msg_size; i++) {
            aux[0] = msg[i];
            send(sock, aux, sizeof(aux), 0);
        }
    }

    /* Receive a message from the python server.
     * The message must be smaller than BUFFER_SIZE.
    */
    char * receive(char * msg) {
        read(sock, msg, BUFFER_SIZE);
        return msg;
    }

    /* End the socket connection.
    */
    void closeConnection() {
        close(sock);
    }
}

#endif