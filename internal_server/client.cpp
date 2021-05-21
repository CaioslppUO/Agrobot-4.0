#include <stdio.h>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

namespace Client {
    int BUFFER_SIZE = 10000;
    char IP[] = "localhost";
    char PORT[] = "3000";
    int sock = socket(AF_INET,SOCK_STREAM,0);
    struct sockaddr_in server_address;

    void connect() {
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(atoi(PORT));
        inet_pton(AF_INET, IP, &(server_address.sin_addr.s_addr));

        std::cout << "attempting to connect to server" << std::endl;

        int conn_success = connect(sock, (struct sockaddr*)&server_address, sizeof(server_address));

        if (conn_success < 0) {
            perror("ERROR connecting");
        } else {
            std::cout << "connection successful" << std::endl;
        }
    }

    void emit(char * msg) {
        int msg_size = strlen(msg),i,j;
        char aux[8];
        for(i=j=0; i < msg_size; i++,j++) { // Send 8 by 8 characters.
            aux[j] = msg[i];
            if(j == 7 || i+1 == msg_size) {
                send(sock, aux, sizeof(aux), 0);
                j = 0;
                aux[0] = 0; aux[1] = 0; aux[2] = 0; aux[3] = 0;
                aux[4] = 0; aux[5] = 0; aux[6] = 0; aux[7] = 0;
            }
        }
    }

    void receive(char * msg) {
        read(sock, msg, 10000);
        printf(msg);
    }
}