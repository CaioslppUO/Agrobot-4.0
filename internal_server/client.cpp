#include "client.h"

// C++ client for communication between ROS python nodes and Agrobot core C++ program.
namespace Client {
    int BUFFER_SIZE = 10000; // MAX size of read/write characters.
    char IP[] = "localhost"; // LOCAL machine IP
    char PORT[] = "3000"; // LOCAL Port
    int sock = socket(AF_INET, SOCK_STREAM, 0);
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

    int getNumberOfParameters(char* command) {
        std::vector <std::string> tokens;
        std::stringstream aux(command);
        std::string aux2;
        while (getline(aux, aux2, ';')) { tokens.push_back(aux2); }
        return tokens.size();
    }

    /* Send a message to the python server.
     * Throws buffer overflow if msg_size is bigger than BUFFER_SIZE.
    */
    void emit(char* msg) {
        int msg_size = strlen(msg), i, j;
        char aux[1];
        if (msg_size > BUFFER_SIZE) throw "buffer overflow";
        send(sock, std::to_string(getNumberOfParameters(msg) + 2).c_str(), sizeof(aux), 0);
        send(sock, ";", sizeof(aux), 0);
        for (i = 0; i < msg_size; i++) {
            aux[0] = msg[i];
            send(sock, aux, sizeof(aux), 0);
        }
        send(sock, ";", sizeof(aux), 0);
        send(sock, "$", sizeof(aux), 0);
    }

    /* Receive a message from the python server.
     * The message must be smaller than BUFFER_SIZE.
    */
    std::string receive() {
        char aux[BUFFER_SIZE];
        read(sock, aux, BUFFER_SIZE);
        return std::string(aux);
    }

    /* End the socket connection.
    */
    void closeConnection() {
        close(sock);
    }
}

