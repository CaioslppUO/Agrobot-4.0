#ifndef CLIENT_H
#define CLIENT_H

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <bits/stdc++.h>

namespace Client {
  /* Connect to the local server if it is running.
   * Return 0 if connection is successfully and 1 if it is not.
   */
  int connect();

  int getNumberOfParameters(char* command);

  /* Send a message to the python server.
   * Throws buffer overflow if msg_size is bigger than BUFFER_SIZE.
  */
  void emit(char* msg);

  /* Receive a message from the python server.
   * The message must be smaller than BUFFER_SIZE.
  */
  std::string receive();

  /* End the socket connection.
  */
  void closeConnection();
}

#endif
