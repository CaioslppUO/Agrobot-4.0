#include "client.h"
#include <thread>
#include <string.h>

using namespace std;

void keepReceiving()
{
    while (true)
    {
        cout << "Mensagem recebida: " << Client::receive() << endl;
    }
}

int main()
{
    Client::connect();
    char aux[10000];
    string msg2;
    std::thread read_t(keepReceiving);
    try
    {
        while (true)
        {
            cout << "digite: " << endl;
            cin >> aux;
            Client::emit(aux);
        }
    }
    catch (exception e)
    {
        Client::closeConnection();
        read_t.join();
        cout << "except" << endl;
    }
    return 0;
}
