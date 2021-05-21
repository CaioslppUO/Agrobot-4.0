#include "internal_server/client.cpp"

using namespace std;

int main() {
    Client::connect();
    Client::emit("Mensagem de Teste.");
    char Teste[10000];
    Client::receive(Teste);
    cout << Teste << endl;
    return 0;
}