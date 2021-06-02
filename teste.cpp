#include "internal_server/client.cpp"
#include "teste2.cpp"

using namespace std;

int main() {
    Client::connect();
    int i;
    try {
        while(true) {
            cin >> i;
            teste2::test();
        }
    }catch(exception e) {
        Client::closeConnection();
        cout << "except" << endl;
    }
    return 0;
}