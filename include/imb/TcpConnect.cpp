#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class M3SonarListener {
    
};
int main() {
    int clientSocket;
    struct sockaddr_in serverAddr;
    char buffer[1024];

    // Create a socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    // Set up server address information
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(200001); // Change this to your server's port
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Change this to your server's IP address

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to the server" << std::endl;
        close(clientSocket);
        return 1;
    }

    while (true) {
        // Receive data from the server
        int bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesRead == -1) {
            std::cerr << "Error receiving data from the server" << std::endl;
            break;
        } else if (bytesRead == 0) {
            // Connection closed by the server
            std::cout << "Server closed the connection" << std::endl;
            break;
        } else {
            buffer[bytesRead] = '\0';
            std::cout << "Received from server: " << buffer << std::endl;
        }
    }

    // Close the socket
    close(clientSocket);

    return 0;
}