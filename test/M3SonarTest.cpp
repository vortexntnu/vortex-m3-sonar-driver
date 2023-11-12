#include <gtest/gtest.h>
#include <imb/M3Listener.hpp>

TEST(M3SonarTest, TcpConnection)
{
    int clientSocket;
    struct sockaddr_in serverAddr;
    char buffer[1024];
    //auto a = argv[0];
    // printf(a);
    // Create a socket
    
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        // return 1;
    }

    // Set up server address information
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(20001U); // Change this to your server's port
    serverAddr.sin_addr.s_addr = inet_addr("10.0.0.205"); // Change this to your server's IP address
    // Connect to the server
    int test = connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    std::cout << "dwdr" << std::endl;
    if (test == -1) {
        std::cerr << "Error connecting to the server" << std::endl;
        close(clientSocket);
        // return 1;, M3PclPublisher publisher
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
            
            std::cout << std::hex << buffer << std::endl;
        }
    }

    // Close the socket
    close(clientSocket);

    // return 0;
}