#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <arpa/inet.h>

struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};

int PORT = 8080;
int serverSocket;
void handle_sigint(int);

int main() {
    // Signal handlers
    signal(SIGINT, handle_sigint);

    if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        return -1;
    }

    if (listen(serverSocket, 1) < 0) {
        std::cerr << "Error listening on socket" << std::endl;
        return -1;
    }

    while (true) {
        int clientSocket = accept(serverSocket, NULL, NULL);
        if (clientSocket < 0) {
            std::cerr << "Accepting client failed, retrying..." << std::endl;
            return -1;
        }

        // Print the server address for debug
        char ipstr[INET_ADDRSTRLEN];
        int port = ntohs(serverAddress.sin_port);
        inet_ntop(AF_INET, &serverAddress.sin_addr, ipstr, INET6_ADDRSTRLEN);
        std::cout << "Server running at: " << ipstr << ":" << port << std::endl;

        while (true) {
            PositionCommand positionCommand;

            ssize_t bytesRead = recv(clientSocket, &positionCommand, sizeof(positionCommand), 0);
            if (bytesRead == 0) {
                std::cout << "Client disconnected" << std::endl;
                break;
            } else if (bytesRead == -1) {
                std::cerr << "Error reading from client" << std::endl;
                break;
            }

            std::cout << "Received PositionCommand: " 
                    << positionCommand.joint1 << ", " 
                    << positionCommand.joint2 << ", " 
                    << positionCommand.joint3 << std::endl;
        }
        
        close(clientSocket);
    }

    return 0;
}

void handle_sigint(int sig) {
    close(serverSocket);
    exit(0);
}