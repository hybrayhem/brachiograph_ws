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

int sock = 0;
struct sockaddr_in serv_addr;
void handle_sigint(int);


int main() {
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8080);

    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address / Address not supported" << std::endl;
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        return -1;
    }

    signal(SIGINT, handle_sigint);

    PositionCommand positionCommand;
    while (true) {
        ssize_t bytesRead = read(sock, &positionCommand, sizeof(positionCommand));
        if (bytesRead == -1) {
            std::cerr << "Error reading from socket" << std::endl;
            break;
        }

        std::cout << "Received PositionCommand: " 
                  << positionCommand.joint1 << ", " 
                  << positionCommand.joint2 << ", " 
                  << positionCommand.joint3 << std::endl;
    }

    return 0;
}

void handle_sigint(int sig) {
    close(sock);
    exit(0);
}