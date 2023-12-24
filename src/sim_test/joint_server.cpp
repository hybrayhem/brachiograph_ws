#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <arpa/inet.h>

// Structs
struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};

struct JointDirection {
    int direction1;
    int direction2;
    int direction3;
};

// Functions
void handle_sigint(int);
PositionCommand nextMessage(PositionCommand, JointDirection&);

// Constants
const double safety_margin = 5; // degrees
const double joint1_upper_limit = 68.75 - safety_margin;      // 1.20 radians
const double joint2_upper_limit = 74.48 - safety_margin * 4;  // 1.30 radians
const double joint3_upper_limit = 0;                          // 0 radians
const double joint1_lower_limit = -180 + safety_margin * 14; // -3.14 radians
const double joint2_lower_limit = -51.5 + safety_margin;     // -0.90 radians
const double joint3_lower_limit = -63;                       // -1.10 radians (if collides with ground, set to -1.00)

const int PORT = 8080;
int serverFd;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);

// Main
int main() {
    if ((serverFd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR /*| SO_REUSEPORT*/, &opt, sizeof(opt))) {
        std::cerr << "Setsockopt failed with error: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(serverFd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(serverFd, 3) < 0) {
        std::cerr << "Listen failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    int newSocket;
    if ((newSocket = accept(serverFd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        std::cerr << "Accept failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Print the server address
    char ipstr[INET6_ADDRSTRLEN];
    int port;
    if (address.sin_family == AF_INET) {
        struct sockaddr_in *s = (struct sockaddr_in *)&address;
        port = ntohs(s->sin_port);
        inet_ntop(AF_INET, &s->sin_addr, ipstr, sizeof ipstr);
    } else { // AF_INET6
        struct sockaddr_in6 *s = (struct sockaddr_in6 *)&address;
        port = ntohs(s->sin6_port);
        inet_ntop(AF_INET6, &s->sin6_addr, ipstr, sizeof ipstr);
    }
    std::cout << "Server address: " << ipstr << ":" << port << std::endl;


    signal(SIGINT, handle_sigint);

    PositionCommand message = {0, 0, 0};
    JointDirection direction = {1, 1, 1};
    while (true) {
        send(newSocket, &message, sizeof(message), 0);
        usleep(100000); // 100ms

        std::cout << "PositionCommand Sent: " 
            << message.joint1 << ", " 
            << message.joint2 << ", " 
            << message.joint3 << std::endl;

        message = nextMessage(message, direction);
    }

    return 0;
}

// Implementation
void handle_sigint(int sig) {
    close(serverFd);
    exit(0);
}

PositionCommand nextMessage(PositionCommand message, JointDirection& direction) {
    if (message.joint1 >= joint1_upper_limit) direction.direction1 = -1;
    if (message.joint2 >= joint2_upper_limit) direction.direction2 = -1;
    if (message.joint3 >= joint3_upper_limit) direction.direction3 = -1;

    if (message.joint1 <= joint1_lower_limit) direction.direction1 = 1;
    if (message.joint2 <= joint2_lower_limit) direction.direction2 = 1;
    if (message.joint3 <= joint3_lower_limit) direction.direction3 = 1;

    message.joint1 += (double) direction.direction1;
    message.joint2 += (double) direction.direction2;
    message.joint3 += (double) direction.direction3;

    return message;
}