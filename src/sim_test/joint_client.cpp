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

// Constants
const double safety_margin = 5; // degrees
const double joint1_upper_limit = 68.75 - safety_margin;      // 1.20 radians
const double joint2_upper_limit = 74.48 - safety_margin * 4;  // 1.30 radians
const double joint3_upper_limit = 0;                          // 0 radians
const double joint1_lower_limit = -180 + safety_margin * 14; // -3.14 radians
const double joint2_lower_limit = -51.5 + safety_margin;     // -0.90 radians
const double joint3_lower_limit = -63;                       // -1.10 radians (if collides with ground, set to -1.00)

// Socket
int PORT = 8080;
char* IP_ADDRESS = "127.0.0.1"; // default values

int clientSocket;
void handle_sigint(int);
PositionCommand nextMessage(PositionCommand, JointDirection&);

// Main
int main(int argc, char *argv[]) {
    // Signal handlers
    signal(SIGINT, handle_sigint);

    // Command-line aarguments
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <ip_address> <port>\n" << std::endl;
    } else {
        IP_ADDRESS = argv[1];
        PORT = std::atoi(argv[2]);
    }
    std::cerr << "Connecting to " << IP_ADDRESS << ":" << PORT << std::endl;
    
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    
    // Construct serverAddress from IP_ADDRESS
    if(inet_pton(AF_INET, IP_ADDRESS, &serverAddress.sin_addr) <= 0) { 
        std::cerr << "Invalid address / Address not supported" << std::endl;
        return -1;
    }

    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cerr << "Error connecting to server" << std::endl;
        return 1;
    }

    PositionCommand message = {0, 0, 0};
    JointDirection direction = {1, 1, 1};
    while (true) {
        send(clientSocket, &message, sizeof(message), 0);
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
    close(clientSocket);
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