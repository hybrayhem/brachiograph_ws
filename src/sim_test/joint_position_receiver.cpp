#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};

int fifoFd;
const char* fifoPath = "/tmp/brachiograph_fifo";
void handle_sigint(int);


int main() {
    int fifoFd = open(fifoPath, O_RDONLY);
    if (fifoFd == -1) {
        std::cerr << "Error opening FIFO for reading" << std::endl;
        return 1;
    }

    signal(SIGINT, handle_sigint);

    PositionCommand positionCommand;
    while (true) {
        ssize_t bytesRead = read(fifoFd, &positionCommand, sizeof(positionCommand));
        if (bytesRead == -1) {
            std::cerr << "Error reading from FIFO" << std::endl;
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
    close(fifoFd);
    exit(0);
}