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

struct JointDirection {
    int direction1;
    int direction2;
    int direction3;
};

const double safety_margin = 5; // degrees
const double joint1_upper_limit = 68.75 - safety_margin;      // 1.20 radians
const double joint2_upper_limit = 74.48 - safety_margin * 4;  // 1.30 radians
const double joint3_upper_limit = 0;                          // 0 radians
const double joint1_lower_limit = -180 + safety_margin * 14; // -3.14 radians
const double joint2_lower_limit = -51.5 + safety_margin;     // -0.90 radians
const double joint3_lower_limit = -63;                       // -1.10 radians (if collides with ground, set to -1.00)

int fifoFd;
const char* fifoPath = "/tmp/brachiograph_fifo";
void handle_sigint(int);
PositionCommand nextMessage(PositionCommand, JointDirection&);


int main() {
    mkfifo(fifoPath, 0666); // read/write for owner, group, others

    int fifoFd = open(fifoPath, O_WRONLY);
    if (fifoFd == -1) {
        std::cerr << "Error opening FIFO for writing" << std::endl;
        return 1;
    }

    signal(SIGINT, handle_sigint);

    PositionCommand message = {0, 0, 0};
    JointDirection direction = {1, 1, 1};
    while (true) {
        write(fifoFd, &message, sizeof(message));
        usleep(100000); // 100ms

        std::cout << "PositionCommand Sent: " 
            << message.joint1 << ", " 
            << message.joint2 << ", " 
            << message.joint3 << std::endl;

        message = nextMessage(message, direction);
    }

    return 0;
}


void handle_sigint(int sig) {
    close(fifoFd); // close
    unlink(fifoPath); // delete
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