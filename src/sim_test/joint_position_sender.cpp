#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>

struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};

int fifoFd;
const char* fifoPath = "/tmp/brachiograph_fifo";
void handle_sigint(int);
PositionCommand nextMessage(PositionCommand);


int main() {
    mkfifo(fifoPath, 0666); // read/write for owner, group, others

    int fifoFd = open(fifoPath, O_WRONLY);
    if (fifoFd == -1) {
        std::cerr << "Error opening FIFO for writing" << std::endl;
        return 1;
    }

    signal(SIGINT, handle_sigint);

    PositionCommand message = {0, 0, 0};
    while (true) {
        write(fifoFd, &message, sizeof(message));
        usleep(100000); // 100ms

        message = nextMessage(message);
    }

    return 0;
}


void handle_sigint(int sig) {
    close(fifoFd); // close
    unlink(fifoPath); // delete
    exit(0);
}

PositionCommand nextMessage(PositionCommand message) {
    message.joint1 += 1;
    message.joint2 += 1;
    message.joint3 += 1;

    if (message.joint1 > 360) message.joint1 = 0;
    if (message.joint2 > 360) message.joint2 = 0;
    if (message.joint3 > 360) message.joint3 = 0;

    return message;
}