// FIFO
#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <atomic>
#include <thread>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>

struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};

 // FIFO
int fifoFd;
const char* fifoPath = "/tmp/brachiograph_fifo";
std::thread reader;
std::atomic<bool> newDataAvailable(false);
void handle_sigint(int);
bool readPositionCommand(PositionCommand&);
void readThread();
// ROS
PositionCommand command;
double deg2rad(double);
void publishJointPosition(ros::Publisher, double);


int main(int argc, char **argv)
{
    std::cout << "Joint Controller starting..." << std::endl;

// FIFO
    int fifoFd = open(fifoPath, O_RDONLY | O_NONBLOCK);
    if (fifoFd == -1) {
        std::cerr << "Error opening FIFO for reading" << std::endl;
        return 1;
    }
    signal(SIGINT, handle_sigint);
    reader = std::thread(readThread);

// ROS
    // initialize as node
    ros::init(argc, argv, "brachiograph_controller");
    ros::NodeHandle nh;

    // create publisher
    ros::Publisher pub_joint1 = nh.advertise<std_msgs::Float64>("brachiograph/joint1_position_controller/command", 1000);
    ros::Publisher pub_joint2 = nh.advertise<std_msgs::Float64>("brachiograph/joint2_position_controller/command", 1000);
    ros::Publisher pub_joint3 = nh.advertise<std_msgs::Float64>("brachiograph/joint3_position_controller/command", 1000);

    std::cout << "Joint Controller started" << std::endl;
    // publish loop at 10Hz
    ros::Rate rate(10);
    // while(true) {
    while(ros::ok()) {
        std::cout << "Looping..." << std::endl;
        if (newDataAvailable.load()) {
            publishJointPosition(pub_joint1, command.joint1);
            publishJointPosition(pub_joint2, command.joint2);
            publishJointPosition(pub_joint3, command.joint3);
            newDataAvailable.store(false);
        }

        ros::spinOnce(); // let ROS process incoming messages
        rate.sleep(); // sleep until the next cycle
    }

    return 0;
}


// ROS
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void publishJointPosition(ros::Publisher pub, double degrees) {
    std_msgs::Float64 msg;
    msg.data = deg2rad(degrees); // convert to radians
    pub.publish(msg);
}

// FIFO
void readThread() {
    PositionCommand tempCommand;
    while (true) {
        if (readPositionCommand(tempCommand)) {
            command = tempCommand;
            newDataAvailable.store(true);
        }
    }
}

bool readPositionCommand(PositionCommand& command) {
    ssize_t bytesRead = read(fifoFd, &command, sizeof(command));
    if (bytesRead == -1) {
        std::cerr << "Error reading from FIFO" << std::endl;
        exit(1);
    }

    if (bytesRead == 0) return false;

    std::cout << "Received PositionCommand: " 
            << command.joint1 << ", " 
            << command.joint2 << ", " 
            << command.joint3 << std::endl;

    return true;
}

void handle_sigint(int sig) {
    reader.detach();
    close(fifoFd);
    exit(0);
}