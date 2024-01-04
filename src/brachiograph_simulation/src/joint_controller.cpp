// TCP
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <arpa/inet.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>

struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};
const struct PositionCommand homePosition = {-90, 90, 0};

 // TCP
int serverSocket;
void handle_sigint(int);
void printServerAddress(const sockaddr_in&);

// ROS
int port = 8080; // default = 8080
std::string ip_address; // default = any
double deg2rad(double);
void publishJointPosition(ros::Publisher, double);
void publishAllJointPositions(std::vector<ros::Publisher>, PositionCommand);


int main(int argc, char *argv[]) {
    // Signal handlers
    signal(SIGINT, handle_sigint);

// ROS
    // initialize as node
    ros::init(argc, argv, "brachiograph_controller");

    // get parameters
    ros::NodeHandle nh_param("~");
    nh_param.getParam("port", port);
    nh_param.getParam("ip_address", ip_address);

    std::cerr << "\nparams: " << ip_address << ", " << port << std::endl;

// TCP
    if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        exit(EXIT_FAILURE);
    }

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);

    if(inet_pton(AF_INET, ip_address.c_str(), &serverAddress.sin_addr) > 0) {
        std::cerr << "\nServer address = " << ip_address << ":" << port << std::endl;
    } else {
        std::cerr << "Setting ip adress to ANY" << std::endl;
        serverAddress.sin_addr.s_addr = INADDR_ANY;
    }

    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(serverSocket, 1) < 0) {
        std::cerr << "Error listening on socket" << std::endl;
        exit(EXIT_FAILURE);
    }

// ROS
    // create publisher node
    ros::NodeHandle nh;

    // create publisher
    ros::Publisher pub_joint1 = nh.advertise<std_msgs::Float64>("brachiograph/joint1_position_controller/command", 1000);
    ros::Publisher pub_joint2 = nh.advertise<std_msgs::Float64>("brachiograph/joint2_position_controller/command", 1000);
    ros::Publisher pub_joint3 = nh.advertise<std_msgs::Float64>("brachiograph/joint3_position_controller/command", 1000);
    std::vector<ros::Publisher> publishers = {pub_joint1, pub_joint2, pub_joint3};

    // publish loop at 10Hz
    ros::Rate rate(10);

    while (ros::ok()) {
// TCP
        int clientSocket = accept(serverSocket, NULL, NULL);
        if (clientSocket < 0) {
            std::cerr << "Accepting client failed, retrying..." << std::endl;
            exit(EXIT_FAILURE);
        }
        printServerAddress(serverAddress); // For debug

        while(ros::ok()) {
            PositionCommand command;
            
            ssize_t bytesRead = recv(clientSocket, &command, sizeof(command), 0);
            if (bytesRead == 0) {
                std::cout << "Client disconnected" << std::endl;
                break;
            } else if (bytesRead == -1) {
                std::cerr << "Error reading from client" << std::endl;
                break;
            }
// ROS
            std::cout << "Received PositionCommand: "
                    << command.joint1 << ", " 
                    << command.joint2 << ", " 
                    << command.joint3 << std::endl;
            
            publishAllJointPositions(publishers, command);

            ros::spinOnce(); // let ROS process incoming messages
            // rate.sleep(); // sleep until the next cycle
        }

        close(clientSocket);
    }

    if (!ros::ok()) {
        std::cerr << "ROS is not ok." << std::endl;
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

void publishAllJointPositions(std::vector<ros::Publisher> pubs, PositionCommand command) {
    publishJointPosition(pubs[0], command.joint1 - homePosition.joint1);
    publishJointPosition(pubs[1], command.joint2 - homePosition.joint2);

    double joint3 = command.joint3 - homePosition.joint3;

    joint3 > 1500 ? joint3 = 0 : joint3 = -60;
    publishJointPosition(pubs[2], joint3);
}

// TCP
void handle_sigint(int sig) {
    close(serverSocket);
    exit(0);
}

void printServerAddress(const sockaddr_in& serverAddress) {
    char ipstr[INET_ADDRSTRLEN];
    int port = ntohs(serverAddress.sin_port);
    inet_ntop(AF_INET, &serverAddress.sin_addr, ipstr, sizeof(ipstr));
    std::cout << "Server running at: " << ipstr << ":" << port << std::endl;
}