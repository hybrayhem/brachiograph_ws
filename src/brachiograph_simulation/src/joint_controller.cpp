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

 // TCP
int port;
std::string ip_address;
int sock = 0;
struct sockaddr_in serv_addr;
void handle_sigint(int);
// ROS
double deg2rad(double);
void publishJointPosition(ros::Publisher, double);


int main(int argc, char *argv[])
{
// ROS
    // initialize as node
    ros::init(argc, argv, "brachiograph_controller");

    // get parameters
    ros::NodeHandle nh_param("~");
    nh_param.getParam("port", port);
    nh_param.getParam("ip_address", ip_address);

// TCP
    std::cerr << "\nStarting at " << ip_address << ":" << port << std::endl;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if(inet_pton(AF_INET, ip_address.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address / Address not supported" << std::endl;
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        return -1;
    }

    signal(SIGINT, handle_sigint);

// ROS
    // create publisher node
    ros::NodeHandle nh;

    // create publisher
    ros::Publisher pub_joint1 = nh.advertise<std_msgs::Float64>("brachiograph/joint1_position_controller/command", 1000);
    ros::Publisher pub_joint2 = nh.advertise<std_msgs::Float64>("brachiograph/joint2_position_controller/command", 1000);
    ros::Publisher pub_joint3 = nh.advertise<std_msgs::Float64>("brachiograph/joint3_position_controller/command", 1000);

    // publish loop at 10Hz
    ros::Rate rate(10);
    PositionCommand command;
    
    while(ros::ok()) {
        ssize_t bytesRead = read(sock, &command, sizeof(command));
        if (bytesRead == -1) {
            std::cerr << "Error reading from socket" << std::endl;
            break;
        }

        std::cout << "Received PositionCommand: " 
                << command.joint1 << ", " 
                << command.joint2 << ", " 
                << command.joint3 << std::endl;
        
        publishJointPosition(pub_joint1, command.joint1);
        publishJointPosition(pub_joint2, command.joint2);
        publishJointPosition(pub_joint3, command.joint3);

        ros::spinOnce(); // let ROS process incoming messages
        // rate.sleep(); // sleep until the next cycle
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

// TCP
void handle_sigint(int sig) {
    close(sock);
    exit(0);
}