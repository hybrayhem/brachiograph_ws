// TCP
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <arpa/inet.h>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

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
void publishJointPosition(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, double);

int main(int argc, char *argv[])
{
// ROS
    // initialize as node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("brachiograph_controller");

    // get parameters
    node->declare_parameter("port");
    node->declare_parameter("ip_address");
    node->get_parameter("port", port);
    node->get_parameter("ip_address", ip_address);

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
    // create publisher
    auto pub_joint1 = node->create_publisher<std_msgs::msg::Float64>("brachiograph/joint1_position_controller/command", 10);
    auto pub_joint2 = node->create_publisher<std_msgs::msg::Float64>("brachiograph/joint2_position_controller/command", 10);
    auto pub_joint3 = node->create_publisher<std_msgs::msg::Float64>("brachiograph/joint3_position_controller/command", 10);

    // publish loop at 10Hz
    rclcpp::Rate rate(10);
    PositionCommand command;

    while(rclcpp::ok()) {
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

        rclcpp::spin_some(node); // let ROS process incoming messages
        // rate.sleep(); // sleep until the next cycle
    }

    rclcpp::shutdown();
    return 0;
}

// ROS
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void publishJointPosition(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub, double degrees) {
    auto msg = std_msgs::msg::Float64();
    msg.data = deg2rad(degrees); // convert to radians
    pub->publish(msg);
}

// TCP
void handle_sigint(int sig) {
    close(sock);
    exit(0);
}