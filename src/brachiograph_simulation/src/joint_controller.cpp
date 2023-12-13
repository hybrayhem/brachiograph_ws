#include <ros/ros.h>
#include <std_msgs/Float64.h>

struct PositionCommand {
    double joint1;
    double joint2;
    double joint3;
};

double deg2rad(double);
void publishJointPosition(ros::Publisher, double);


int main(int argc, char **argv)
{
    PositionCommand command;

    // initialize as node
    ros::init(argc, argv, "brachiograph_controller");
    ros::NodeHandle nh;

    // create publisher
    ros::Publisher pub_joint1 = nh.advertise<std_msgs::Float64>("brachiograph/joint1_position_controller/command", 1000);
    ros::Publisher pub_joint2 = nh.advertise<std_msgs::Float64>("brachiograph/joint2_position_controller/command", 1000);
    ros::Publisher pub_joint3 = nh.advertise<std_msgs::Float64>("brachiograph/joint3_position_controller/command", 1000);

    // publish loop at 10Hz
    ros::Rate rate(10);
    while(ros::ok()) {
        // publishJointPosition(pub_joint1, command.joint1);
        // publishJointPosition(pub_joint2, command.joint2);
        // publishJointPosition(pub_joint3, command.joint3);
        publishJointPosition(pub_joint1, 60);
        publishJointPosition(pub_joint2, 60);
        publishJointPosition(pub_joint3, 60);

        ros::spinOnce(); // let ROS process incoming messages
        rate.sleep(); // sleep until the next cycle
    }

    return 0;
}


double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void publishJointPosition(ros::Publisher pub, double degrees) {
    std_msgs::Float64 msg;
    msg.data = deg2rad(degrees); // convert to radians
    pub.publish(msg);
}