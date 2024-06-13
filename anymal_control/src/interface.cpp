#include <ros/ros.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/console.h>

class Msg_Interface
{
public:
    Msg_Interface(ros::NodeHandle &n)
    {
        // ros::Publisher joint_pub = n.advertise<control_msgs::FollowJointTrajectoryAction>("/joint_group_position_controller/command", 100);
        ros::Subscriber sub = n.subscribe("/xpp/joint_hyq_des", 10, &Msg_Interface::msg_callback, this);
        }
    void msg_callback(const xpp_msgs::RobotStateJoint::ConstPtr &msg)
    {
        ROS_DEBUG("Hello %s", "World");
        sensor_msgs::JointState joints = msg->joint_state;
        std::cout << joints.position[0] << std::endl;
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface");
    ros::NodeHandle n;
    Msg_Interface msg_node = Msg_Interface(n);
    ros::spin();
    return 0;
}
