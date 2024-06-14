#include <ros/ros.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<control_msgs ::FollowJointTrajectoryAction> TrajClient;
class Msg_Interface
{
private:
    TrajClient *traj_client_;
    std::vector<sensor_msgs::JointState> joint_trajectory;
    ros::Subscriber sub;

public:
    Msg_Interface(ros::NodeHandle *n)
    {
        // ros::Publisher joint_pub = n.advertise<control_msgs::FollowJointTrajectoryAction>("/joint_group_position_controller/command", 100);
        sub = n->subscribe("/xpp/joint_hyq_des", 10, &Msg_Interface::msg_callback, this);
        // change the below topic
        traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
        while (!traj_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }
    ~Msg_Interface()
    {
        delete traj_client_;
    }
    void msg_callback(const xpp_msgs::RobotStateJoint &msg)
    {
        sensor_msgs::JointState joints = msg.joint_state;
        // if (!joints.name.empty())
        // {
        //     std::cout << joints.name[0] << std::endl;
        // }
        // else
        // {
        //     ROS_WARN("Joint names are empty.");
        // }
        joint_trajectory.push_back(joints);
    }
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
        goal.trajectory.header.stamp = ros::Time::now();
        traj_client_->sendGoal(goal);
    }
    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("LF_HAA");
        goal.trajectory.joint_names.push_back("LF_HFE");
        goal.trajectory.joint_names.push_back("LF_KFE");

        goal.trajectory.joint_names.push_back("RF_HAA");
        goal.trajectory.joint_names.push_back("RF_HFE");
        goal.trajectory.joint_names.push_back("RF_KFE");

        goal.trajectory.joint_names.push_back("LH_HAA");
        goal.trajectory.joint_names.push_back("LH_HFE");
        goal.trajectory.joint_names.push_back("LH_KFE");

        goal.trajectory.joint_names.push_back("RH_HAA");
        goal.trajectory.joint_names.push_back("RH_HFE");
        goal.trajectory.joint_names.push_back("RH_KFE");

        goal.trajectory.points.resize(joint_trajectory.size());
    }

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface");
    ros::NodeHandle n;
    Msg_Interface msg_node = Msg_Interface(&n);
    ros::spin();
}
