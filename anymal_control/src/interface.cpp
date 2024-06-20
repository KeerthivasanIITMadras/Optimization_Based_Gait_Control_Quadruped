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
#include <xpp_msgs/RobotStateCartesian.h>
#include <gazebo_msgs/ContactsState.h>

using namespace std;
typedef actionlib::SimpleActionClient<control_msgs ::FollowJointTrajectoryAction> TrajClient;

class Msg_Interface
{
private:
    TrajClient *traj_client_;
    std::vector<sensor_msgs::JointState> joint_trajectory;
    std::vector<ros::Publisher> force_pub;
    ros::Subscriber sub;
    ros::Subscriber sub_forces;
    ros::Subscriber sub_gazebo_forces;
    ros::Publisher gazebo_force_pub;
    ros::Timer timer;
    ros::Duration timeout_duration;
    ros::Time prev_time;
    int message_count;
    bool message_received;

public:
    Msg_Interface(ros::NodeHandle *n)
    {
        for (int i = 0; i < 4; i++)
        {
            force_pub.push_back(n->advertise<geometry_msgs::Vector3>("force_at_contact_" + std::to_string(i + 1), 100));
        }
        force_pub.resize(4);
        gazebo_force_pub = n->advertise<geometry_msgs::Vector3>("force_from_gazebo_lf", 100);
        sub = n->subscribe("/xpp/joint_hyq_des", 10, &Msg_Interface::msg_callback, this);
        sub_forces = n->subscribe("/xpp/state_des", 10, &Msg_Interface::force_callback, this);
        sub_gazebo_forces = n->subscribe("/lf_force", 10, &Msg_Interface::gazebo_force_callback, this);
        // change the below topic
        traj_client_ = new TrajClient("/joint_group_position_controller/follow_joint_trajectory", true);
        while (!traj_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
        timer = n->createTimer(ros::Duration(2.0), &Msg_Interface::timerCallback, this);
        message_count = 0;
        message_received = false;
        timeout_duration = ros::Duration(2.0);
    }
    ~Msg_Interface()
    {
        delete traj_client_;
    }
    void gazebo_force_callback(const gazebo_msgs::ContactsState &msg)
    {
        string ground = "ground_plane::link::collision";
        if (msg.states.size() > 0 && ground.compare(msg.states[0].collision2_name) == 0)
        {
            geometry_msgs::Vector3 force = msg.states[0].total_wrench.force;
            gazebo_force_pub.publish(force);
        }
    }
    void msg_callback(const xpp_msgs::RobotStateJoint &msg)
    {
        message_received = true;
        sensor_msgs::JointState joints = msg.joint_state;
        joint_trajectory.push_back(joints);
        prev_time = ros::Time::now();
    }
    void force_callback(const xpp_msgs::RobotStateCartesian &msg)
    {
        for (int i = 0; i < 4; i++)
        {
            force_pub[i].publish(msg.ee_forces[i]);
        }
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

        goal.trajectory.points.resize(joint_trajectory.size() + 1);
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(12);
        goal.trajectory.points[ind].velocities.resize(12);
        for (size_t j = 0; j < 12; ++j)
        {
            goal.trajectory.points[ind].positions[j] = 0.0;
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        goal.trajectory.points[ind].time_from_start = ros::Duration(ind / 20.0);
        ind += 1;

        for (auto joint : joint_trajectory)

        {

            for (int i = 0; i < 12; i++)
            {
                goal.trajectory.points[ind].positions.resize(12);
                goal.trajectory.points[ind].positions[0] = joint.position[0];
                goal.trajectory.points[ind].positions[1] = joint.position[1];
                goal.trajectory.points[ind].positions[2] = joint.position[2];
                goal.trajectory.points[ind].positions[3] = joint.position[3];
                goal.trajectory.points[ind].positions[4] = joint.position[4];
                goal.trajectory.points[ind].positions[5] = joint.position[5];
                goal.trajectory.points[ind].positions[6] = joint.position[6];
                goal.trajectory.points[ind].positions[7] = joint.position[7];
                goal.trajectory.points[ind].positions[8] = joint.position[8];
                goal.trajectory.points[ind].positions[9] = joint.position[9];
                goal.trajectory.points[ind].positions[10] = joint.position[10];
                goal.trajectory.points[ind].positions[11] = joint.position[11];
            }
            goal.trajectory.points[ind].velocities.resize(12);
            for (size_t j = 0; j < 12; ++j)
            {
                goal.trajectory.points[ind].velocities[j] = 0.0;
            }
            goal.trajectory.points[ind].time_from_start = ros::Duration(ind / 20.0);
            ind += 1;
        }
        cout << joint_trajectory.size() + 1 << endl;
        return goal;
    }
    actionlib::SimpleClientGoalState getState()
    {
        return traj_client_->getState();
    }
    void timerCallback(const ros::TimerEvent &)
    {

        double delta_t = ros::Time::now().toSec() - prev_time.toSec();
        if (delta_t > timeout_duration.toSec() && message_count == 0 && message_received == true && joint_trajectory.size() > 50)
        {
            cout << "Started sending messages to the controller" << endl;
            control_msgs::FollowJointTrajectoryGoal goal = armExtensionTrajectory();
            startTrajectory(goal);
            message_count = 1;
        }
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface");
    ros::NodeHandle n;
    Msg_Interface msg_node = Msg_Interface(&n);
    ros::spin();
}
