#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <iostream>

using namespace std;

void callback(const gazebo_msgs::ContactsState &msg)
{
    cout << msg.states.size() << endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_contact_forces");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/gazebo/LF_FOOT", 10, callback);
    ros::spin();
}