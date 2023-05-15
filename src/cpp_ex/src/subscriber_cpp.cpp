#include<ros/ros.h>
#include<std_msgs/String.h>
void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cpp_subscriber_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cpp_topic", 10, callback);
    ros::spin();
    return 0;
}