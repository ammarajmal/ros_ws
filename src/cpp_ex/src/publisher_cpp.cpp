#include<ros/ros.h>
#include<std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cpp_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("cpp_topic", 10);
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        std_msgs::String message;
        message.data = "Hello from C++ Publisher!";
        pub.publish(message);
        loop_rate.sleep();
    }
    return 0;
}