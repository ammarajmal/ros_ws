
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DSLRCamNode {
public:
    DSLRCamNode();
    void spin();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_image_;
    ros::Publisher pub_camera_info_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    std::string camera_frame_id_;

    void publishImage();
    void initializeCamera();
    cv::VideoCapture cap_;
};

DSLRCamNode::DSLRCamNode() : it_(nh_) {
    // Initialize camera
    initializeCamera();

    // Advertise image and camera info topics
    pub_image_ = it_.advertise("~image", 1);
    pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("~camera_info", 1);

    // Load camera info
    std::string camera_info_url;
    nh_.param<std::string>("camera_info_url", camera_info_url, "");
    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, "usb_cam", camera_info_url));
    camera_frame_id_ = "usb_cam";
}

void DSLRCamNode::initializeCamera() {
    // Open the default camera using OpenCV. Replace with your camera initialization code.
    cap_.open(0);
    if (!cap_.isOpened()) {
        ROS_ERROR("Could not open camera.");
        ros::shutdown();
    }
}

void DSLRCamNode::publishImage() {
    cv::Mat frame;
    if (cap_.read(frame)) { // Capture a frame
        // Convert the captured frame to a ROS image message
        cv_bridge::CvImage cv_image;
        cv_image.image = frame;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        ros_image.header.frame_id = camera_frame_id_;
        ros_image.header.stamp = ros::Time::now();

        // Publish the image
        pub_image_.publish(ros_image);

        // Publish the CameraInfo
        sensor_msgs::CameraInfo camera_info = camera_info_manager_->getCameraInfo();
        camera_info.header.frame_id = ros_image.header.frame_id;
        camera_info.header.stamp = ros_image.header.stamp;
        pub_camera_info_.publish(camera_info);
    } else {
        ROS_WARN("No frame captured.");
    }
}

void DSLRCamNode::spin() {
    ros::Rate loop_rate(60); // 30 Hz
    while (ros::ok()) {
        publishImage();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dslr_camera");
    DSLRCamNode node;
    node.spin();
    return 0;
}