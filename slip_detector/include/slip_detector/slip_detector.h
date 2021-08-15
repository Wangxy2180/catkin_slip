#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <celex5_msgs_sdk/Event.h>
#include <celex5_msgs_sdk/EventVector.h>

#define MAT_ROWS 800
#define MAT_COLS 1280
namespace celex_ros{
class SlipDetector
{
public:
    SlipDetector(/* args */);
    ~SlipDetector();

    void setCeleX5(CeleX5 *pcelex);
    bool run();
    void slipPublish(std_msgs::String& msg);
    int getEnvWindowNum(int num);
    bool isLineDetected();
    bool updateEventWindow(int data_size);


    virtual bool isSlipped()=0;
    virtual bool initEventWindow()=0;

private:
    bool isRunning=false;
    ros::Publisher event_pub_, image_pub_;
    // ros::Subscriber data_sub_;

    std::string celex_mode_;
    int threshold_, clock_rate_;


protected:
    ros::NodeHandle node_;

    ros::Publisher slip_pub_;
    int dynamic_threshold_ = -1;
    float dynamic_threshold_scale_ = 2.5;
    std::string detector_name;
    CeleX5 *celex_;
    Eigen::Array<int, 10, 1> env_window_;
    celex5_msgs_sdk::EventVector event_vector_;
    cv::Mat mat_half_;


};

// SlipDetector::SlipDetector(/* args */) : node_("~")
// {
//     image_pub_ = node_.advertise<sensor_msgs::Image>("celex_image", 1);
//     event_pub_ = node_.advertise<celex5_msgs_sdk::EventVector>("celex_event", 1);
//     slip_pub_ = node_.advertise<std_msgs::String>("slip_singal", 1);

//     node_.param<std::string>("celex_mode", celex_mode_,
//                              "Event_Off_Pixel_Timestamp_Mode");
//     node_.param<int>("threshold", threshold_, 170);   // 0-1024
//     node_.param<int>("clock_rate", clock_rate_, 100); // 0-100
//     ROS_INFO("topic is %s", celex_mode_.c_str());

//     env_noise_.setZero();
// }






}