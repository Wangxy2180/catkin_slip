#pragma once

// #include <ros/ros.h>
// #include <celex5/celex5.h>
// #include <celex5/celex5datamanager.h>
// #include <celextypes.h>
// #include <sensor_msgs/Image.h>
// // #include <opencv2/highgui/highgui.hpp>
// // #include <opencv2/imgproc/imgproc.hpp>
// #include <celex5_msgs/Event.h>
// #include <celex5_msgs/EventVector.h>
// #include <cv_bridge/cv_bridge.h>
// #include<std_msgs/String.h>

// #include <Eigen/Core>

// #include "slip_detector/slip_detector.h"


// namespace celex_ros
// {

// class EventCntSlipDetector : public SlipDetector
// {
// public:


//     // EventCntSlipDetector() : node_("~")
//     // {

//     //     image_pub_ = node_.advertise<sensor_msgs::Image>("/celex_image", 1);
//     //     data_pub_ = node_.advertise<celex5_msgs::EventVector>("celex5_event", 1);
//     //     test_pub_ = node_.advertise<std_msgs::String>("test_pub", 1);
//     //     // data_sub_ =
//     //         // node_.subscribe("celex5_event", 1, &SlipDetectorRosNode::celexDataCallback, this);

//     //     // grab the parameters
//     //     node_.param<std::string>("celex_mode", celex_mode_,
//     //                                 "Event_Off_Pixel_Timestamp_Mode");
//     //     node_.param<int>("threshold", threshold_, 170);   // 0-1024
//     //     node_.param<int>("clock_rate", clock_rate_, 100); // 0-100
//     //     ROS_INFO("topic is %s",celex_mode_.c_str());

//     //     env_noise.setZero();
//     // }
//     EventCntSlipDetector();

//     ~EventCntSlipDetector();

// // 纯虚函数
//     bool isSlipped();
//     bool initEventWindow();

//     // subscribe callback function
//     // 现在直接调用SDK，用不上了，先注释掉
//     // void celexDataCallback(const celex5_msgs::EventVector &msg);
//     bool getDataFromCeleX();
//     // void setCeleX5(CeleX5 *pcelex);
//     // bool run();
//     bool isSlipEventSize();
//     bool isSlipEventLen();
//     bool grabEventData(CeleX5 *celex, celex5_msgs_sdk::EventVector &msg);
//     bool grabEventDataSize(CeleX5 *celex);
//     bool updateEventWindow(int data_size);

// private:
//         // private ROS node handle
//     // ros::NodeHandle node_;
//     // custom celex5 message type
//     // celex5_msgs::EventVector event_vector_;
//     // ros::Publisher data_pub_, image_pub_;
//     // ros::Publisher test_pub_;
//     // ros::Subscriber data_sub_;
//     // parameters
//     // std::string celex_mode_;
//     // int threshold_, clock_rate_;
//     // int dynamic_threshold_=-1;
//     // float dynamic_threshold_scale_=3.0;

//     // CelexRos celexRos_;
//     // CeleX5 *celex_;
//     // Eigen::Array<int, 10,1> env_noise;

//     // protected:
//     // CeleX5 *celex_;

// };

// }