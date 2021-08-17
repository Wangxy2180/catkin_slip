#pragma once
#include <ros/ros.h>
#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <celextypes.h>
#include <sensor_msgs/Image.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <celex5_msgs/Event.h>
#include <celex5_msgs/EventVector.h>
#include <cv_bridge/cv_bridge.h>
#include<std_msgs/String.h>

#include <Eigen/Core>

#include "slip_detector/slip_detector.h"


namespace celex_ros
{

class EventCntSlipDetector : public SlipDetector
{
public:

    EventCntSlipDetector();
    ~EventCntSlipDetector();

    // 父类的纯虚函数
    bool isSlipped();
    bool initEventWindow();

    // 现在直接调用SDK，用不上了，先注释掉
    // void celexDataCallback(const celex5_msgs::EventVector &msg);
    bool isSlipEventSizeROI();
    bool grabEventDataSizeROI(CeleX5 *celex, celex5_msgs_sdk::EventVector &msg);

private:
    cv::Mat mat_ROI_;

};

}