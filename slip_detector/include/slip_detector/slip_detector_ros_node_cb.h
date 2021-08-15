#pragma once

#include <ros/ros.h>
#include <unistd.h>
#include <signal.h>
#include <memory>
#include "slip_detector/slip_detector_ros_node.h"
#include "slip_detector/slip_detector.h"
#include "slip_detector/event_cnt_detector.h"
#include "slip_detector/of_direction_detector.h"
namespace celex_ros_cb{
class SensorDataObserver : public CeleX5DataManager
{

public:
    SensorDataObserver(CX5SensorDataServer *pServer,CeleX5* pCelex_);
    ~SensorDataObserver();
    virtual void onFrameDataUpdated(CeleX5ProcessedData *pSensorData);

private:
    /* data */
    CX5SensorDataServer *m_pServer;
    std::shared_ptr<celex_ros::SlipDetector> detector; 
    std::string celex_mode_;
};

// class SensorDataObserver : public CeleX5DataManager
// {
// public:

//     // parameters
//     std::string celex_mode_, event_pic_type_;
//     CeleX5 *celex_;

//     // overrides the update operation
//     virtual void onFrameDataUpdated(CeleX5ProcessedData *pSensorData);

//     // subscribe callback function
//     void celexDataCallback(const celex5_msgs_sdk::EventVector &msg);

//     void setCeleX5(CeleX5 *pcelex);
//     bool spin();
// };

// SensorDataObserver::SensorDataObserver(CX5SensorDataServer *pServer, CeleX5* pCelex_)
// {
//     m_pServer = pServer;
//     m_pServer->registerData(this, CeleX5DataManager::CeleX_Frame_Data);
    

//     ros::param::param<std::string>("~celex_mode",celex_mode_,"Event_Off_Pixel_Timestamp_Mode");

//     if(celex_mode_=="Event_Off_Pixel_Timestamp_Mode")
//         // detector = new celex_ros::EventCntSlipDetector;
//         detector = std::make_shared<celex_ros::EventCntSlipDetector>();
//     else if(celex_mode_=="Optical_Flow_Mode")
//         detector = std::make_shared<celex_ros::OFDirectionSlipDetector>();
//     else
//         ROS_ERROR("Error celex mode");

//     detector->setCeleX5(pCelex_);
//     ROS_INFO("starting......");
// }

// SensorDataObserver::~SensorDataObserver()
// {
//     m_pServer->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
//     // delete
// }
}