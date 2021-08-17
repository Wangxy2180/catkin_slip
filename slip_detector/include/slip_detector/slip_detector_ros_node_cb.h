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
  
}