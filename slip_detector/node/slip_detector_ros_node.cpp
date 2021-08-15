#include <unistd.h>
#include <signal.h>
#include <memory>
#include "slip_detector/slip_detector_ros_node.h"
#include "slip_detector/slip_detector.h"
#include "slip_detector/event_cnt_detector.h"
#include "slip_detector/of_direction_detector.h"
#include "slip_detector/loop_detector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slip_detector");
    if (!ros::ok())
        return 0;

    //创建一个celex
    CeleX5 *pCelex_;
    pCelex_ = new CeleX5;
    if (NULL == pCelex_)
        return 0;

    if(!pCelex_->openSensor(CeleX5::CeleX5_MIPI))
    {
        ROS_ERROR("open sensor failed!");
    }

    std::shared_ptr<celex_ros::SlipDetector> detector;
    // celex_ros::SlipDetector* detector;
    std::string celex_mode;
    // ros::param::get("celex_mode",celex_mode);
    ros::param::param<std::string>("~celex_mode",celex_mode,"Event_Off_Pixel_Timestamp_Mode");
    
    if(celex_mode=="Event_Off_Pixel_Timestamp_Mode")
        // detector = new celex_ros::EventCntSlipDetector;
        detector = std::make_shared<celex_ros::EventCntSlipDetector>();
    else if(celex_mode=="Optical_Flow_Mode")
        detector = std::make_shared<celex_ros::OFDirectionSlipDetector>();
    else if (celex_mode=="Loop_Mode")
        detector=std::make_shared<celex_ros::LoopSlipDetector>();
    else
        ROS_ERROR("Error celex mode");

    detector->setCeleX5(pCelex_);
    ROS_INFO("starting......");
    detector->run();

    ros::shutdown();
    delete pCelex_;
    ROS_INFO("shut down");
    return EXIT_SUCCESS;
}
