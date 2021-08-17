#include <unistd.h>
#include <signal.h>
#include <memory>
#include "slip_detector/slip_detector_ros_node.h"
#include "slip_detector/slip_detector.h"
#include "slip_detector/event_cnt_detector.h"
#include "slip_detector/of_direction_detector.h"
#include "slip_detector/loop_detector.h"

#include "slip_detector/slip_detector_ros_node_cb.h"

static const std::string OPENCV_WINDOW = "Image window";
namespace celex_ros_cb
{

    SensorDataObserver::SensorDataObserver(CX5SensorDataServer *pServer, CeleX5 *pCelex_)
    {
        m_pServer = pServer;
        m_pServer->registerData(this, CeleX5DataManager::CeleX_Frame_Data);

        ros::param::param<std::string>("~celex_mode", celex_mode_, "Event_Off_Pixel_Timestamp_Mode");

        if (celex_mode_ == "Event_Off_Pixel_Timestamp_Mode")
            detector = std::make_shared<celex_ros::EventCntSlipDetector>();
        else if (celex_mode_ == "Optical_Flow_Mode")
            detector = std::make_shared<celex_ros::OFDirectionSlipDetector>();
        else if (celex_mode_ == "Loop_Mode")
            detector = std::make_shared<celex_ros::LoopSlipDetector>();
        else
            ROS_ERROR("Error celex mode");

        detector->setCeleX5(pCelex_);
        ROS_INFO("starting......");
        ROS_INFO("Mode is : %s", celex_mode_.c_str());
    }

    SensorDataObserver::~SensorDataObserver()
    {
        m_pServer->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
        // delete
    }

    void SensorDataObserver::onFrameDataUpdated(
        CeleX5ProcessedData *pSensorData)
    {
        // ROS_INFO("update");

        detector->timer_.tic();
        if (detector->isSlipped())
        {
            detector->set_slip_cnt(detector->get_slip_cnt() + 1);
            // for init
            if (0 != detector->getEnvWindowNum(0))
            {
                // std::cout<<detector->get_slip_cnt()<<std::endl;
                if (detector->get_slip_cnt() > detector->get_max_slip_cnt())
                    return;
                // 连续滑动=0
                std_msgs::String msggg;
                msggg.data = "SLIPPED";
                detector->slipPublish(msggg);

                auto elapsed = detector->timer_.toc();
                ROS_INFO("SLIPPPPPPPPPPPPPPPPPPPPPPPPPING:%fms", elapsed / 1000000.0);

                // auto now_time =static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                // auto elapsed_2 = -(now_time-detector->get_cur_off_time_from_zero());
                // ROS_INFO("SLIP %fms",elapsed_2/1000.0);
                // ROS_INFO("NOW  %ldus",now_time);
                // ROS_INFO("GET  %ldus",detector->get_cur_off_time_from_zero());
            }
        }
        else
            detector->set_slip_cnt(0);
        // detector->set_cur_off_time_from_zero(0);
        std_msgs::String msggg;
        msggg.data = "123123123";
        // detector->slipPublish(msggg);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slip_detector_cb");

    CeleX5 *pCelex_;
    pCelex_ = new CeleX5;
    // ROS_ERROR("here");
    if (NULL == pCelex_)
        return 0;

    if (!pCelex_->openSensor(CeleX5::CeleX5_MIPI))
    {
        ROS_ERROR("open sensor failed!");
    }

    // usleep(10000);

    celex_ros_cb::SensorDataObserver *pSensorData = new celex_ros_cb::SensorDataObserver(pCelex_->getSensorDataServer(), pCelex_);

    // pSensorData->spin()
    // while(true)
    // {
    //     usleep(100);//us
    // }
    ros::spin();

    return EXIT_SUCCESS;
}
