// #include "celex5_ros.h"
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
            // detector = new celex_ros::EventCntSlipDetector;
            detector = std::make_shared<celex_ros::EventCntSlipDetector>();
        else if (celex_mode_ == "Optical_Flow_Mode")
            detector = std::make_shared<celex_ros::OFDirectionSlipDetector>();
        else if (celex_mode_=="Loop_Mode")
            detector=std::make_shared<celex_ros::LoopSlipDetector>();
        else
            ROS_ERROR("Error celex mode");

        detector->setCeleX5(pCelex_);
        ROS_INFO("starting......");
    }

    SensorDataObserver::~SensorDataObserver()
    {
        m_pServer->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
        // delete
    }

    void SensorDataObserver::onFrameDataUpdated(
        CeleX5ProcessedData *pSensorData)
    {
        // detector->initEventWindow();
        // ROS_INFO("update");


        if (detector->isSlipped())
        {
            // for init
            if(0 != detector->getEnvWindowNum(0))
            {
                ROS_INFO("SLIPPPPPPPPPPPPPPPPPPPPPPPPPING");
                std_msgs::String msggg;
                msggg.data = "SLIPPED";
                detector->slipPublish(msggg);
            }
        }
        std_msgs::String msggg;
        msggg.data = "123123123";
        // detector->slipPublish(msggg);
        // 这里还差一部判断并pub
        // celexRos_.grabEventData(celex_, event_vector_);
        // data_pub_.publish(event_vector_);
        // event_vector_.events.clear();

        // get sensor image and publish it, you can use the RVIZ to subscribe the topic "/imgshow"
        // cv::Mat image =
        //     cv::Mat(800, 1280, CV_8UC1,
        //             pSensorData->getEventPicBuffer(CeleX5::EventBinaryPic));
        // sensor_msgs::ImagePtr msg =
        //     cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        // image_pub_.publish(msg);
    }

    // bool SensorDataObserver::spin()
    // {
    //     ros::Rate loop_rate(60);
    //     while (node_.ok())
    //     {
    //         ros::spinOnce();
    //         loop_rate.sleep();
    //     }
    //     return true;
    // }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slip_detector_cb");

    CeleX5 *pCelex_;
    pCelex_ = new CeleX5;
    if (NULL == pCelex_)
        return 0;

    if (!pCelex_->openSensor(CeleX5::CeleX5_MIPI))
    {
        ROS_ERROR("open sensor failed!");
    }

    celex_ros_cb::SensorDataObserver *pSensorData = new celex_ros_cb::SensorDataObserver(pCelex_->getSensorDataServer(), pCelex_);

    // pSensorData->spin()

    // while(true)
    // {
    //     usleep(100);//us
    // }
    ros::spin();

    return EXIT_SUCCESS;
}
