#include "slip_detector/slip_detector.h"
namespace celex_ros {

SlipDetector::SlipDetector() : node_("~")
{
    image_pub_ = node_.advertise<sensor_msgs::Image>("celex_image", 1);
    event_pub_ = node_.advertise<celex5_msgs_sdk::EventVector>("celex_event", 1);
    slip_pub_ = node_.advertise<std_msgs::String>("slip_signal", 1);
    // data_sub_ =
            // node_.subscribe("celex5_event", 1, &SlipDetectorRosNode::celexDataCallback, this);

    node_.param<std::string>("celex_mode", celex_mode_,
                             "Event_Off_Pixel_Timestamp_Mode");
    node_.param<int>("threshold", threshold_, 170);   // 0-1024
    node_.param<int>("clock_rate", clock_rate_, 100); // 0-100
    ROS_INFO("mode is %s", celex_mode_.c_str());

    mat_half_ = cv::Mat::zeros(cv::Size(MAT_COLS/2, MAT_ROWS/2), CV_8UC1);
    env_window_.setZero();
}

SlipDetector::~SlipDetector(){ROS_INFO("--end SlipDetector--");}

void SlipDetector::setCeleX5(CeleX5 *pcelex)
{
    celex_ = pcelex;
    celex_->setThreshold(threshold_);

    CeleX5::CeleX5Mode mode;
    if (celex_mode_ == "Event_Off_Pixel_Timestamp_Mode")
    {
        mode = CeleX5::Event_Off_Pixel_Timestamp_Mode;
        celex_->setSensorFixedMode(mode);
        celex_->setEventFrameTime(1000);
        celex_->disableFrameModule();
    }
    else if (celex_mode_ == "Optical_Flow_Mode")
    {
        mode = CeleX5::Optical_Flow_Mode;
        celex_->enableEventOpticalFlow();
        celex_->setSensorFixedMode(mode);
        // ROS_INFO("of time is %d",celex_->getOpticalFlowFrameTime());
        // 20 for default
        celex_->setOpticalFlowFrameTime(1);
    }
    else if (celex_mode_ == "Loop_Mode")
    {
        celex_->setLoopModeEnabled(true);
        celex_->setSensorLoopMode(CeleX5::Event_Off_Pixel_Timestamp_Mode,1);
        celex_->setSensorLoopMode(CeleX5::Optical_Flow_Mode,2);
        celex_->setSensorLoopMode(CeleX5::Event_Off_Pixel_Timestamp_Mode,3);
        celex_->setPictureNumber(1,CeleX5::Full_Picture_Mode);
        // celex_->setEventDuration(500); //us?
        celex_->setOpticalFlowFrameTime(1);
    }
    ROS_INFO("mode is:%s:",celex_mode_.c_str());


    ROS_INFO("clock rate:%d Hz",celex_->getClockRate());

    celex_->disableIMUModule();
    // if (mode==CeleX5::Event_Off_Pixel_Timestamp_Mode)
    // {
    //     // ROS_ERROR("disable module");
    // }




    ROS_INFO("work rate is:%d us", celex_->getEventFrameTime());
    ROS_INFO("work fixed mode is:%d",celex_->getSensorFixedMode());
    ROS_INFO("work loop1 mode is:%d",celex_->getSensorLoopMode(1));
    ROS_INFO("work loop2 mode is:%d",celex_->getSensorLoopMode(2));
    ROS_INFO("work loop3 mode is:%d",celex_->getSensorLoopMode(3));
}

void SlipDetector::slipPublish(std_msgs::String& msg)
{
    slip_pub_.publish(msg);
}

int SlipDetector::getEnvWindowNum(int num)
{
    return env_window_(num);
}

bool SlipDetector::isLineDetected()
{
    
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(mat_half_,lines,1.0,CV_PI/180,102);

    mat_half_ = cv::Mat::zeros(cv::Size(MAT_COLS/2, MAT_ROWS/2), CV_8UC1);
    // ROS_INFO("line size is:%d",lines.size());

    if(lines.size()>0)return true;
    return false;
}

bool SlipDetector::updateEventWindow(int data_size)
{
    if(data_size<50)return false;
    if(data_size==env_window_(9))return false;
    env_window_.topRows<10-1>()=env_window_.bottomRows<10-1>();
    env_window_(env_window_.size()-1)=data_size;
    dynamic_threshold_=(env_window_.sum()/env_window_.size())*dynamic_threshold_scale_;
    return true;
}



bool SlipDetector::run()
{
    isRunning=true;
    ros::Rate loop_rate(1000);
    initEventWindow();
        ROS_INFO("init done");


    while (node_.ok() && isRunning)
    {
        if (isSlipped())
        {
            ROS_INFO("SLIPPPPPPPPPPPPPPPPPPPPPPPPPING");
            std_msgs::String msggg;
            msggg.data = "SLIPPED";
            slipPublish(msggg);
        }
        std_msgs::String msggg;
        msggg.data="123123123";
        // slip_pub_.publish(msggg);

        // ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}

// void SlipDetector::run()
// {

// }


}
