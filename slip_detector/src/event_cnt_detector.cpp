#include "slip_detector/slip_detector.h"
#include "slip_detector/event_cnt_detector.h"

namespace celex_ros
{

EventCntSlipDetector::EventCntSlipDetector()
{
    detector_name="event_cnt";
    // mat_half_ = cv::Mat::zeros(cv::Size(MAT_COLS/2, MAT_ROWS/2), CV_8UC1);
}

EventCntSlipDetector::~EventCntSlipDetector(){ROS_INFO("--end EventCntSlipDetector--");}

bool EventCntSlipDetector::grabEventData(
    CeleX5 *celex,
    celex5_msgs_sdk::EventVector &msg)
{
    celex5_msgs_sdk::Event event_;
     if (celex->getSensorFixedMode() == CeleX5::Event_Off_Pixel_Timestamp_Mode)
    {
        std::vector<EventData> vecEvent;

        celex->getEventDataVector(vecEvent);

        int dataSize = vecEvent.size();
        // msg.vectorIndex = 0;
        msg.height = MAT_ROWS;
        msg.width = MAT_COLS;
        msg.vector_length = dataSize;
        if(!updateEventWindow(dataSize))return false;

        for (int i = 0; i < dataSize; i++)
        {
              mat_half_.at<uchar>((MAT_ROWS - vecEvent[i].row - 1)/2,
                            (MAT_COLS - vecEvent[i].col - 1)/2) = 255;
            event_.x = vecEvent[i].row;
            event_.y = vecEvent[i].col;
            event_.brightness = 255;
            event_.off_pixel_timestamp = vecEvent[i].tOffPixelIncreasing;
            msg.events.push_back(event_);
        }
        

    }
    else
    {
        msg.vector_length = 0;
        std::cout << "This mode has no event data. " << std::endl;
        return false;
    }

    return true;
}

bool EventCntSlipDetector::grabEventDataSize(CeleX5 *celex)
{
    if (celex->getSensorFixedMode() == CeleX5::Event_Off_Pixel_Timestamp_Mode)
    {
        std::vector<EventData> vecEvent;
        // ROS_INFO("123");
        celex->getEventDataVector(vecEvent);
        // ROS_INFO("456");
        int data_size = vecEvent.size();
        if(!updateEventWindow(data_size))return false;
        return true;
    }
    return false;
}

bool EventCntSlipDetector::getDataFromCeleX()
{
    grabEventData(celex_, event_vector_);

    // grabEventDataSize(celex_);

    // data_pub_.publish(event_vector_);
    event_vector_.events.clear();
    // get sensor image and publish it
    // cv::Mat image = celex_->getEventPicMat(CeleX5::EventBinaryPic);
    // sensor_msgs::ImagePtr msg =
    //     cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    // image_pub_.publish(msg);
    // return false;
    return true;
}

bool EventCntSlipDetector::isSlipped()
{
    // if(isSlipEventLen())
    if(isSlipEventSize())
    {
        return true;
    }
    return false;
}

bool EventCntSlipDetector::initEventWindow()
{
    while(env_window_(0)==0)
    {
        grabEventDataSize(celex_);
    }
    return true;
}

// bool EventCntSlipDetector::updateEventWindow(int data_size)
// {
//     if(data_size<50)return false;
//     if(data_size==env_window_(9))return false;
//     env_window_.topRows<10-1>()=env_window_.bottomRows<10-1>();
//     env_window_(env_window_.size()-1)=data_size;
//     dynamic_threshold_=(env_window_.sum()/env_window_.size())*dynamic_threshold_scale_;
//     return true;
// }



bool EventCntSlipDetector::isSlipEventLen()
{
    bool ret=false;
    // ROS_INFO("123");

    if(!grabEventData(celex_, event_vector_))
    {
        event_vector_.events.clear();
        return false;
    }
    // ROS_INFO("456");

    if (event_vector_.vector_length > dynamic_threshold_)
    {
        ret = true;
        ROS_INFO("more");
    }

    if(!isLineDetected())
    {
        ret = false;
    }

    event_vector_.events.clear();
    return ret;
}

bool EventCntSlipDetector::isSlipEventSize()
{
    // ROS_INFO("123");
    if(!grabEventDataSize(celex_))return false;
    // ROS_INFO("456");
    if (env_window_(9) > dynamic_threshold_)
    {
        return true;
    }
    return false;
}

}