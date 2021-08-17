#include "slip_detector/slip_detector.h"
#include "slip_detector/event_cnt_detector.h"

namespace celex_ros
{

EventCntSlipDetector::EventCntSlipDetector()
{
    detector_name="event_cnt";
    // 用size和直接输入，是相反的Size(宽 高)
    // Size(col, row)
    mat_ROI_ = cv::Mat::zeros(cv::Size(ROI_area_[2], ROI_area_[3]), CV_8UC1);
    std::cout<<mat_ROI_.size();
    // mat_half_ = cv::Mat::zeros(cv::Size(MAT_COLS/2, MAT_ROWS/2), CV_8UC1);
}

EventCntSlipDetector::~EventCntSlipDetector(){ROS_INFO("--end EventCntSlipDetector--");}

bool EventCntSlipDetector::isSlipped()
{
    if(!isSlipEventSizeROI())return false;
    // ROS_INFO("more--");
    // 线检测很费时间，不推荐
    // if(!isLineDetected(mat_ROI_))return false;
    if(!isCornerDetected(mat_ROI_))return false;
    return true;
}


bool EventCntSlipDetector::grabEventDataSizeROI(CeleX5 *celex,celex5_msgs_sdk::EventVector &msg)
{
    if (celex->getSensorFixedMode() == CeleX5::Event_Off_Pixel_Timestamp_Mode)
    {
        std::vector<EventData> vecEvent;
        celex->getEventDataVector(vecEvent);
        if(off_time_zero_==-99 && vecEvent[0].tOffPixelIncreasing != 0)
        {
            auto zero_time =std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            off_time_zero_= static_cast<uint64_t>(zero_time) -  vecEvent[0].tOffPixelIncreasing;
            // ROS_INFO("2 vec is %ld us",vecEvent[0].tOffPixelIncreasing);
            ROS_INFO("off_time_zero is %f s",off_time_zero_/1000000.0);
        }
        // cur_off_time_from_zero_ = off_time_zero_+vecEvent[0].tOffPixelIncreasing;
        // ROS_INFO("1 off_time_zero is %ld us",off_time_zero_);
        // ROS_INFO("2 vec is %ld us",vecEvent[0].tOffPixelIncreasing);
        // ROS_INFO("3 cur_off_time is %ld ns",cur_off_time_from_zero_);

        if(vecEvent[0].tOffPixelIncreasing<5000000)return false;

        int data_size = vecEvent.size();

        mat_ROI_ = cv::Mat::zeros(cv::Size(ROI_area_[2], ROI_area_[3]), CV_8UC1);
        
        int ROI_data_size = 0;
        for(int i = 0;i<data_size;++i)
        {
            if(ROI_area_[0]<=vecEvent[i].row && vecEvent[i].row < ROI_area_[0]+ROI_area_[3]-1)
            {
                if(ROI_area_[1]<=vecEvent[i].col && vecEvent[i].col < ROI_area_[1]+ROI_area_[2]-1)
                {
                    ROI_data_size++;
                    // mat_ROI_.at<uchar>(ROI_area_[3]-(vecEvent[i].row - ROI_area_[0])-1,ROI_area_[2]-(vecEvent[i].col - ROI_area_[1])-1) = 255;
                    // 下边这样和人眼看到的是一样的
                    mat_ROI_.at<uchar>(ROI_area_[3]-(vecEvent[i].row - ROI_area_[0])-1,(vecEvent[i].col - ROI_area_[1])) = 255;
                    // 下边的用不上，有这张图就够了，不需要把他包起来
                    // event_.y = vecEvent[i].row;
                    // event_.x = vecEvent[i].col;
                    // event_.brightness = 255;
                    // event_.off_pixel_timestamp = vecEvent[i].tOffPixelIncreasing;
                    // msg.events.push_back(event_);
                    // std::cout<<"row,col"<<vecEvent[i].row<<","<<vecEvent[i].col<<std::endl;
                }
            }
        }
        // cv::imshow("ROI_show",mat_ROI_);
        // cv::waitKey(1);
        // ROS_INFO("event cnt : %d",ROI_data_size);
        // 为了统计环境事件数量的均值
        if(isInitThresTest_)
        {
            event_cnt_sss++;
            event_total+=ROI_data_size;
            event_avg_num=event_total/event_cnt_sss;
            ROS_INFO("avg_event_num is : %d",event_avg_num);
        }

        // 其实这里是有问题的，这时的阈值无法代表前一状态
        // if(!updateEventWindow(data_size))return false;
        if(!updateEventWindow(ROI_data_size))return false;
        return true;
    }
    ROS_INFO("---------------------------");
    return false;
}

bool EventCntSlipDetector::isSlipEventSizeROI()
{
    if(!grabEventDataSizeROI(celex_,event_vector_))return false;
    // 为了测试环境阈值
    if(isInitThresTest_){isCornerDetected(mat_ROI_);}
    // 为了初始化更新角点阈值
    if(/*env_window_(0)!=0 &&*/ cor_window_(0)==0 )
    {
        isCornerDetected(mat_ROI_);
        // ROS_INFO("-----------------------%d",cor_threshold_);
        return false;
    }
    // 判断事件数量超出阈值了吗
    if (env_window_(envWindowSize-1) > dynamic_threshold_)
    {
        // ROS_INFO("%dthreshold is:%d",env_window_(envWindowSize-1),dynamic_threshold_);
        return true;
    }
    return false;
}


bool EventCntSlipDetector::initEventWindow()
{
    while(env_window_(0)==0)
    {
        grabEventDataSizeROI(celex_, event_vector_);
    }
    return true;
}

}