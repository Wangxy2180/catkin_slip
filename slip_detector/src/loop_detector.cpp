#include "slip_detector/loop_detector.h"

namespace celex_ros{

LoopSlipDetector::LoopSlipDetector()
{
    loop_of_buffer_ = new uint8_t[CELEX5_PIXELS_NUMBER];
	loop_bin_buffer_ = new uint8_t[CELEX5_PIXELS_NUMBER];
	// loop_buffer3 = new uint8_t[CELEX5_PIXELS_NUMBER];
}

LoopSlipDetector::~LoopSlipDetector()
{
    delete[] loop_bin_buffer_;
    delete[] loop_of_buffer_;
}

bool LoopSlipDetector::initEventWindow()
{
    while(env_window_(0)==0)
    {
        // int data_size=0;
        std::vector<EventData> vecEvent;
        celex_->getEventDataVector(vecEvent);
        updateEventWindow(vecEvent.size());
    }
}

bool LoopSlipDetector::getOFDirection(uint8_t* loop_of_buffer)
{
    // ROS_INFO("of");
    int value = 0;
    uint32_t dir_count1 = 0; //right
    uint32_t dir_count2 = 0; //down
    uint32_t dir_count3 = 0; //left
    uint32_t dir_count4 = 0; //up

    int direction = 0;
    for (int i = 0; i < CELEX5_ROW; ++i)
    {
        for (int j = 0; j < CELEX5_COL; ++j)
        {
            value = loop_of_buffer[i*CELEX5_COL+j];
            if (0 == value)
            {
                ;
            }
            //else if (value < 21 || value > 210) //30 300 red
            else if (value <= 32 || value > 223) //30 300 red
            {
                dir_count1++;
            }
            else if (value > 32 && value <= 96) //45 135 blue
            {
                dir_count2++;
            }
            else if (value > 96 && value <= 159) //135 225 green
            {
                dir_count3++;
            }
            else if (value > 159 && value <= 223) //225 315 yellow
            {
                dir_count4++;
            }
            else
            {
                ;
            }
        }
    }
    memset(loop_of_buffer,0,CELEX5_PIXELS_NUMBER);

    int dir1 = 0;
    int dir2 = 0;
    int max1 = 0;
    int max2 = 0;
    if (dir_count1 > dir_count2) //right
    {
        dir1 = 2;
        max1 = dir_count1;
    }
    else //down
    {
        dir1 = 4;
        max1 = dir_count2;
    }
    //
    if (dir_count3 > dir_count4) //left
    {
        dir2 = 1;
        max2 = dir_count3;
    }
    else //up
    {
        dir2 = 3;
        max2 = dir_count4;
    }
    //
    if (max1 > max2)
    {
        if (max1 > 5000)
        {
            direction = dir1;
            //cout << max1 << endl;
        }
        else
        {
            // m_iLastDirection = 0;
            direction = 0;
            // m_iDirectionCount = 0;
        }
    }
    else
    {
        if (max2 > 5000)
        {
            direction = dir2;
            //cout << max2 << endl;
        }
        else
        {
            // m_iLastDirection = 0;
            direction = 0;
            // m_iDirectionCount = 0;
        }
    }

    // 什么特么的情况，为啥是反的？
    // if(direction==4)
    // ROS_INFO("222");
    if(direction==3)
    {
        ROS_INFO("down!");
        return true;
    }


    return false;
}


bool LoopSlipDetector::grabLoopData()
{
    // ROS_INFO("grab1");
    std::vector<EventData> vecEvent;
    // ROS_INFO("grab999");
    celex_->getEventDataVector(vecEvent);
    // ROS_INFO("size:%d",vecEvent.size());
    updateEventWindow(vecEvent.size());
    // ROS_INFO("grab2");


    celex_->getOpticalFlowPicBuffer(loop_of_buffer_, CeleX5::OpticalFlowDirectionPic);

    
    celex_->getEventPicBuffer(loop_bin_buffer_, CeleX5::EventBinaryPic);
    cv::resize(cv::Mat(CELEX5_ROW,CELEX5_COL,CV_8UC1,loop_bin_buffer_),mat_half_,cv::Size(CELEX5_COL/2,CELEX5_ROW/2));
    memset(loop_bin_buffer_,0,CELEX5_PIXELS_NUMBER);
    // cv::imshow("123",mat_half_);
    // cv::waitKey(30);

    // bool ret=false;


    // if(vecEvent.size()<=dynamic_threshold_)return false;
    //     ROS_INFO("more");

    if(!getOFDirection(loop_of_buffer_))return false;
        ROS_INFO("down");

    if(!isLineDetected(mat_half_))return false;


    return true;
}

bool LoopSlipDetector::isSlipped()
{
    if(isLoopSlip())return true;
    return false;
}

bool LoopSlipDetector::isLoopSlip()
{
    if(grabLoopData())
    {
        // ROS_INFO("123");
    return true;
    }
    // ROS_INFO("456");
    return false;
}


}