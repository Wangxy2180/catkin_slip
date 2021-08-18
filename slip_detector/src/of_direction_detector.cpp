#include "slip_detector/of_direction_detector.h"

namespace celex_ros{

OFDirectionSlipDetector::OFDirectionSlipDetector(/* args */)
{
    detector_name = "of_direction";
    OF_buffer=new uint8_t[CELEX5_PIXELS_NUMBER];
    memset(OF_buffer,0,CELEX5_PIXELS_NUMBER);
    // 因为没有角点阈值，所以为了让他有一个
    cor_window_(0)=5;
}

OFDirectionSlipDetector::~OFDirectionSlipDetector()
{
    ROS_INFO("--end OFDirectionSlipDetector--");
    delete[] OF_buffer;
    OF_buffer=nullptr;
}

bool OFDirectionSlipDetector::isSlipped()
{
    if(isOFSlip())
    {
        return true;
    }
    return false;
}

bool OFDirectionSlipDetector::grabOFState(CeleX5* celex)
{
    if(celex->getSensorFixedMode()!=CeleX5::Optical_Flow_Mode)
    {
        // memset
        ROS_ERROR("celex mode is not Optical_Flow_Mode");
        return false;
    }

	// cv::Mat matOpticalDirection = celex->getOpticalFlowPicMat(CeleX5::OpticalFlowDirectionPic);
	celex->getOpticalFlowPicBuffer(OF_buffer,CeleX5::OpticalFlowDirectionPic);
    if(!getOFDirection(OF_buffer))return false;

    return true;
}

bool OFDirectionSlipDetector::initEventWindow()
{
    while(env_window_(0)==0)
    {
        int data_size=0;
    	celex_->getOpticalFlowPicBuffer(OF_buffer,CeleX5::OpticalFlowDirectionPic);
        for (int i = 0; i < CELEX5_ROW; ++i)
        {
            for (int j = 0; j < CELEX5_COL; ++j)
            {
                if(OF_buffer[i*CELEX5_COL+j]!=0)
                    data_size++;
            }
        }
        updateEventWindow(data_size);
    }
    return true;
}


bool OFDirectionSlipDetector::getOFDirection(uint8_t* OF_buffer)
{
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
            if(!isInRangeROI(i,j)){ROS_INFO("==============");continue;}
            value = OF_buffer[i*CELEX5_COL+j];
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
    cv::resize(cv::Mat(CELEX5_ROW,CELEX5_COL,CV_8UC1,OF_buffer),mat_half_,cv::Size(CELEX5_COL/2,CELEX5_ROW/2));
    memset(OF_buffer,0,CELEX5_PIXELS_NUMBER);
    int data_size = dir_count1+dir_count2+dir_count3+dir_count4;
    updateEventWindow(data_size);

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
    if(direction==3)
    {
        ROS_INFO("down!");
        return true;
    }


    return false;
}

bool OFDirectionSlipDetector::isOFSlip()
{
    bool ret = false;
    // 方向是否是down
    if(!grabOFState(celex_))return false;
    // 数量是否满足
    if (env_window_(envWindowSize-1) > dynamic_threshold_)
    {
        ROS_INFO("more");
        ret = true;
    }
    // 这一步似乎意义不大，因为基本more了，就slip了
    // if(!isLineDetected(mat_half_))
    // {
    //     ret = false;
    // }

    return ret;
}



}