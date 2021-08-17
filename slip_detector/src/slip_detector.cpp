#include "slip_detector/slip_detector.h"
#include<typeinfo>
namespace celex_ros {

SlipDetector::SlipDetector() : node_("~"),off_time_zero_(-99),cur_off_time_from_zero_(0),ROI_area_(4),max_slip_cnt_(2),continuous_slip_cnt_(0)
{
    // image_pub_ = node_.advertise<sensor_msgs::Image>("celex_image", 1);
    // event_pub_ = node_.advertise<celex5_msgs_sdk::EventVector>("celex_event", 1);
    slip_pub_ = node_.advertise<std_msgs::String>("slip_signal", 1);

    node_.param<std::string>("celex_mode", celex_mode_, "Event_Off_Pixel_Timestamp_Mode");
    ROS_INFO("mode is %s", celex_mode_.c_str());

    node_.param<int>("threshold", threshold_, 170);   // 0-1024
    node_.param<int>("clock_rate", clock_rate_, 100); // 0-100

    node_.param<int>("ROI_top",ROI_area_[0],0);
    node_.param<int>("ROI_left",ROI_area_[1],0);
    node_.param<int>("ROI_width",ROI_area_[2],1280);
    node_.param<int>("ROI_height",ROI_area_[3],800);
    ROS_INFO("ROI_area is: topLetf(%d, %d); wifth:%d; height:%d.",ROI_area_[0],ROI_area_[1],ROI_area_[2],ROI_area_[3]);


    node_.param<float>("dynamic_threshold_scale",dynamic_threshold_scale_,2.55);
    node_.param<float>("cor_threshold_scale",cor_threshold_scale_,1.55);
    // std::cout<<"dynamic_threshold_scale_ is:"<<dynamic_threshold_scale_<<"; cor_threshold_scale_: "<<cor_threshold_scale_<<std::endl;

    node_.param<int>("max_slip_cnt",max_slip_cnt_,2);

    node_.param<bool>("isInitThresTest",isInitThresTest_,false);
    node_.param<int>("cor_init_threshold",cor_init_threshold_,10);
    node_.param<int>("event_init_threshold",event_init_threshold_,10);
    ROS_INFO("(isInitThresTest, cor_init_threshold_, event_init_threshold_)is(%d,%d,%d)",isInitThresTest_,cor_init_threshold_,event_init_threshold_);
    // std::cout<<isInitThresTest_<<","<<cor_init_threshold_<<","<<event_init_threshold_<<std::endl;

    mat_half_ = cv::Mat::zeros(cv::Size(MAT_COLS/2, MAT_ROWS/2), CV_8UC1);
    env_window_.setZero();
    cor_window_.setZero();

}

SlipDetector::~SlipDetector(){ROS_INFO("--end SlipDetector--");}

void SlipDetector::setCeleX5(CeleX5 *pcelex)
{
    celex_ = pcelex;
    celex_->setThreshold(threshold_);

    CeleX5::CeleX5Mode mode;
    if (celex_mode_ == "Event_Off_Pixel_Timestamp_Mode")
    {
        int event_frame_time=1000;
        node_.param<int>("event_frame_time",event_frame_time,1000);
        mode = CeleX5::Event_Off_Pixel_Timestamp_Mode;
        celex_->setSensorFixedMode(mode);
        celex_->setEventFrameTime(event_frame_time);
        celex_->disableFrameModule();
    }
    else if (celex_mode_ == "Optical_Flow_Mode")
    {
        mode = CeleX5::Optical_Flow_Mode;
        celex_->enableEventOpticalFlow();
        celex_->setSensorFixedMode(mode);
        // ROS_INFO("of time is %d",celex_->getOpticalFlowFrameTime());
        // 20 for default
        // range is (10,180)
        celex_->setOpticalFlowFrameTime(11);
        // ROS_INFO("of time is %d",celex_->getOpticalFlowFrameTime());
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


    ROS_INFO("work rate is:%d us", celex_->getEventFrameTime());
    ROS_INFO("work fixed mode is:%d",celex_->getSensorFixedMode());
    ROS_INFO("work loop1 mode is:%d",celex_->getSensorLoopMode(1));
    ROS_INFO("work loop2 mode is:%d",celex_->getSensorLoopMode(2));
    ROS_INFO("work loop3 mode is:%d",celex_->getSensorLoopMode(3));
}

uint64_t SlipDetector::get_cur_off_time_from_zero(){return cur_off_time_from_zero_;}
void SlipDetector::set_cur_off_time_from_zero(int a){cur_off_time_from_zero_=a;}

void SlipDetector::slipPublish(std_msgs::String& msg)
{
    slip_pub_.publish(msg);
}

int SlipDetector::getEnvWindowNum(int num)
{
    return env_window_(num);
}

int SlipDetector::getCorWindowNum(int num){return cor_window_(num);}

bool SlipDetector::isCornerDetected(cv::Mat& mat_corner)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::FastFeatureDetector> fast_detector = cv::FastFeatureDetector::create();
    fast_detector->detect(mat_corner,keypoints);
    // ROS_INFO("corner num is:%d",keypoints.size());
    // 这里就不应该随便更新，否则会导致角点数量过多
    if(/*env_window_(0)!=0 &&*/ cor_window_(0)==0)updateCorWindow(keypoints.size());
    // 为了计算稳定环境下角点均值
    if(isInitThresTest_)
    {
        cor_cnt_sss++;
        cor_total+=keypoints.size();
        cor_avg_num=cor_total/cor_cnt_sss;
        ROS_INFO("avg_cor_num is : %d",cor_avg_num);
    }
    if(keypoints.size()>cor_threshold_){
        // ROS_INFO("last cor:%d,thresis %d",keypoints.size(),cor_threshold_);
        return true;
    }
    return false;
}

// 这个现在没用
bool SlipDetector::isLineDetected(cv::Mat& mat_hough)
{
    
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(mat_hough,lines,1.0,CV_PI/180,102);
    // cv::imshow("123",mat_half_);
    // cv::waitKey(1);
    // 这里置0没卵用，因为不一定走这里
    mat_half_ = cv::Mat::zeros(cv::Size(MAT_COLS/2, MAT_ROWS/2), CV_8UC1);
    // std::cout<<"clos rows"<<mat_hough.cols<<","<<mat_hough.rows<<std::endl;
    // mat_hough = cv::Mat::zeros(cv::Size(mat_hough.cols,mat_hough.rows),CV_8UC1);
    ROS_INFO("line size is:%d",lines.size());
    
    if(lines.size()>0)return true;
    return false;
}

bool SlipDetector::updateEventWindow(int data_size)
{
    // 这个值可以考虑修改一下
    if(data_size<event_init_threshold_)return false;
    if(data_size==env_window_(envWindowSize-1))return false;
    env_window_.topRows<envWindowSize-1>()=env_window_.bottomRows<envWindowSize-1>();
    env_window_(env_window_.size()-1)=data_size;
    // 这里改成了11个大小的，其中前10个用来计算阈值，第11个是最新的
    // dynamic_threshold_=((env_window_.sum()-env_window_(envWindowSize-1))/(env_window_.size()-1))*dynamic_threshold_scale_;
    dynamic_threshold_ =(env_window_.topRows<envWindowSize-1>().sum()/(env_window_.size()-1))*dynamic_threshold_scale_;
    return true;
}

// 其实这两个函数可以合并的
bool SlipDetector::updateCorWindow(int cor_cnt)
{
    if(cor_cnt<cor_init_threshold_)return false;
    if(cor_cnt==cor_window_(envWindowSize-1))return false;
    cor_window_.topRows<envWindowSize-1>()=cor_window_.bottomRows<envWindowSize-1>();
    cor_window_(cor_window_.size()-1)=cor_cnt;
    // 这里改成了11个大小的，其中前10个用来计算阈值，第11个是最新的
    cor_threshold_ =(cor_window_.topRows<envWindowSize-1>().sum()/(cor_window_.size()-1))*cor_threshold_scale_;
    if(cor_window_(0)!=0)ROS_INFO("==corner_threshold init done: %d==",cor_threshold_);
    // std::cout<<cor_window_<<std::endl<<"----"<<std::endl;
    return true;
}

// 被动模式(cb)下，这个函数用不到
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
        slip_pub_.publish(msggg);

        // ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}


void SlipDetector::set_slip_cnt(int cnt){continuous_slip_cnt_=cnt;}
int SlipDetector::get_slip_cnt(){return continuous_slip_cnt_;}
int SlipDetector::get_max_slip_cnt(){return max_slip_cnt_;}

// void SlipDetector::run()
// {

// }


}
