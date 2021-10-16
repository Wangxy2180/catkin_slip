#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <celex5_msgs_sdk/Event.h>
#include <celex5_msgs_sdk/EventVector.h>
#include "timer.h"

#define MAT_ROWS 800
#define MAT_COLS 1280
namespace celex_ros{
class SlipDetector
{
public:
    SlipDetector(/* args */);
    virtual ~SlipDetector();

    void setCeleX5(CeleX5 *pcelex);
    bool run();
    void slipPublish(std_msgs::String& msg);
    int getEnvWindowNum(int num);
    int getCorWindowNum(int num);
    bool isLineDetected(cv::Mat& mat_hough);
    bool isCornerDetected(cv::Mat& mat_corner);
    bool isInRangeROI(int row, int col);


    bool updateEventWindow(int data_size);
    bool updateCorWindow(int cor_cnt);
    uint64_t get_cur_off_time_from_zero();
    void set_cur_off_time_from_zero(int a);

    void set_slip_cnt(int cnt);
    int  get_slip_cnt();

    int get_max_slip_cnt();



    virtual bool isSlipped()=0;
    virtual bool initEventWindow()=0;

    Stopwatch timer_;


private:
    bool isRunning=false;
    // ros::Publisher event_pub_, image_pub_;

    std::string celex_mode_;
    int threshold_, clock_rate_;

    int max_slip_cnt_;

    int slip_total_cnt_;
    int slip_total_ns_;

protected:

    static const int envWindowSize = 11;
    float dynamic_threshold_scale_;// = 2.5;
    float cor_threshold_scale_;// = 1.5;

    ros::NodeHandle node_;

    ros::Publisher slip_pub_;
    int dynamic_threshold_ = -1;
    int cor_threshold_ = -1;


    std::string detector_name;
    CeleX5 *celex_;
    Eigen::Array<int, envWindowSize, 1> env_window_;
    Eigen::Array<int,envWindowSize,1> cor_window_;


    celex5_msgs_sdk::EventVector event_vector_;
    cv::Mat mat_half_;
    std::vector<int> ROI_area_;

    int ROI_area_top_;
    int ROI_area_bot_;
    int ROI_area_left_;
    int ROI_area_right_;

    int continuous_slip_cnt_;

    int event_avg_num=0;
    int event_total=0;
    int event_cnt_sss=0;

    int cor_avg_num=0;
    int cor_total=0;
    int cor_cnt_sss=0;

    bool isInitThresTest_;
    int cor_init_threshold_;
    int event_init_threshold_;





    // off_pixel_time zero time
    uint64_t off_time_zero_;
    uint64_t cur_off_time_from_zero_;

};





}