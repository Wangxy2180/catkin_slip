#pragma once

#include "slip_detector/slip_detector.h"

namespace celex_ros{

class LoopSlipDetector : public SlipDetector
{

public:
    LoopSlipDetector(/* args */);
    ~LoopSlipDetector();

    bool isSlipped();
    bool initEventWindow();

    bool isLoopSlip();
    bool grabLoopData();
    // bool grabOFState(CeleX5 *celex);
    // bool updateEventWindow(int data_size);
    bool getOFDirection(uint8_t* loop_of_buffer);

private:
    uint8_t* loop_of_buffer_;
	uint8_t* loop_bin_buffer_;
	// uint8_t* loop_buffer3;
    cv::Mat bin_pic_half;
    // std::shared_ptr<uin;t8_t> OF_buffer;
    /* data */
};







}