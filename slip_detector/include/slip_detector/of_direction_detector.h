#pragma once

#include "slip_detector/slip_detector.h"

namespace celex_ros{

class OFDirectionSlipDetector : public SlipDetector
{

public:
    OFDirectionSlipDetector(/* args */);
    ~OFDirectionSlipDetector();

    bool isSlipped();
    bool initEventWindow();

    bool isOFSlip();
    bool grabOFState(CeleX5 *celex);
    // bool updateEventWindow(int data_size);
    bool getOFDirection(uint8_t* OF_buffer);

    private:
    // std::shared_ptr<uint8_t> OF_buffer;
    uint8_t* OF_buffer;
    Stopwatch of_timer_;
    /* data */
};







}