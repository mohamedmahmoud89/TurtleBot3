#ifndef ODOM_ITF_HPP
#define ODOM_ITF_HPP

#include "funcItf.hpp"
#include "common.hpp"

class OdomInputData : public FuncItf{
public:
    WheelVelocity wheelVel{0,0};
};

class OdomOutputData : public FuncItf{
public:
    RobotPos robotPos{0,0,M_PI_2};
};
#endif