#ifndef ROS_NODE_BASE_HPP
#define ROS_NODE_BASE_HPP

#include <ros/ros.h>
#include "common.hpp"

using namespace ros;

enum class RosNodeBaseType{
    SPINNING=0,
    NON_SPINNING
};

class RosNodeBase{
    virtual void update()=0;

public:
    RosNodeBase(
        const RosNodeBaseType type = RosNodeBaseType::SPINNING,
        const u16 param_rate = SystemCfg::rate_hz):
            n(),
            rate(param_rate),
            nodeType(type){}
    virtual ~RosNodeBase(){}
    void run(){
        while (ros::ok())
        {
            update();
            if(nodeType==RosNodeBaseType::SPINNING){
                ros::spinOnce();
                rate.sleep();
            }
        }
    }
protected:
    NodeHandle n;
private:
    Rate rate; // Hz
    RosNodeBaseType nodeType;
};

#endif