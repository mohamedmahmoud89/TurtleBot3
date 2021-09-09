#ifndef ROS_NODE_BASE_HPP
#define ROS_NODE_BASE_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
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
        const RosNodeBaseType type,
        const u16 param_rate,
        const string& s):
            n(),
            rate(param_rate),
            nodeType(type),
            runtime_sec(0),
            name(s),
            runtimePub(n.advertise<std_msgs::Float64>("rt_"+name, 1000)){}
    RosNodeBase(const string& s):
            n(),
            rate(SystemCfg::rate_hz),
            nodeType(RosNodeBaseType::SPINNING),
            runtime_sec(0),
            name(s),
            runtimePub(n.advertise<std_msgs::Float64>("rt_"+name, 1000)){}
    RosNodeBase(const RosNodeBaseType type,const string& s):
            n(),
            rate(SystemCfg::rate_hz),
            nodeType(type),
            runtime_sec(0),
            name(s),
            runtimePub(n.advertise<std_msgs::Float64>("rt_"+name, 1000)){}
    RosNodeBase(const u16 param_rate,const string& s):
            n(),
            rate(param_rate),
            nodeType(RosNodeBaseType::SPINNING),
            runtime_sec(0),
            name(s),
            runtimePub(n.advertise<std_msgs::Float64>("rt_"+name, 1000)){}
    virtual ~RosNodeBase(){}
    void run(){
        std_msgs::Float64 msg;
        while (ros::ok())
        {
            ros::Time t=ros::Time::now();
            update();
            msg.data=(ros::Time::now()-t).toSec();
            runtimePub.publish(msg);
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
    string name;
    double runtime_sec;
    Publisher runtimePub;
};

#endif