#ifndef ROS_NODE_BASE_HPP
#define ROS_NODE_BASE_HPP

#include <ros/ros.h>

using namespace ros;

class RosNodeBase{
    virtual void update()=0;

public:
    RosNodeBase(const int param_rate):n(),rate(param_rate){}
    virtual ~RosNodeBase(){}
    void run(){
        while (ros::ok())
        {
            update();
            ros::spinOnce();
            rate.sleep();
        }
    }
protected:
    NodeHandle n;
private:
    Rate rate;
};

#endif