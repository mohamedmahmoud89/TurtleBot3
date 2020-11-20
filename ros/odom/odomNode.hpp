#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

#include "nodeBase.hpp"
#include "common.hpp"
#include <std_msgs/Int16MultiArray.h>

using namespace std;

#define MIN_TO_SEC 60 

class OdomNode : public RosNodeBase{
    void update() override{}
    
    static void updatePos(const WheelVelocity& wheelVelocity){
        f32 num_revolutions_left = 
            (static_cast<f32>(wheelVelocity.left_rpm))/(MIN_TO_SEC*SystemCfg::rate_hz);
        drivenDistLeft += num_revolutions_left*RobotCfg::wheelDiameter_mm;
    }  
    static void processData(const std_msgs::Int16MultiArray& msg){
        WheelVelocity vel;
        vel.left_rpm=msg.data[0];
        vel.right_rpm=msg.data[1];
        updatePos(vel);
        //cout<<"left="<<vel.left_rpm<<endl;
        //cout<<"right="<<vel.right_rpm<<endl;
        cout<<"dd="<<drivenDistLeft<<endl;
    }
public:
    OdomNode(): 
        RosNodeBase(SystemCfg::rate_hz),
        odomDataSub(n.subscribe("odom",100,processData)){}
private:
    Subscriber odomDataSub;
    static f32 drivenDistLeft;
};

f32 OdomNode::drivenDistLeft=0;

#endif