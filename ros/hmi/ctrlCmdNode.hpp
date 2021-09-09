#ifndef CTRL_CMD_NODE_HPP
#define CTRL_CMD_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>
#include <iostream>

using namespace std;

class CtrlCmdNode : public RosNodeBase{
    void update() override{
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;
        string cmd;
        cout<<"Enter Command:"<<endl;
        
        cin>>cmd;
        if(cmd=="f")msg.data="fwd";
        if(cmd=="b")msg.data="bwd";
        if(cmd=="s")msg.data="stop";
        if(cmd=="l")msg.data="left";
        if(cmd=="r")msg.data="right";
        if(cmd=="+")msg.data="up";
        if(cmd=="-")msg.data="down";

        if(msg.data.size()){
            ROS_INFO("%s", msg.data.c_str());
            ctrlCmdDataPub.publish(msg);
        }
    }
public:
    CtrlCmdNode():
        RosNodeBase("hmi"),
        ctrlCmdDataPub(n.advertise<std_msgs::String>("ctrlCmd", 1000)){}

private:
    Publisher ctrlCmdDataPub;
};

#endif