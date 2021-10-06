#ifndef CTRL_GATEWAY_NODE_HPP
#define CTRL_GATEWAY_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>

using namespace std;

class HmiGatewayNode : public RosNodeBase{
    void update() override{
        Lock l(lock);
        std_msgs::String msg;
        msg.data=cmd;
        ctrlCmdDataPub.publish(msg);
    }

    static void processData(const std_msgs::String& msg){
        Lock l(lock);
        cmd=msg.data;
    }
public:
    HmiGatewayNode():
        RosNodeBase("hmiGateway"),
        ctrlCmdDataSub(n.subscribe("ctrlCmdCaptured",100,processData)),
        ctrlCmdDataPub(n.advertise<std_msgs::String>("ctrlCmd", 1000)){}

private:
    Subscriber ctrlCmdDataSub;
    Publisher ctrlCmdDataPub;
    static mutex lock;
    static string cmd;
};

mutex HmiGatewayNode::lock;
string HmiGatewayNode::cmd="";

#endif