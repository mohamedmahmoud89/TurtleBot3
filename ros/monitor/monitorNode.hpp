#ifndef MONITOR_NODE_HPP
#define MONITOR_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/Float64.h>

using namespace std;

enum Nodes{
    ODOM=0,
    HMI,
    OPENCR,
    LDS,
    LFX,
    DISP,
    MAP,
    NUM_NODES
};

struct rtElement{
    double rt;
    string name;
};

class MonitorNode : public RosNodeBase{
    void update() override{
        Lock l(lock);
        for(int i=0;i<NUM_NODES;++i){
            cout<<"rt of "<<latest_rt[i].name<<" = "<<latest_rt[i].rt*1000<<" ms"<<endl;
        }
    }

    static void processOdomData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[ODOM].rt=msg.data;
    }
    static void processOpencrData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[OPENCR].rt=msg.data;
    }
    static void processLdsData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[LDS].rt=msg.data;
    }
    static void processLfxData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[LFX].rt=msg.data;
    }
    static void processDispData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[DISP].rt=msg.data;
    }
    static void processHmiData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[HMI].rt=msg.data;
    }
    static void processMapData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[MAP].rt=msg.data;
    }
public:
    MonitorNode():
        RosNodeBase(rate_hz,"monitor"),
        odomDataSub(n.subscribe("rt_odom",100,processOdomData)),
        opencrDataSub(n.subscribe("rt_openCR",100,processOpencrData)),
        ldsDataSub(n.subscribe("rt_lds",100,processLdsData)),
        lfxDataSub(n.subscribe("rt_lfx",100,processLfxData)),
        hmiDataSub(n.subscribe("rt_hmiGateway",100,processHmiData)),
        mapDataSub(n.subscribe("rt_map",100,processMapData)),
        dispDataSub(n.subscribe("rt_display",100,processDispData)){}

private:
    static rtElement latest_rt[NUM_NODES];
    Subscriber odomDataSub;
    Subscriber opencrDataSub;
    Subscriber ldsDataSub;
    Subscriber lfxDataSub;
    Subscriber hmiDataSub;
    Subscriber mapDataSub;
    Subscriber dispDataSub;
    static constexpr u16 rate_hz=10;
    static mutex lock;
};

rtElement MonitorNode::latest_rt[NUM_NODES]={{0,"odom"},{0,"hmi"},{0,"opencr"},{0,"lds"},{0,"lfx"},{0,"disp"},{0,"map"}};
mutex MonitorNode::lock;

#endif