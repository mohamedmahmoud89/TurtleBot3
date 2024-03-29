#ifndef MONITOR_NODE_HPP
#define MONITOR_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/Float64.h>

using namespace std;

enum Nodes{
    OPENCR=0,
    LDS,
    LFX,
    DISP,
    MCL,
    PP,
    CTRL,
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

    static void processPpData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[PP].rt=msg.data;
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
    static void processCtrlData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[CTRL].rt=msg.data;
    }
    static void processMclData(const std_msgs::Float64& msg){
        Lock l(lock);
        latest_rt[MCL].rt=msg.data;
    }
public:
    MonitorNode():
        RosNodeBase(rate_hz,"monitor"),
        ppDataSub(n.subscribe("rt_pp",100,processPpData)),
        opencrDataSub(n.subscribe("rt_openCR",100,processOpencrData)),
        ldsDataSub(n.subscribe("rt_lds",100,processLdsData)),
        lfxDataSub(n.subscribe("rt_lfx",100,processLfxData)),
        ctrlDataSub(n.subscribe("rt_ctrl",100,processCtrlData)),
        mclDataSub(n.subscribe("rt_mcl",100,processMclData)),
        dispDataSub(n.subscribe("rt_display",100,processDispData)){}

private:
    static rtElement latest_rt[NUM_NODES];
    Subscriber ppDataSub;
    Subscriber opencrDataSub;
    Subscriber ldsDataSub;
    Subscriber lfxDataSub;
    Subscriber ctrlDataSub;
    Subscriber mclDataSub;
    Subscriber dispDataSub;
    static constexpr u16 rate_hz=10;
    static mutex lock;
};

rtElement MonitorNode::latest_rt[NUM_NODES]={{0,"opencr"},{0,"lds"},{0,"lfx"},{0,"disp"},{0,"mcl"},{0,"pp"},{0,"ctrl"}};
mutex MonitorNode::lock;

#endif