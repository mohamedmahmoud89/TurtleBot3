#ifndef OPENCR_COMM_NODE_HPP
#define OPENCR_COMM_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include<fstream>
#include<string>

using namespace std;
#define FILE "/dev/ttyACM0"

struct OpenCrCommNodeCfg{
    static constexpr u16 rx_watchdog_bouncer_limit{10};
};

class OpenCrCommNode : public RosNodeBase{
    si16 parseStr(string&& s){
        si16 ret(0);
        si16 sign(1);

        if(s[0]=='-'){
            sign*=-1;
            s=s.substr(1,s.size()-1);
        }

        for(int idx=s.size()-1;idx>=0;--idx){
            ret+=((s[idx]-'0')*pow(10,((s.size()-1)-idx)));
        }

        return ret*sign;
    }
    void update() override{
        std_msgs::Int16MultiArray msg;
        std_msgs::MultiArrayDimension dim;
        fstream fs;
        string line;
        bool stop=false;

        if(rx_watchdog_bouncer<=OpenCrCommNodeCfg::rx_watchdog_bouncer_limit){
            rx_watchdog_bouncer++;
        }
        else if(!stopped){
            stop=true;
            stopped=true;
        }
        
        dim.size=2;
        dim.stride=1;
        msg.layout.dim.push_back(dim);
        {
            Lock l(fileMux);
            fs.open(FILE);
            if(fs.is_open()){
                // force stop if no commands coming
                if(stop){
                    lastCmd="stop";
                    fs<<"stop#"<<endl;
                    while(getline(fs,line)&&line!="over"){}
                }
                else{
                    fs<<"query#"<<endl;
                    while(getline(fs,line)&&line!="over"){
                        //cout<<line<<endl;
                        if(line.size()){
                            msg.data.push_back(parseStr(line.substr(2,line.size()-2)));
                        }
                    }
                }
                fs.close();
            }
        }

        if(msg.data.size()==2)
            odomDataPub.publish(msg);
    }
    static void sendData(const std_msgs::String& msg){
        fstream fs;
        string line;
        Lock l(fileMux);
        rx_watchdog_bouncer=0;
        stopped=false;
            
        if(msg.data.empty()||lastCmd==msg.data)
            return;
       	
        fs.open(FILE);
        if(fs.is_open()){
            fs<<msg.data<<'#'<<endl;
            while(getline(fs,line)&&line!="over"){}
            fs.close();
            lastCmd=msg.data;
        }
    }
public:
    OpenCrCommNode():
            RosNodeBase("openCR"),
            ctrlCmdDataSub(n.subscribe("ctrlCmd",100,sendData)),
            odomDataPub(n.advertise<std_msgs::Int16MultiArray>("odom", 1000)){}

private:
    Subscriber ctrlCmdDataSub;
    Publisher odomDataPub;
    static mutex fileMux;
    static string lastCmd;
    static bool stopped;
    static u16 rx_watchdog_bouncer;
};

mutex OpenCrCommNode::fileMux;
string OpenCrCommNode::lastCmd="";
u16 OpenCrCommNode::rx_watchdog_bouncer=0;
bool OpenCrCommNode::stopped=false;
#endif
