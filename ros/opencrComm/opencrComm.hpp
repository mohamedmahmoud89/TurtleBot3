#ifndef OPENCR_COMM_NODE_HPP
#define OPENCR_COMM_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include<fstream>
#include<string>

using namespace std;

#define FILE "/dev/ttyACM0"

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

        dim.size=2;
        dim.stride=1;
        msg.layout.dim.push_back(dim);
        {
            Lock l(fileMux);
            fs.open(FILE);
            if(fs.is_open()){
                fs<<"query#"<<endl;
                while(getline(fs,line)&&line!="over"){
                    //cout<<line<<endl;
                    if(line.size()){
                        msg.data.push_back(parseStr(line.substr(2,line.size()-2)));
                    }
                }
                fs.close();
            }
        }
        odomDataPub.publish(msg);
    }
    static void sendData(const std_msgs::String& msg){
        fstream fs;
        string line;
        Lock l(fileMux);
        if(msg.data==lastCmd)
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
};

mutex OpenCrCommNode::fileMux;
string OpenCrCommNode::lastCmd="";

#endif
