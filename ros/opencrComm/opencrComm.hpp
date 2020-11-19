#ifndef OPENCR_COMM_NODE_HPP
#define OPENCR_COMM_NODE_HPP

#include "nodeBase.hpp"
#include "common.hpp"
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include<fstream>
#include<string>

using namespace std;

#define FILE "/dev/ttyACM0"

class OpenCrCommNode : public RosNodeBase{
    void update() override{
        std_msgs::UInt8MultiArray msg;
        std_msgs::MultiArrayDimension dim;
        fstream fs;
        string line;

        dim.size=2;
        dim.stride=1;
        msg.layout.dim.push_back(dim);
        fs.open(FILE);
        if(fs.is_open()){
                fs<<"query#"<<endl;
                while(getline(fs,line)&&line!="over"){
                    cout<<line<<endl;
                    if(line.size()&&line[0]=='l')
                        msg.data[0]=static_cast<u8>(line[2]);
                    else if(line.size()&&line[0]=='l')
                        msg.data[1]=static_cast<u8>(line[2]);
                }
                fs.close();
        }
        odomDataPub.publish(msg);
    }
    static void sendData(const std_msgs::String& msg){
        fstream fs;
        string line;
        fs.open(FILE);
        if(fs.is_open()){
                fs<<msg.data<<'#'<<endl;
                while(getline(fs,line)&&line!="over")
                        //cout<<line<<endl;
                fs.close();
        }
    }
public:
    OpenCrCommNode():
            RosNodeBase(SystemCfg::rate_hz),
            ctrlCmdDataSub(n.subscribe("ctrlCmd",100,sendData)),
            odomDataPub(n.advertise<std_msgs::UInt8MultiArray>("odom", 1000)){}

private:
    Subscriber ctrlCmdDataSub;
    Publisher odomDataPub;
};

#endif
