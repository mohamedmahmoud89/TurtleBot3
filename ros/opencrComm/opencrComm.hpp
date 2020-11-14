#ifndef OPENCR_COMM_NODE_HPP
#define OPENCR_COMM_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>
#include<fstream>

using namespace std;

class OpenCrCommNode : public RosNodeBase{
    void update() override{}
    static void sendData(const std_msgs::String& msg){
        fstream fs;
        fs.open("/dev/ttyACM0");
        fs<<msg.data;
        fs.close();
        cout<<"Cmd = "<<msg.data<<endl;
    }
public:
    OpenCrCommNode():RosNodeBase(1), ctrlCmdDataSub(n.subscribe("ctrlCmd",100,sendData)){}

private:
    Subscriber ctrlCmdDataSub;
};

#endif