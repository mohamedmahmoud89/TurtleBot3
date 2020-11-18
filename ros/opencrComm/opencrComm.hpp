#ifndef OPENCR_COMM_NODE_HPP
#define OPENCR_COMM_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>
#include<fstream>
#include<string>

using namespace std;

#define FILE "/dev/ttyACM0"

class OpenCrCommNode : public RosNodeBase{
    void update() override{
        fstream fs;
        string line;
        fs.open(FILE);
        if(fs.is_open()){
                fs<<"query#"<<endl;
                while(getline(fs,line)&&line!="over"){
                        cout<<line<<endl;
                }
                fs.close();
        }
    }
    static void sendData(const std_msgs::String& msg){
        fstream fs;
        string line;
        fs.open(FILE);
        if(fs.is_open()){
                fs<<msg.data<<'#'<<endl;
                while(getline(fs,line)&&line!="over")
                        cout<<line<<endl;
                fs.close();
        }
    }
public:
    OpenCrCommNode():
            RosNodeBase(20),
            ctrlCmdDataSub(n.subscribe("ctrlCmd",100,sendData)){}

private:
    Subscriber ctrlCmdDataSub;
};

#endif
