#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/String.h>

using namespace std;

class OdomNode : public RosNodeBase{
    void update() override{}
    static void processData(const std_msgs::String& msg){
        cout<<msg.data<<endl;
    }
public:
    OdomNode(): 
        RosNodeBase(20),
        odomDataSub(n.subscribe("odom",100,processData)){}
private:
    Subscriber odomDataSub;
};

#endif