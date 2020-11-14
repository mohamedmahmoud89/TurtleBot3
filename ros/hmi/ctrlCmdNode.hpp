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

        cout<<"Enter Command:"<<endl;
        cin>>msg.data;

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        ctrlCmdDataPub.publish(msg);
    }
public:
    CtrlCmdNode():RosNodeBase(10),ctrlCmdDataPub(n.advertise<std_msgs::String>("ctrlCmd", 1000)){}

private:
    Publisher ctrlCmdDataPub;
};

#endif