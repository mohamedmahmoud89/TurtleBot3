#ifndef MAZE_RUNNER_NODE_HPP
#define MAZE_RUNNER_NODE_HPP

#include "nodeBase.hpp"
#include "odomFunc.hpp"
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

class MazeRunnerNode : public RosNodeBase{
    void exec(){
        odom.exec();
    }

    void update() override{
        geometry_msgs::Pose2D msg;

        // set inputs
        {
            Lock lock(wheelDataLock);
            odomInData.wheelVel=wheelData;
        }

        // execute
        exec();

        // send outputs
        msg.x=odomOutData.robotPos.x_mm;
        msg.y=odomOutData.robotPos.y_mm;
        msg.theta=odomOutData.robotPos.theta_rad;
        posDataPub.publish(msg);
        cout<<"pos = "<<msg.x<<" , "<<msg.y<<" , "<<msg.theta*AngConversions::radToDegree<<endl;
    }
     
    static void processData(const std_msgs::Int16MultiArray& msg){
        Lock lock(wheelDataLock);
        wheelData.left_rpm=msg.data[0];
        wheelData.right_rpm=msg.data[1];
    }
public:
    MazeRunnerNode(): 
        RosNodeBase(),
        odom(odomInData,odomOutData),
        odomDataSub(n.subscribe("odom",100,processData)),
        posDataPub(n.advertise<geometry_msgs::Pose2D>("robotPos",1000)){}
private:
    Subscriber odomDataSub;
    Publisher posDataPub;
    OdomInputData odomInData;
    OdomOutputData odomOutData;
    OdomFunc odom;
    static mutex wheelDataLock;
    static WheelVelocity wheelData;
};

mutex MazeRunnerNode::wheelDataLock;
WheelVelocity MazeRunnerNode::wheelData(0,0);

#endif