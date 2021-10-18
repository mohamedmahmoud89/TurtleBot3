#ifndef PP_NODE_HPP
#define PP_NODE_HPP

#include "nodeBase.hpp"
#include "gs.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <vector>

using namespace std;

struct PpNodeCfg{
};

enum PathSegType{
    STRAIGHT=0,
    RIGHT_TURN,
    LEFT_TURN
};

struct PathSeg{
    PathSeg():p1(),p2(),type(){}
    PathSeg(const RobotPos& arg1,const RobotPos& arg2, const PathSegType t):p1(arg1),p2(arg2),type(t){}
    RobotPos p1;
    RobotPos p2;
    PathSegType type;
};

class PpNode : public RosNodeBase{
    void update() override{
        list<GraphNode> nodeSeq;
        GraphNode start,end;
        gs.search(MazeGraph::nodes,start,end,nodeSeq);
        constructPath(nodeSeq);
    }

    void constructPath(const list<GraphNode>& nodeSeq){

    }

    void publish(){
        /*geometry_msgs::PoseArray particlesMsg;
        geometry_msgs::Pose2D globalPosMsg;
        
        // path
        particlesMsg.poses.reserve(MclNodeCfg::particles_num);
        for(auto& pt:pf.getParticles()){
            geometry_msgs::Pose p;
            p.position.x=pt.x_mm;
            p.position.y=pt.y_mm;
            p.orientation.z=pt.theta_rad;
            particlesMsg.poses.push_back(p);
        }
        particlesPub.publish(particlesMsg);*/
    }

    static void storePos(const geometry_msgs::Pose2D& msg){
        //Lock l(posMux);
    }
public:
    PpNode():
        RosNodeBase("pp"),
        globalPosSub(n.subscribe("globalPos",100,storePos)),
        pathsPub(n.advertise<geometry_msgs::PoseArray>("path", 1000)){}
private:
    Subscriber globalPosSub;
    Publisher pathsPub;
    GraphSearch gs;
    vector<PathSeg> path;
};

#endif 