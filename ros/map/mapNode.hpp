#ifndef MAP_NODE_HPP
#define MAP_NODE_HPP

#include "nodeBase.hpp"
#include "featAssoc.hpp"
#include "ekf.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>

using namespace std;

class MapNode : public RosNodeBase{
    void update() override{

    }

    static void storeCorners(const geometry_msgs::PoseArray& msg){

    }

    static void storeEdges(const geometry_msgs::PoseArray& msg){
        
    }

    static void storePos(const geometry_msgs::Pose2D& msg){
        Lock mux(posMux);
        pos.x_mm = msg.x;
        pos.y_mm = msg.y;
        pos.theta_rad =msg.theta;
    }
public:
    MapNode():
        RosNodeBase("map"),
        robotPosSub(n.subscribe("robotPos",100,storePos)),
        featCornerSub(n.subscribe("featCorners",100,storeCorners)),
        featEdgeSub(n.subscribe("featEdges",100,storeEdges)),
        landmarkCornerPub(n.advertise<geometry_msgs::PoseArray>("landmarkCorners", 1000)),
        landmarkEdgePub(n.advertise<geometry_msgs::PoseArray>("landmarkEdges", 1000)){}
private:
    Subscriber robotPosSub;
    Subscriber featCornerSub;
    Subscriber featEdgeSub;
    Publisher landmarkCornerPub;
    Publisher landmarkEdgePub;
    static RobotPos pos;
    //static vector<Pose2D>featCorners;
    //static vector<Pose2D>featEdges;
    static mutex posMux;
};

RobotPos MapNode::pos(0,0,M_PI_2);
mutex MapNode::posMux;

#endif 