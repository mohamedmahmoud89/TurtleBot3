#ifndef PP_NODE_HPP
#define PP_NODE_HPP

#include "nodeBase.hpp"
#include "gs.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>

using namespace std;

struct PpNodeCfg{
    static constexpr f32 unit_maze_cell_length_mm{290};
};

class PpNode : public RosNodeBase{
    void update() override{
        if(locDone&&!pathPlanned){
            path.clear();
            list<GraphNode> nodeSeq;
            GraphNode start(
                static_cast<u16>(globalPos.x_mm/PpNodeCfg::unit_maze_cell_length_mm),
                static_cast<u16>(globalPos.y_mm/PpNodeCfg::unit_maze_cell_length_mm));
            if(MazeGraph::nodes.find(start)!=MazeGraph::nodes.end()){
                bool path_found = gs.search(MazeGraph::nodes,start,MazeGraph::goal,nodeSeq);
                if(!nodeSeq.empty())
                    constructPath(nodeSeq,path);
                    pathPlanned=true;
            }
        }
        publish(path);
    }

    void constructPath(const list<GraphNode>& nodeSeq,vector<PathSeg>& path){
        vector<GraphNode>vec;
        vec.assign(nodeSeq.begin(),nodeSeq.end());
        for(u16 idx=0;idx<nodeSeq.size()-1;++idx){
            PathSeg seg;
            seg.p1.x_mm=(vec[idx].x*PpNodeCfg::unit_maze_cell_length_mm)+(PpNodeCfg::unit_maze_cell_length_mm/2);
            seg.p1.y_mm=(vec[idx].y*PpNodeCfg::unit_maze_cell_length_mm)+(PpNodeCfg::unit_maze_cell_length_mm/2);
            seg.p2.x_mm=(vec[idx+1].x*PpNodeCfg::unit_maze_cell_length_mm)+(PpNodeCfg::unit_maze_cell_length_mm/2);
            seg.p2.y_mm=(vec[idx+1].y*PpNodeCfg::unit_maze_cell_length_mm)+(PpNodeCfg::unit_maze_cell_length_mm/2);
            if(vec[idx+1].x>vec[idx].x){
                seg.p1.theta_rad=0;
                seg.p2.theta_rad=0;
            }
            else if(vec[idx+1].x<vec[idx].x){
                seg.p1.theta_rad=M_PI;
                seg.p2.theta_rad=M_PI;
            }
            else if(vec[idx+1].y>vec[idx].y){
                seg.p1.theta_rad=M_PI_2;
                seg.p2.theta_rad=M_PI_2;
            }
            else{
                seg.p1.theta_rad=M_PI_2+M_PI;
                seg.p2.theta_rad=M_PI_2+M_PI;
            }
            path.push_back(seg);
        }
    }

    void publish(const vector<PathSeg>& path){
        geometry_msgs::PoseArray pathMsg;
        
        // path
        pathMsg.poses.reserve(path.size()*2);
        for(auto& seg:path){
            geometry_msgs::Pose p1,p2;
            p1.position.x=seg.p1.x_mm;
            p1.position.y=seg.p1.y_mm;
            p1.orientation.z=seg.p1.theta_rad;
            p2.position.x=seg.p2.x_mm;
            p2.position.y=seg.p2.y_mm;
            p2.orientation.z=seg.p2.theta_rad;
            pathMsg.poses.push_back(p1);
            pathMsg.poses.push_back(p2);
        }
        pathPub.publish(pathMsg);
    }

    static void storePos(const geometry_msgs::Pose2D& msg){
        Lock l(posMux);
        globalPos.x_mm=msg.x;
        globalPos.y_mm=msg.y;
        globalPos.theta_rad=msg.theta;
    }

    static void storeLocStatus(const std_msgs::Bool& msg){
        locDone=msg.data;
        if(!locDone){
            pathPlanned=false;
            path.clear();
        }
    }
public:
    PpNode():
        RosNodeBase("pp"),
        globalPosSub(n.subscribe("globalPos",100,storePos)),
        locStatSub(n.subscribe("locStatus",100,storeLocStatus)),
        pathPub(n.advertise<geometry_msgs::PoseArray>("path", 1000)){}
private:
    Subscriber globalPosSub;
    Subscriber locStatSub;
    Publisher pathPub;
    GraphSearch gs;
    static vector<PathSeg> path;
    static RobotPos globalPos;
    static mutex posMux;
    static bool locDone;
    static bool pathPlanned;
};

RobotPos PpNode::globalPos={0,0,M_PI_2};
mutex PpNode::posMux;
bool PpNode::locDone=false;
bool PpNode::pathPlanned=false;
vector<PathSeg> PpNode::path;

#endif 