#ifndef MAZE_RUNNER_NODE_HPP
#define MAZE_RUNNER_NODE_HPP

#include "nodeBase.hpp"
#include "odomFunc.hpp"
#include "lfxFunc.hpp"
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class MazeRunnerNode : public RosNodeBase{
    void displayFeats(const LfxFeats& feats,const RobotPos& pos){
        // points visualization
        visualization_msgs::Marker points;
        points.action = visualization_msgs::Marker::ADD;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.g = 1.0f;
        points.color.a = 1.0;
        points.pose.orientation.w = 1.0;
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.id = 0;
        
        for(u16 idx=0;idx<feats.points.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.points[idx].x_mm/100;
            p.y=feats.points[idx].y_mm/100;
            p.z=0.2;
            points.points.push_back(p);
        }

        rvizPointsPub.publish(points);

        // lines visualization
        visualization_msgs::Marker lines;
        lines.action = visualization_msgs::Marker::ADD;
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.scale.x = 0.07;
        lines.color.r = 1.0;
        lines.color.a = 1.0;
        lines.pose.orientation.w = 1.0;
        lines.header.frame_id = "/map";
        lines.header.stamp = ros::Time::now();
        lines.id = 1;
        
        for(u16 idx=0;idx<feats.lines.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.lines[idx].p1.x_mm/100;
            p.y=feats.lines[idx].p1.y_mm/100;
            p.z=0.2;
            lines.points.push_back(p);
            p.x=feats.lines[idx].p2.x_mm/100;
            p.y=feats.lines[idx].p2.y_mm/100;
            lines.points.push_back(p);
        }

        rvizLinesPub.publish(lines);

        // corners visualization
        visualization_msgs::Marker corners;
        corners.action = visualization_msgs::Marker::ADD;
        corners.type = visualization_msgs::Marker::POINTS;
        corners.scale.x = 0.1;
        corners.scale.y = 0.1;
        corners.color.b = 1.0f;
        corners.color.a = 1.0;
        corners.pose.orientation.w = 1.0;
        corners.header.frame_id = "/map";
        corners.header.stamp = ros::Time::now();
        corners.ns = "corners";
        corners.id = 0;
        
        for(u16 idx=0;idx<feats.corners.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.corners[idx].x_mm/100;
            p.y=feats.corners[idx].y_mm/100;
            p.z=0.2;
            corners.points.push_back(p);
        }

        rvizCornersPub.publish(corners);

        // edges visualization
        visualization_msgs::Marker edges;
        edges.action = visualization_msgs::Marker::ADD;
        edges.type = visualization_msgs::Marker::POINTS;
        edges.scale.x = 0.1;
        edges.scale.y = 0.1;
        edges.color.b = 1.0f;
        edges.color.a = 1.0;
        edges.color.r = 1.0f;
        edges.color.g = 1.0;
        edges.pose.orientation.w = 1.0;
        edges.header.frame_id = "/map";
        edges.header.stamp = ros::Time::now();
        edges.ns = "edges";
        edges.id = 0;
        
        for(u16 idx=0;idx<feats.edges.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.edges[idx].x_mm/100;
            p.y=feats.edges[idx].y_mm/100;
            p.z=0.2;
            edges.points.push_back(p);
        }

        rvizEdgesPub.publish(edges);

        // pos visualization
        geometry_msgs::PoseStamped rviz_pos;
        rviz_pos.header.frame_id = "/map";
        rviz_pos.header.stamp = ros::Time::now();
        rviz_pos.pose.position.x = pos.x_mm/100;
        rviz_pos.pose.position.y = pos.y_mm/100;
        rviz_pos.pose.orientation.x=cos(pos.theta_rad/2);
        rviz_pos.pose.orientation.y=sin(pos.theta_rad/2);

        rvizPosPub.publish(rviz_pos);

        // lines vectors
        /*geometry_msgs::PoseArray pos_arr;
        pos_arr.header.frame_id = "/map";
        pos_arr.header.stamp = ros::Time::now();
        for(int i=0;i<feats.lines.size();++i){
            geometry_msgs::Pose pose;
            pose.position.x = feats.lines[i].p1.x_mm/100;
            pose.position.y = feats.lines[i].p1.y_mm/100;
            pose.orientation.x=cos(feats.lines[i].angle/2);
            pose.orientation.y=sin(feats.lines[i].angle/2);
            pos_arr.poses.push_back(pose);
        }
        
        rvizPosArrPub.publish(pos_arr);*/
    }
    
    void exec(){
        lfx.exec();
    }

    void update() override{
        geometry_msgs::Pose2D msg;

        // set inputs
        {
            Lock lock(posMux);
            lfxInData.pos=pos;
        }
        {
            Lock lock(scanMux);
            lfxInData.scan=scanData;
        }

        // execute
        exec();

        // send outputs
        displayFeats(lfxOutData.feats,lfxInData.pos);
        
        //featPub.publish(msg);
    }
     
    static void storePos(const geometry_msgs::Pose2D& msg){
        Lock mux(posMux);
        pos.x_mm = msg.x;
        pos.y_mm = msg.y;
        pos.theta_rad =msg.theta;
    }

    static void storeScan(const sensor_msgs::LaserScan& msg){
        Lock mux(scanMux);
        for(u16 idx=0;idx<msg.ranges.size();idx++){
            scanData[idx]=static_cast<u16>(msg.ranges[idx]);
        }
    }
public:
    MazeRunnerNode(): 
        RosNodeBase("mazeRunner"),
        lfx(lfxInData,lfxOutData),
        robotPosSub(n.subscribe("robotPos",100,storePos)),
        ldsScanSub(n.subscribe("ldsScan",100,storeScan)),
        posDataPub(n.advertise<geometry_msgs::Pose2D>("robotPos",1000)),
        rvizPointsPub(n.advertise<visualization_msgs::Marker>("rvizPoints", 1000)),
        rvizLinesPub(n.advertise<visualization_msgs::Marker>("rvizLines", 1000)),
        rvizCornersPub(n.advertise<visualization_msgs::Marker>("rvizCorners", 1000)),
        rvizEdgesPub(n.advertise<visualization_msgs::Marker>("rvizEdges", 1000)),
        rvizPosArrPub(n.advertise<geometry_msgs::PoseArray>("rvizLinesVectors", 1000)),
        rvizPosPub(n.advertise<geometry_msgs::PoseStamped>("rvizPos", 1000)){}
private:
    Subscriber robotPosSub;
    Subscriber ldsScanSub;
    Publisher posDataPub;
    Publisher rvizPointsPub;
    Publisher rvizLinesPub;
    Publisher rvizCornersPub;
    Publisher rvizEdgesPub;
    Publisher rvizPosPub;
    Publisher rvizPosArrPub;
    LfxInputData lfxInData;
    LfxOutputData lfxOutData;
    LfxFunc lfx;
    static mutex posMux;
    static mutex scanMux;
    static RobotPos pos;
    static vector<u16> scanData;
};

mutex MazeRunnerNode::posMux;
mutex MazeRunnerNode::scanMux;
RobotPos MazeRunnerNode::pos(0,0,M_PI_2);
vector<u16> MazeRunnerNode::scanData(360,0);

#endif