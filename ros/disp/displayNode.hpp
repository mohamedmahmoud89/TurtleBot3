#ifndef DISPLAY_NODE_HPP
#define DISPLAY_NODE_HPP

#include "nodeBase.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

class DisplayNode : public RosNodeBase{
    void update() override{
        // points visualization
        {
            Lock l(featsMux);
            featPoints.header.stamp = ros::Time::now();
            rvizPointsPub.publish(featPoints);
        }

        // lines visualization
        {
            Lock l(featsLineMux);
            featLines.header.stamp = ros::Time::now();
            rvizLinesPub.publish(featLines);
        }

        // edges visualization
        {
            Lock l(featsEdgeMux);
            featEdges.header.stamp = ros::Time::now();
            rvizEdgesPub.publish(featEdges);
        }

        // corners visualization
        {
            Lock l(featsCornerMux);
            featCorners.header.stamp = ros::Time::now();
            rvizCornersPub.publish(featCorners);
        }

        // pos visualization
        {
            Lock mux(posMux);
            pos.header.stamp = ros::Time::now();
            rvizPosPub.publish(pos);
        }
    }

    static void processOdomData(const geometry_msgs::Pose2D& msg){
        Lock mux(posMux);
        pos.pose.position.x = msg.x/100;
        pos.pose.position.y = msg.y/100;
        pos.pose.orientation.x=cos(msg.theta/2);
        pos.pose.orientation.y=sin(msg.theta/2);
    }
   
    static void processLfxData(const geometry_msgs::PoseArray& msg){
        Lock l(featsMux);
        featPoints.points.clear();
        featPoints.points.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x/100;
            p.y=pt.position.y/100;
            p.z=0.2;
            featPoints.points.push_back(p);
        }
    }

    static void processLfxLineData(const geometry_msgs::PoseArray& msg){
        Lock l(featsLineMux);
        featLines.points.clear();
        featLines.points.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x/100;
            p.y=pt.position.y/100;
            p.z=0.2;
            featLines.points.push_back(p);
        }
    }

    static void processLfxCornerData(const geometry_msgs::PoseArray& msg){
        Lock l(featsCornerMux);
        featCorners.points.clear();
        featCorners.points.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x/100;
            p.y=pt.position.y/100;
            p.z=0.2;
            featCorners.points.push_back(p);
        }
    }

    static void processLfxEdgeData(const geometry_msgs::PoseArray& msg){
        Lock l(featsEdgeMux);
        featEdges.points.clear();
        featEdges.points.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x/100;
            p.y=pt.position.y/100;
            p.z=0.2;
            featEdges.points.push_back(p);
        }
    }
public:
    DisplayNode():
        RosNodeBase(rate_hz,"display"),
        odomDataSub(n.subscribe("robotPos",100,processOdomData)),
        lfxDataSub(n.subscribe("featPoints",100,processLfxData)),
        lfxLineDataSub(n.subscribe("featLines",100,processLfxLineData)),
        lfxCornerDataSub(n.subscribe("featCorners",100,processLfxCornerData)),
        lfxEdgeDataSub(n.subscribe("featEdges",100,processLfxEdgeData)),
        rvizPointsPub(n.advertise<visualization_msgs::Marker>("rvizPoints", 1000)),
        rvizLinesPub(n.advertise<visualization_msgs::Marker>("rvizLines", 1000)),
        rvizCornersPub(n.advertise<visualization_msgs::Marker>("rvizCorners", 1000)),
        rvizEdgesPub(n.advertise<visualization_msgs::Marker>("rvizEdges", 1000)),
        rvizPosPub(n.advertise<geometry_msgs::PoseStamped>("rvizPos", 1000)){}

    void init(){
        featPoints.action = visualization_msgs::Marker::ADD;
        featPoints.type = visualization_msgs::Marker::POINTS;
        featPoints.scale.x = 0.05;
        featPoints.scale.y = 0.05;
        featPoints.color.g = 1.0f;
        featPoints.color.a = 1.0;
        featPoints.pose.orientation.w = 1.0;
        featPoints.header.frame_id = "/map";
        featPoints.ns = "points";
        featPoints.id = 0;

        pos.header.frame_id = "/map";

        featLines.action = visualization_msgs::Marker::ADD;
        featLines.type = visualization_msgs::Marker::LINE_LIST;
        featLines.scale.x = 0.07;
        featLines.color.r = 1.0;
        featLines.color.a = 1.0;
        featLines.pose.orientation.w = 1.0;
        featLines.header.frame_id = "/map";
        featLines.id = 1;

        featCorners.action = visualization_msgs::Marker::ADD;
        featCorners.type = visualization_msgs::Marker::POINTS;
        featCorners.scale.x = 0.1;
        featCorners.scale.y = 0.1;
        featCorners.color.b = 1.0f;
        featCorners.color.a = 1.0;
        featCorners.pose.orientation.w = 1.0;
        featCorners.header.frame_id = "/map";
        featCorners.ns = "corners";
        featCorners.id = 0;

        featEdges.action = visualization_msgs::Marker::ADD;
        featEdges.type = visualization_msgs::Marker::POINTS;
        featEdges.scale.x = 0.1;
        featEdges.scale.y = 0.1;
        featEdges.color.b = 1.0f;
        featEdges.color.a = 1.0;
        featEdges.color.r = 1.0f;
        featEdges.color.g = 1.0;
        featEdges.pose.orientation.w = 1.0;
        featEdges.header.frame_id = "/map";
        featEdges.ns = "edges";
        featEdges.id = 0;
    }

private:
    Subscriber odomDataSub;
    Subscriber lfxDataSub;
    Subscriber lfxLineDataSub;
    Subscriber lfxCornerDataSub;
    Subscriber lfxEdgeDataSub;
    Publisher rvizPointsPub;
    Publisher rvizLinesPub;
    Publisher rvizCornersPub;
    Publisher rvizEdgesPub;
    Publisher rvizPosPub;
    static constexpr u16 rate_hz=10;
    static mutex posMux;
    static mutex featsMux;
    static mutex featsLineMux;
    static mutex featsCornerMux;
    static mutex featsEdgeMux;
    static visualization_msgs::Marker featPoints;
    static visualization_msgs::Marker featLines;
    static visualization_msgs::Marker featCorners;
    static visualization_msgs::Marker featEdges;
    static geometry_msgs::PoseStamped pos;
};

visualization_msgs::Marker DisplayNode::featPoints;
visualization_msgs::Marker DisplayNode::featLines;
visualization_msgs::Marker DisplayNode::featCorners;
visualization_msgs::Marker DisplayNode::featEdges;
geometry_msgs::PoseStamped DisplayNode::pos;
mutex DisplayNode::posMux;
mutex DisplayNode::featsMux;
mutex DisplayNode::featsLineMux;
mutex DisplayNode::featsCornerMux;
mutex DisplayNode::featsEdgeMux;

#endif