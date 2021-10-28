#ifndef DISPLAY_NODE_HPP
#define DISPLAY_NODE_HPP

#include "nodeBase.hpp"
#include "feat.hpp"
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
            featPoints.points.clear();
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
            featEdges.points.clear();
        }

        // corners visualization
        {
            Lock l(featsCornerMux);
            featCorners.header.stamp = ros::Time::now();
            rvizCornersPub.publish(featCorners);
            featCorners.points.clear();
        }

        // pos visualization
        {
            Lock mux(posMux);
            pos.header.stamp = ros::Time::now();
            rvizPosPub.publish(pos);
        }

        // global pos visualization
        {
            Lock mux(globalPosMux);
            globalPos.header.stamp = ros::Time::now();
            globalPos.pose.position.x=globalRobotPos.x_mm/100;
            globalPos.pose.position.y=globalRobotPos.y_mm/100;
            globalPos.pose.orientation.x=cos(globalRobotPos.theta_rad/2);
            globalPos.pose.orientation.y=sin(globalRobotPos.theta_rad/2);
            rvizGlobalPosPub.publish(globalPos);
        }

        // particles visualization
        {
            Lock mux(mclParticlesMux);
            particles.header.stamp = ros::Time::now();
            rvizParticlesPub.publish(particles);
        }

        // landmarks virusalization
        landmarkPoints.header.stamp = ros::Time::now();
        rvizLandmarksPub.publish(landmarkPoints);

        // path visualization
        {
            Lock l(pathMux);
            path.header.stamp = ros::Time::now();
            pathPub.publish(path);
        }
    }

    static void processMclPosData(const geometry_msgs::Pose2D& msg){
        Lock mux(globalPosMux);
        globalRobotPos.x_mm = msg.x;
        globalRobotPos.y_mm = msg.y;
        globalRobotPos.theta_rad=msg.theta;
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
        RobotPos gpos;
        {
            Lock l2(globalPosMux);
            gpos = globalRobotPos;
        }

        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x;
            p.y=pt.position.y;
            p.z=0.2;
            Point2D temp(p.x,p.y);
            Point2D ret=calcFeatGlobalPos(temp,gpos);
            p.x=ret.x_mm/100;
            p.y=ret.y_mm/100;
            featPoints.points.push_back(p);
        }
    }

    static void processLfxLineData(const geometry_msgs::PoseArray& msg){
        Lock l(featsLineMux);
        featLines.points.clear();
        featLines.points.reserve(msg.poses.size());
        RobotPos gpos;
        {
            Lock l2(globalPosMux);
            gpos = globalRobotPos;
        }

        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x;
            p.y=pt.position.y;
            p.z=0.2;
            Point2D temp(p.x,p.y);
            Point2D ret=calcFeatGlobalPos(temp,gpos);
            p.x=ret.x_mm/100;
            p.y=ret.y_mm/100;
            featLines.points.push_back(p);
        }
    }

    static void processPathData(const geometry_msgs::PoseArray& msg){
        Lock l(pathMux);
        path.points.clear();
        path.points.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x/100;
            p.y=pt.position.y/100;
            path.points.push_back(p);
        }
    }

    static void processLfxCornerData(const geometry_msgs::PoseArray& msg){
        Lock l(featsCornerMux);
        featCorners.points.clear();
        featCorners.points.reserve(msg.poses.size());
        RobotPos gpos;
        {
            Lock l2(globalPosMux);
            gpos = globalRobotPos;
        }
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x;
            p.y=pt.position.y;
            Point2D temp(p.x,p.y);
            Point2D ret=calcFeatGlobalPos(temp,gpos);
            p.x=ret.x_mm/100;
            p.y=ret.y_mm/100;
            featCorners.points.push_back(p);
        }
    }

    static void processLfxEdgeData(const geometry_msgs::PoseArray& msg){
        Lock l(featsEdgeMux);
        featEdges.points.clear();
        featEdges.points.reserve(msg.poses.size());
        RobotPos gpos;
        {
            Lock l2(globalPosMux);
            gpos = globalRobotPos;
        }
        for(auto& pt:msg.poses){
            geometry_msgs::Point p;
            p.x=pt.position.x;
            p.y=pt.position.y;
            Point2D temp(p.x,p.y);
            Point2D ret=calcFeatGlobalPos(temp,gpos);
            p.x=ret.x_mm/100;
            p.y=ret.y_mm/100;
            featEdges.points.push_back(p);
        }
    }

    static void processMclParticlesData(const geometry_msgs::PoseArray& msg){
        Lock l(mclParticlesMux);
        particles.poses.clear();
        particles.poses.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            geometry_msgs::Pose p;
            p.position.x=pt.position.x/100;
            p.position.y=pt.position.y/100;
            p.orientation.x=cos(pt.orientation.z/2);
            p.orientation.y=sin(pt.orientation.z/2);
            particles.poses.push_back(p);
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
        mclPosDataSub(n.subscribe("globalPos",100,processMclPosData)),
        mclParticlesDataSub(n.subscribe("particles",100,processMclParticlesData)),
        pathDataSub(n.subscribe("path",100,processPathData)),
        rvizPointsPub(n.advertise<visualization_msgs::Marker>("rvizPoints", 1000)),
        rvizLinesPub(n.advertise<visualization_msgs::Marker>("rvizLines", 1000)),
        rvizCornersPub(n.advertise<visualization_msgs::Marker>("rvizCorners", 1000)),
        rvizLandmarksPub(n.advertise<visualization_msgs::Marker>("rvizLandmarks", 1000)),
        rvizEdgesPub(n.advertise<visualization_msgs::Marker>("rvizEdges", 1000)),
        rvizPosPub(n.advertise<geometry_msgs::PoseStamped>("rvizPos", 1000)),
        rvizGlobalPosPub(n.advertise<geometry_msgs::PoseStamped>("rvizGlobalPos", 1000)),
        rvizParticlesPub(n.advertise<geometry_msgs::PoseArray>("rvizParticles", 1000)),
        pathPub(n.advertise<visualization_msgs::Marker>("rvizPath", 1000)){}

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

        landmarkPoints.action = visualization_msgs::Marker::ADD;
        landmarkPoints.type = visualization_msgs::Marker::POINTS;
        landmarkPoints.scale.x = 0.05;
        landmarkPoints.scale.y = 0.05;
        landmarkPoints.color.g = 1.0f;
        landmarkPoints.color.a = 1.0;
        landmarkPoints.pose.orientation.w = 1.0;
        landmarkPoints.header.frame_id = "/map";
        landmarkPoints.ns = "points";
        landmarkPoints.id = 0;
        landmarkPoints.points.reserve(Landmarks::corners.size());
        for(auto& pt:Landmarks::corners){
            geometry_msgs::Point p;
            p.x=pt.x_mm/100;
            p.y=pt.y_mm/100;
            landmarkPoints.points.push_back(p);
        }

        pos.header.frame_id = "/map";

        globalPos.header.frame_id = "/map";

        particles.header.frame_id = "/map";

        featLines.action = visualization_msgs::Marker::ADD;
        featLines.type = visualization_msgs::Marker::LINE_LIST;
        featLines.scale.x = 0.07;
        featLines.color.r = 1.0;
        featLines.color.a = 1.0;
        featLines.pose.orientation.w = 1.0;
        featLines.header.frame_id = "/map";
        featLines.id = 1;

        path.action = visualization_msgs::Marker::ADD;
        path.type = visualization_msgs::Marker::LINE_LIST;
        path.scale.x = 0.07;
        path.color.g = 1.0;
        path.color.a = 1.0;
        path.pose.orientation.w = 1.0;
        path.header.frame_id = "/map";
        path.id = 2;

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
    Subscriber mclPosDataSub;
    Subscriber mclParticlesDataSub;
    Subscriber pathDataSub;
    Publisher rvizPointsPub;
    Publisher rvizLinesPub;
    Publisher rvizCornersPub;
    Publisher rvizEdgesPub;
    Publisher rvizPosPub;
    Publisher rvizParticlesPub;
    Publisher rvizGlobalPosPub;
    Publisher rvizLandmarksPub;
    Publisher pathPub;
    visualization_msgs::Marker landmarkPoints;
    static constexpr u16 rate_hz=10;
    static mutex posMux;
    static RobotPos globalRobotPos;
    static mutex globalPosMux;
    static mutex featsMux;
    static mutex featsLineMux;
    static mutex featsCornerMux;
    static mutex featsEdgeMux;
    static mutex mclParticlesMux;
    static mutex pathMux;
    static visualization_msgs::Marker featPoints;
    static visualization_msgs::Marker featLines;
    static visualization_msgs::Marker featCorners;
    static visualization_msgs::Marker featEdges;
    static geometry_msgs::PoseStamped pos;
    static geometry_msgs::PoseStamped globalPos;
    static geometry_msgs::PoseArray particles;
    static visualization_msgs::Marker path;
};

visualization_msgs::Marker DisplayNode::featPoints;
visualization_msgs::Marker DisplayNode::featLines;
visualization_msgs::Marker DisplayNode::featCorners;
visualization_msgs::Marker DisplayNode::featEdges;
geometry_msgs::PoseStamped DisplayNode::pos;
geometry_msgs::PoseStamped DisplayNode::globalPos;
geometry_msgs::PoseArray DisplayNode::particles;
visualization_msgs::Marker DisplayNode::path;
mutex DisplayNode::posMux;
mutex DisplayNode::globalPosMux;
mutex DisplayNode::featsMux;
mutex DisplayNode::featsLineMux;
mutex DisplayNode::featsCornerMux;
mutex DisplayNode::featsEdgeMux;
mutex DisplayNode::mclParticlesMux;
mutex DisplayNode::pathMux;
RobotPos DisplayNode::globalRobotPos;

#endif