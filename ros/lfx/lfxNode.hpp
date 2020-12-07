#ifndef LFX_NODE_HPP
#define LFX_NODE_HPP

#include "nodeBase.hpp"
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <visualization_msgs/Marker.h>

using namespace std;

enum LfxFeatType{
    CORNER=0,
    EDGE
};

class LfxNode : public RosNodeBase{
    void update() override{
        RobotPos loc_pos(0,0,0);
        vector<u16> loc_scan;
        geometry_msgs::PoseArray msg;
        geometry_msgs::Pose feat;
        
        {
            Lock mux(posMux);
            loc_pos=pos;
        }
        {
            Lock mux(scanMux);
            loc_scan=scan;
        }

        extractFeats(loc_pos,loc_scan,msg.poses);

        featPub.publish(msg);
    }

    void extractFeats(
            const RobotPos& pos,
            const vector<u16>& scan,
            vector<geometry_msgs::Pose>& feats){

        // visualization
        visualization_msgs::Marker points;
        points.type = visualization_msgs::Marker::POINTS;
        for(u16 idx=0;idx<scan.size();++idx){
            if(scan[idx]>=LdsSensorCfg::minDepth_mm && scan[idx]<=LdsSensorCfg::maxDepth_mm){
                geometry_msgs::Point p;
                f32 range=scan[idx]-LdsSensorCfg::sensorOffset_mm;
                p.x=pos.x_mm + (range*cos(idx*AngConversions::degToRad));
                p.y=pos.y_mm + (range*sin(idx*AngConversions::degToRad));
                p.z=200;
                points.points.push_back(p);
            }
        }

        pointPub.publish(points);
    }

    static void storePos(const geometry_msgs::Pose2D& msg){
        Lock mux(posMux);
        pos.x_mm = msg.x;
        pos.y_mm = msg.y;
        pos.theta_rad = msg.theta;
    }

    static void storeScan(const sensor_msgs::LaserScan& msg){
        Lock mux(scanMux);
        for(u16 idx=0;idx<msg.ranges.size();idx++){
            scan[idx]=static_cast<u16>(msg.ranges[idx]);
        }
    }
public:
    LfxNode():
        RosNodeBase(),
        robotPosSub(n.subscribe("robotPos",100,storePos)),
        ldsScanSub(n.subscribe("ldsScan",100,storeScan)),
        pointPub(n.advertise<visualization_msgs::Marker>("rvizPoints", 1000)){}
private:
    Subscriber robotPosSub;
    Subscriber ldsScanSub;
    Publisher featPub;
    Publisher pointPub;
    static RobotPos pos;
    static vector<u16> scan; 
    static mutex posMux;
    static mutex scanMux;
};

RobotPos LfxNode::pos(0,0,M_PI_2);
vector<u16> LfxNode::scan(0,360);
mutex LfxNode::posMux;
mutex LfxNode::scanMux;

#endif