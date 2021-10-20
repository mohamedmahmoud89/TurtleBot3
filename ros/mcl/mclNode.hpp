#ifndef MCL_NODE_HPP
#define MCL_NODE_HPP

#include "nodeBase.hpp"
#include "feat.hpp"
#include "pf.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vector>

using namespace std;

struct MclNodeCfg{
    static constexpr f32 ctrl_motion_std{0.1};
    static constexpr f32 ctrl_turn_std{0.05};
    static constexpr f32 meas_x_std{50};
    static constexpr f32 meas_y_std{50};
    static constexpr u16 particles_num{5000};
    static constexpr u16 world_x_mm{880};
    static constexpr u16 world_y_mm{580};
    static constexpr u16 min_cov_x_y_loc_done{30};
};

class MclNode : public RosNodeBase{
    void update() override{
        WheelVelocity vel_copy(0,0);
        vector<Point2D>corners_copy;
        vector<Point2D>edges_copy;
        RobotPos pos,cov;

        {
            Lock l(velMux);
            vel_copy=vel;
        }

        {
            Lock l(cornersMux);
            corners_copy=featCorners;
        }

        {
            Lock l(edgesMux);
            edges_copy=featEdges;
        }

        if(reset_filter){
            pf.reset();
            reset_filter=false;
        }

        pf.predict(vel_copy);
        if((vel_copy.left_rpm&&vel_copy.right_rpm)&&(corners_copy.size()+edges_copy.size()>=2)){
            pf.update(corners_copy,edges_copy);
        }
        pf.getPosDensityParams(pos,cov);
        if(cov.x_mm<MclNodeCfg::min_cov_x_y_loc_done&&cov.y_mm<MclNodeCfg::min_cov_x_y_loc_done)
            loc_done=true;
        publish(pos,cov);
    }

    void publish(const RobotPos& pos,const RobotPos& cov){
        geometry_msgs::PoseArray particlesMsg;
        geometry_msgs::Pose2D globalPosMsg;
        std_msgs::Bool locStatMsg;
        
        globalPosMsg.x=pos.x_mm;
        globalPosMsg.y=pos.y_mm;
        globalPosMsg.theta=pos.theta_rad;
        globalPosPub.publish(globalPosMsg);

        locStatMsg.data=loc_done;
        locStatPub.publish(locStatMsg);
        
        // particles
        particlesMsg.poses.reserve(MclNodeCfg::particles_num);
        for(auto& pt:pf.getParticles()){
            geometry_msgs::Pose p;
            p.position.x=pt.x_mm;
            p.position.y=pt.y_mm;
            p.orientation.z=pt.theta_rad;
            particlesMsg.poses.push_back(p);
        }
        particlesPub.publish(particlesMsg);
    }

    static void storeCorners(const geometry_msgs::PoseArray& msg){
        Lock l(cornersMux);
        featCorners.clear();
        featCorners.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            featCorners.push_back(Point2D(pt.position.x,pt.position.y));
        }
    }

    static void storeEdges(const geometry_msgs::PoseArray& msg){
        Lock l(edgesMux);
        featEdges.clear();
        featEdges.reserve(msg.poses.size());
        for(auto& pt:msg.poses){
            featEdges.push_back(Point2D(pt.position.x,pt.position.y));
        }
    }

    static void storeVel(const std_msgs::Int16MultiArray& msg){
        Lock l(velMux);
        vel.left_rpm=msg.data[0];
        vel.right_rpm=msg.data[1];
    }

    static void resetFilter(const std_msgs::String& msg){
        if(msg.data=="reset"){
            reset_filter=true;
            loc_done=false;
        }
    }
public:
    MclNode():
        RosNodeBase("mcl"),
        ctrlDataSub(n.subscribe("odom",100,storeVel)),
        featCornerSub(n.subscribe("featCorners",100,storeCorners)),
        featEdgeSub(n.subscribe("featEdges",100,storeEdges)),
        hmiSub(n.subscribe("resetCmd",100,resetFilter)),
        globalPosPub(n.advertise<geometry_msgs::Pose2D>("globalPos", 1000)),
        locStatPub(n.advertise<std_msgs::Bool>("locStatus", 1000)),
        particlesPub(n.advertise<geometry_msgs::PoseArray>("particles", 1000)){}
private:
    Subscriber ctrlDataSub;
    Subscriber featCornerSub;
    Subscriber featEdgeSub;
    Subscriber hmiSub;
    Publisher globalPosPub;
    Publisher locStatPub;
    Publisher particlesPub;
    static ParticleFilter pf;
    static WheelVelocity vel;
    static vector<Point2D>featCorners;
    static vector<Point2D>featEdges;
    static mutex velMux;
    static mutex cornersMux;
    static mutex edgesMux;
    static bool reset_filter;
    static bool loc_done;
};

WheelVelocity MclNode::vel(0,0);
vector<Point2D>MclNode::featCorners;
vector<Point2D>MclNode::featEdges;
mutex MclNode::velMux;
mutex MclNode::cornersMux;
mutex MclNode::edgesMux;
ParticleFilter MclNode::pf(
            ParticleFilterInitList(
                MclNodeCfg::ctrl_motion_std,
                MclNodeCfg::ctrl_turn_std,
                MclNodeCfg::meas_x_std,
                MclNodeCfg::meas_y_std,
                MclNodeCfg::particles_num,
                MclNodeCfg::world_x_mm,
                MclNodeCfg::world_y_mm));
bool MclNode::reset_filter=false;
bool MclNode::loc_done=false;
#endif 