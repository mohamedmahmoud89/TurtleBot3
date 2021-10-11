#ifndef MCL_NODE_HPP
#define MCL_NODE_HPP

#include "nodeBase.hpp"
#include "feat.hpp"
#include "pf.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>

using namespace std;

const vector<Point2D> Landmarks::corners={};
const vector<Point2D> Landmarks::edges={};

struct MclNodeCfg{
    static constexpr f32 ctrl_motion_std{0};
    static constexpr f32 ctrl_turn_std{0};
    static constexpr f32 meas_x_std{0};
    static constexpr f32 meas_y_std{0};
    static constexpr u16 particles_num{0};
    static constexpr u16 world_width_mm{0};
    static constexpr u16 world_length_mm{0};
};

class MclNode : public RosNodeBase{
    void update() override{
        WheelVelocity vel_copy(0,0);
        vector<Point2D>corners_copy;
        vector<Point2D>edges_copy;
        geometry_msgs::Pose2D msg;
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
        pf.predict(vel_copy);
        pf.update(corners_copy,edges_copy);
        RobotPos pos=pf.getPosMean();
        msg.x=pos.x_mm;
        msg.y=pos.y_mm;
        msg.theta=pos.theta_rad;
        globalPosPub.publish(msg);
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
public:
    MclNode():
        RosNodeBase("mcl"),
        pf(
            ParticleFilterInitList(
                MclNodeCfg::ctrl_motion_std,
                MclNodeCfg::ctrl_turn_std,
                MclNodeCfg::meas_x_std,
                MclNodeCfg::meas_y_std,
                MclNodeCfg::particles_num,
                MclNodeCfg::world_length_mm,
                MclNodeCfg::world_width_mm)),
        robotPosSub(n.subscribe("odom",100,storeVel)),
        featCornerSub(n.subscribe("featCorners",100,storeCorners)),
        featEdgeSub(n.subscribe("featEdges",100,storeEdges)),
        globalPosPub(n.advertise<geometry_msgs::Pose2D>("globalPos", 1000)){}
private:
    Subscriber robotPosSub;
    Subscriber featCornerSub;
    Subscriber featEdgeSub;
    Publisher globalPosPub;
    ParticleFilter pf;
    static WheelVelocity vel;
    static vector<Point2D>featCorners;
    static vector<Point2D>featEdges;
    static mutex velMux;
    static mutex cornersMux;
    static mutex edgesMux;
};

WheelVelocity MclNode::vel(0,0);
vector<Point2D>MclNode::featCorners;
vector<Point2D>MclNode::featEdges;
mutex MclNode::velMux;
mutex MclNode::cornersMux;
mutex MclNode::edgesMux;

#endif 