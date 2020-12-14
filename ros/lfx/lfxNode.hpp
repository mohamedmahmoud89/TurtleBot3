#ifndef LFX_NODE_HPP
#define LFX_NODE_HPP

#include "nodeBase.hpp"
#include <geometry_msgs/PoseStamped.h>
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

struct LfxFeats{
    vector<Point2D> points;
    vector<Line2D> lines;
    vector<Point2D> corners;
    vector<Point2D> edges;
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

        //featPub.publish(msg);
    }

    void extractFeats(
            const RobotPos& pos,
            const vector<u16>& scan,
            vector<geometry_msgs::Pose>& feats){
        LfxFeats lfx_feats;
        vector<Point2DPolar> valid_scan;
        vector<Point2DPolar> line_params;

        // create points out of the scan
        for(u16 idx=0;idx<scan.size();idx++){
            if(scan[idx]>=LdsSensorCfg::minDepth_mm && scan[idx]<=LdsSensorCfg::maxDepth_mm){
                f32 range=scan[idx]-LdsSensorCfg::sensorOffset_mm;
                f32 x=pos.x_mm + (range*cos(idx*AngConversions::degToRad));
                f32 y=pos.y_mm + (range*sin(idx*AngConversions::degToRad));
                lfx_feats.points.push_back(Point2D(x,y));
                valid_scan.push_back(Point2DPolar(scan[idx],(idx)*AngConversions::degToRad));
            }
        }

        segmentScan(valid_scan,line_params);
        for(u16 i=0;i<line_params.size();++i){
            auto line_param=line_params[i];
            Line2D l;
            l.p1.x_mm=pos.x_mm;
            l.p1.y_mm=pos.y_mm;
            l.p2.x_mm=line_param.r_mm*cos(line_param.theta_rad);
            l.p2.y_mm=line_param.r_mm*sin(line_param.theta_rad);
            lfx_feats.lines.push_back(l);
        }
        
        displayFeats(lfx_feats,pos);
    }

    void segmentScan(
            const vector<Point2DPolar>& scan_segment,
            vector<Point2DPolar>& line_params){
        u16 split_idx=0;
        f32 max_norm_dist=0;
        bool is_fitting_ok=false;
        Point2DPolar temp_line_param;

        if(scan_segment.empty())
            return;
        
        fitLineLeastSquares(scan_segment,temp_line_param,max_norm_dist,split_idx); 

        if(max_norm_dist>normal_dist_threshold_mm){
            // split at slpit_idx and call segmentScan() twice 
            vector<Point2DPolar> seg_1;
            vector<Point2DPolar> seg_2;

            seg_1.insert(seg_1.end(),scan_segment.begin(),scan_segment.begin()+split_idx);
            seg_2.insert(seg_2.end(),scan_segment.begin()+split_idx+1,scan_segment.end());
            segmentScan(seg_1,line_params);
            segmentScan(seg_2,line_params);
            return;
        }

        line_params.push_back(temp_line_param);
    }

    void fitLineLeastSquares(
            const vector<Point2DPolar>& scan,
            Point2DPolar& line_params,
            f32& max_norm_dist,
            u16& max_norm_dist_idx){
        f64 first=0,second=0,third=0,fourth=0,sig=0;
        u16 M=scan.size();
        
        if(M){
            for(u16 i=0;i<M;++i){
                f64 p=pow(scan[i].r_mm,2);
                first+=(p*sin(2*scan[i].theta_rad));
                third+=(p*cos(2*scan[i].theta_rad));

                for(u16 j=0;j<M;++j){
                    second+=(scan[i].r_mm*scan[j].r_mm*cos(scan[i].theta_rad)*sin(scan[j].theta_rad));
                    fourth+=(scan[i].r_mm*scan[j].r_mm*cos(scan[i].theta_rad+scan[j].theta_rad));
                }
            }

            line_params.theta_rad=0.5*atan2((second*2.0/M)-first,(fourth/M)-third);

            for(u16 i=0;i<M;++i){
                sig+=scan[i].r_mm*cos(scan[i].theta_rad-line_params.theta_rad);
            }

            line_params.r_mm=sig/M;
            line_params.r_mm>0?line_params.r_mm-=20:line_params.r_mm+=20;

            for(u16 i=0;i<M;++i){
                f32 norm_dist=scan[i].r_mm*cos(scan[i].theta_rad-line_params.theta_rad) - line_params.r_mm;
                if(fabs(norm_dist)>max_norm_dist){
                    max_norm_dist=fabs(norm_dist);
                    max_norm_dist_idx=i;
                }
            }
        }
    }

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

        // pos visualization
        geometry_msgs::PoseStamped rviz_pos;
        rviz_pos.header.frame_id = "/map";
        rviz_pos.header.stamp = ros::Time::now();
        rviz_pos.pose.position.x = pos.x_mm/100;
        rviz_pos.pose.position.y = pos.y_mm/100;
        rviz_pos.pose.orientation.x=cos(pos.theta_rad/2);
        rviz_pos.pose.orientation.y=sin(pos.theta_rad/2);

        rvizPosPub.publish(rviz_pos);
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
            scan[idx]=static_cast<u16>(msg.ranges[idx]);
        }
    }
public:
    LfxNode():
        RosNodeBase(),
        robotPosSub(n.subscribe("robotPos",100,storePos)),
        ldsScanSub(n.subscribe("ldsScan",100,storeScan)),
        rvizPointsPub(n.advertise<visualization_msgs::Marker>("rvizPoints", 1000)),
        rvizLinesPub(n.advertise<visualization_msgs::Marker>("rvizLines", 1000)),
        rvizPosPub(n.advertise<geometry_msgs::PoseStamped>("rvizPos", 1000)){}
private:
    Subscriber robotPosSub;
    Subscriber ldsScanSub;
    Publisher featPub;
    Publisher rvizPointsPub;
    Publisher rvizLinesPub;
    Publisher rvizPosPub;
    static RobotPos pos;
    static vector<u16> scan; 
    static mutex posMux;
    static mutex scanMux;
    static const u16 normal_dist_threshold_mm=50;
};

RobotPos LfxNode::pos(0,0,M_PI_2);
vector<u16> LfxNode::scan(360,0);
mutex LfxNode::posMux;
mutex LfxNode::scanMux;

#endif