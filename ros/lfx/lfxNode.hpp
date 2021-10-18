#ifndef LFX_NODE_HPP
#define LFX_NODE_HPP

#include "nodeBase.hpp"
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>

using namespace std;

struct LfxCfg{
    static const u16 normal_dist_threshold_mm{25};
    static const u16 max_dist_adj_pts_mm{10};
    static const u16 min_pts_for_line_fitting{5};
    static const u16 min_pts_gap_between_lines{10};
    static const u16 max_dist_intersect_line_line_mm{200};
};

class LfxNode : public RosNodeBase{
    void update() override{
        RobotPos loc_pos(0,0,M_PI_2);
        vector<u16> loc_scan;
        vector<u16> loc_intensity;
        LfxFeats feats;

        {
            Lock mux(scanMux);
            loc_scan=scan;
            loc_intensity=intensity;
        }

        extractFeats(loc_pos,loc_scan,loc_intensity,feats) ;
        publish(feats);
        
    }

    void publish(const LfxFeats& feats){
        geometry_msgs::PoseArray msg_pts;
        geometry_msgs::PoseArray msg_lines;
        geometry_msgs::PoseArray msg_corners;
        geometry_msgs::PoseArray msg_edges;
        
        // points
        msg_pts.poses.reserve(feats.points.size());
        for(auto& pt:feats.points){
            geometry_msgs::Pose p;
            p.position.x=pt.x_mm;
            p.position.y=pt.y_mm;
            msg_pts.poses.push_back(p);
        }
        featPtPub.publish(msg_pts);

        // lines
        msg_lines.poses.reserve(feats.lines.size()<<1);
        for(auto&l:feats.lines){
            geometry_msgs::Pose p;
            p.position.x=l.p1.x_mm;
            p.position.y=l.p1.y_mm;
            msg_lines.poses.push_back(p);
            p.position.x=l.p2.x_mm;
            p.position.y=l.p2.y_mm;
            msg_lines.poses.push_back(p);
        }
        featLnPub.publish(msg_lines);

        // corners
        msg_corners.poses.reserve(feats.corners.size());
        for(auto& pt:feats.corners){
            geometry_msgs::Pose p;
            p.position.x=pt.x_mm;
            p.position.y=pt.y_mm;
            msg_corners.poses.push_back(p);
        }
        featCornerPub.publish(msg_corners);

        // edges
        msg_edges.poses.reserve(feats.edges.size());
        for(auto& pt:feats.edges){
            geometry_msgs::Pose p;
            p.position.x=pt.x_mm;
            p.position.y=pt.y_mm;
            msg_edges.poses.push_back(p);
        }
        featEdgePub.publish(msg_edges);
    }

    inline bool calculateIntersection(const Line2D& l1,const Line2D& l2,Point2D& t){
        if(l1.p1.x_mm-l1.p2.x_mm!=0&&l2.p1.x_mm-l2.p2.x_mm!=0){
            f32 m1=((l1.p1.y_mm-l1.p2.y_mm)/(l1.p1.x_mm-l1.p2.x_mm));
            f32 b1=l1.p1.y_mm-(m1*l1.p1.x_mm);

            f32 m2=((l2.p1.y_mm-l2.p2.y_mm)/(l2.p1.x_mm-l2.p2.x_mm));
            f32 b2=l2.p1.y_mm-(m2*l2.p1.x_mm);

            if(m1-m2!=0){
                t.x_mm=((b2-b1)/(m1-m2));
                t.y_mm=(m1*t.x_mm)+b1;
                return true;
            }
        }
        return false;
    }

    void extractFeats(
            const RobotPos& pos,
            const vector<u16>& scan,
            const vector<u16>& intensity,
            LfxFeats& lfx_feats){
        vector<vector<Point2DPolar>> valid_scan;
        

        // extract and pre-filter points
        u16 idx=0;      
        si16 last_r_mm=0;
        bool last_r_initialized=false;
        while(idx<scan.size()){
            vector<Point2DPolar> temp;
            u16 count=0;
            while(
                    idx<scan.size()&&count<LfxCfg::min_pts_gap_between_lines&&
                    scan[idx]>=LdsSensorCfg::minDepth_mm&&
                    scan[idx]<=LdsSensorCfg::maxDepth_mm&&
                    (
                        abs(si16(scan[idx])-last_r_mm)<LfxCfg::max_dist_adj_pts_mm||
                        !last_r_initialized
                    )
                )
            {
                if(intensity[idx]>=LdsSensorCfg::minIntensity){
                    f32 range=scan[idx]-LdsSensorCfg::sensorOffset_mm;
                    f32 ang=pos.theta_rad+(((idx+180)%360)*AngConversions::degToRad);
                    ang=fmod(ang+(2*M_PI),2*M_PI);
                    //f32 ang=(idx*AngConversions::degToRad);
                    f32 x=pos.x_mm + (range*cos(ang));
                    f32 y=pos.y_mm + (range*sin(ang));
                    lfx_feats.points.push_back(Point2D(x,y));
                    temp.push_back(Point2DPolar(scan[idx],ang));
                    count=0;
                }
                else
                {
                    count++;
                }
                last_r_mm=scan[idx];
                last_r_initialized=true;
                idx++;
            }
            if(temp.size()>LfxCfg::min_pts_for_line_fitting)
                valid_scan.push_back(temp);
            while(
                    idx<scan.size()&&
                    (
                        intensity[idx]<LdsSensorCfg::minIntensity||
                        scan[idx]<LdsSensorCfg::minDepth_mm||
                        scan[idx]>LdsSensorCfg::maxDepth_mm||
                        abs(si16(scan[idx])-last_r_mm)>=LfxCfg::max_dist_adj_pts_mm
                    )
                ){
                last_r_mm=scan[idx];
                idx++;
            }
        }

        // extract lines
        for(auto& set:valid_scan){
            vector<Line2DPolar> line_params;
            segmentScan(set,0,set.size()-1,line_params);
            for(u16 i=0;i<line_params.size();++i){
                constructLineFeat(pos,line_params[i],lfx_feats);
            }
        }

        // extract corners/edges
        for(u16 i=0;i<lfx_feats.lines.size();++i){
            u16 j=(i+1)%lfx_feats.lines.size();
            auto l1=lfx_feats.lines[i];
            auto l2=lfx_feats.lines[j];
            // calc dist & delta angle between each 2 consecutive lines
            f32 dist=sqrt(pow(l1.p2.x_mm-l2.p1.x_mm,2)+pow(l1.p2.y_mm-l2.p1.y_mm,2));
            f32 ang=l2.angle-l1.angle;

            // normalize the ang
            ang=fmod(ang+(2*M_PI),2*M_PI);

            // check whether the 2 lines form corner/edge
            if(dist<LfxCfg::max_dist_intersect_line_line_mm){
                if((ang>70*AngConversions::degToRad) && (ang<110*AngConversions::degToRad)){
                    // calculate intersections
                    Point2D t;
                    if(calculateIntersection(l1,l2,t))
                        lfx_feats.corners.push_back(t);
                }
                else if((ang>250*AngConversions::degToRad) && (ang<290*AngConversions::degToRad)){
                    // calculate intersections
                    Point2D t;
                    if(calculateIntersection(l1,l2,t))
                        lfx_feats.edges.push_back(t);
                }
            }
        }
    }

    inline void constructLineFeat(const RobotPos& pos,const Line2DPolar& line_param,LfxFeats& lfx_feats){
        f32 r=line_param.center.r_mm;
        f32 theta=line_param.center.theta_rad;
        Line2D l;
        f32 d_start=r*tan(theta-line_param.start_ang_rad);
        f32 d_end  =r*tan(line_param.end_ang_rad-theta);
        f32 ang_start= theta-M_PI_2;
        f32 ang_end= theta+M_PI_2;
        f32 center_x=pos.x_mm+r*cos(theta);
        f32 center_y=pos.y_mm+r*sin(theta);
        l.angle=theta+M_PI;
        l.angle=fmod(l.angle+(2*M_PI),2*M_PI);
        l.p1.x_mm=center_x+d_start*cos(ang_start);
        l.p1.y_mm=center_y+d_start*sin(ang_start);
        l.p2.x_mm=center_x+d_end*cos(ang_end);
        l.p2.y_mm=center_y+d_end*sin(ang_end);
        lfx_feats.lines.push_back(l);
        lfx_feats.lines_polar.push_back(line_param);
    }

    void segmentScan(
            const vector<Point2DPolar>& scan_segment,
            const u16 start_idx,
            const u16 end_idx,
            vector<Line2DPolar>& line_params){
        u16 split_idx=0;
        f32 max_norm_dist=0;
        bool is_fitting_ok=false;
        Line2DPolar temp_line_param;

        if(end_idx-start_idx<2)
            return;

        fitLineLeastSquares(scan_segment,start_idx,end_idx,temp_line_param,max_norm_dist,split_idx); 

        if(max_norm_dist>LfxCfg::normal_dist_threshold_mm){
            // split at slpit_idx and call segmentScan() twice 
            segmentScan(scan_segment,start_idx,split_idx,line_params);
            segmentScan(scan_segment,split_idx+1,end_idx,line_params);
            return;
        }

        line_params.push_back(temp_line_param);
    }

    inline void fitLineLeastSquares(
            const vector<Point2DPolar>& scan,
            const u16 start_idx,
            const u16 end_idx,
            Line2DPolar& line_params,
            f32& max_norm_dist,
            u16& max_norm_dist_idx){
        f64 first=0,second=0,third=0,fourth=0,sig=0;
        u16 M=end_idx-start_idx+1;
        
        if(M){
            for(u16 i=start_idx;i<=end_idx;++i){
                f64 p=pow(scan[i].r_mm,2);
                first+=(p*sin(2*scan[i].theta_rad));
                third+=(p*cos(2*scan[i].theta_rad));

                for(u16 j=start_idx;j<=end_idx;++j){
                    second+=(scan[i].r_mm*scan[j].r_mm*cos(scan[i].theta_rad)*sin(scan[j].theta_rad));
                    fourth+=(scan[i].r_mm*scan[j].r_mm*cos(scan[i].theta_rad+scan[j].theta_rad));
                }
            }

            line_params.center.theta_rad=0.5*atan2((second*2.0/M)-first,(fourth/M)-third);

            for(u16 i=start_idx;i<=end_idx;++i){
                sig+=scan[i].r_mm*cos(scan[i].theta_rad-line_params.center.theta_rad);
            }

            line_params.center.r_mm=sig/M;
            if(line_params.center.r_mm<0){
                line_params.center.theta_rad+=M_PI;
            }
            
            line_params.center.r_mm=fabs(line_params.center.r_mm)-LdsSensorCfg::sensorOffset_mm;
            line_params.center.theta_rad=fmod(line_params.center.theta_rad+(2*M_PI),2*M_PI);

            line_params.start_ang_rad=scan[start_idx].theta_rad;
            line_params.end_ang_rad=scan[end_idx].theta_rad;

            vector<f32> norm_dist(M,0);
            for(u16 i=start_idx;i<=end_idx;++i){
                norm_dist[i-start_idx]=scan[i].r_mm*cos(scan[i].theta_rad-line_params.center.theta_rad);
                norm_dist[i-start_idx]=fabs(fabs(norm_dist[i-start_idx])-line_params.center.r_mm);
            }

            for(u16 i=1;i<M-1;++i){
                if(norm_dist[i]>=norm_dist[i-1]&&norm_dist[i]>=norm_dist[i+1]&&norm_dist[i]>max_norm_dist){
                    max_norm_dist=norm_dist[i];
                    max_norm_dist_idx=i+start_idx;
                }
            }
        }
    }

    static void storeScan(const sensor_msgs::LaserScan& msg){
        Lock mux(scanMux);
        for(u16 idx=0;idx<msg.ranges.size();idx++){
            scan[idx]=static_cast<u16>(msg.ranges[idx]);
            intensity[idx]=static_cast<u16>(msg.intensities[idx]);
        }
    }
public:
    LfxNode():
        RosNodeBase("lfx"),
        ldsScanSub(n.subscribe("ldsScan",100,storeScan)),
        featPtPub(n.advertise<geometry_msgs::PoseArray>("featPoints", 1000)),
        featCornerPub(n.advertise<geometry_msgs::PoseArray>("featCorners", 1000)),
        featEdgePub(n.advertise<geometry_msgs::PoseArray>("featEdges", 1000)),
        featLnPub(n.advertise<geometry_msgs::PoseArray>("featLines", 1000)){}
private:
    Subscriber ldsScanSub;
    Publisher featPtPub;
    Publisher featLnPub;
    Publisher featCornerPub;
    Publisher featEdgePub;
    static vector<u16> scan; 
    static vector<u16> intensity; 
    static mutex scanMux;
};

vector<u16> LfxNode::scan(360,0);
vector<u16> LfxNode::intensity(360,0);
mutex LfxNode::scanMux;

#endif