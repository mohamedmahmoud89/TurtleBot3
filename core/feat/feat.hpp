#ifndef FEAT_ASSOCIATION_HPP
#define FEAT_ASSOCIATION_HPP

#include "common.hpp"
#include <unordered_map>

using namespace std; 

struct FeatAssociationCfg{
    static constexpr f32 max_ref_dist_mm{5000};
};

inline Point2D calcFeatGlobalPos(const Point2D& feat,const RobotPos& coord){
    return Point2D(
        coord.x_mm+feat.x_mm*cos(coord.theta_rad)-feat.y_mm*sin(coord.theta_rad),
        coord.y_mm+feat.x_mm*sin(coord.theta_rad)+feat.y_mm*cos(coord.theta_rad));
}

// nearest neighbor association
// returns a dict[measurement idx,landmark idx]
inline unordered_map<u16,u16> featAssociate(
        const vector<Point2D>& features,
        const vector<Point2D>& landmarks){
    unordered_map<u16,u16>ret;
    for(u16 i=0;i<features.size();++i){
        f64 min_squared(pow(FeatAssociationCfg::max_ref_dist_mm,2));
        for(u16 j=0;j<landmarks.size();++j){
            f64 dist_squared(
                pow(features[i].x_mm-landmarks[j].x_mm,2)+
                pow(features[i].y_mm-landmarks[j].y_mm,2)); 
            if(dist_squared<min_squared){
                min_squared=dist_squared;
                ret[i]=j;
            }
        }
    }
    return ret;
}

#endif