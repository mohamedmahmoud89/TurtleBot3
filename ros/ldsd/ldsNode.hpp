#ifndef LDS_NODE_HPP
#define LDS_NODE_HPP

#include "ldsDriver.hpp"
#include "nodeBase.hpp"

#define USB_PORT "/dev/ttyUSB0"
#define LDS_NODE_RATE 1

class LdsNode : public RosNodeBase{
    void update() override{
        ldsDriver.poll(ldsScan);
        ldsScan.header.stamp = ros::Time::now();

        ldsPub.publish(ldsScan);
    }

public:
    LdsNode():
        RosNodeBase(RosNodeBaseType::NON_SPINNING,LDS_NODE_RATE),
        io(),
        ldsScan(),
        ldsDriver(USB_PORT,baudRate,io),
        ldsPub(n.advertise<sensor_msgs::LaserScan>("ldsScan",1000)){
            ldsScan.header.frame_id = "laser";
            ldsScan.angle_increment = (2.0*M_PI/360.0);
            ldsScan.angle_min = 0.0;
            ldsScan.angle_max = 2.0*M_PI-ldsScan.angle_increment;
            ldsScan.range_min = LdsSensorCfg::minDepth_mm;
            ldsScan.range_max = LdsSensorCfg::maxDepth_mm;
            ldsScan.ranges.resize(360);
            ldsScan.intensities.resize(360);
        };
private:
    static const u32 baudRate{230400};
    boost::asio::io_service io;
    hls_lfcd_lds::LFCDLaser ldsDriver;
    Publisher ldsPub;
    sensor_msgs::LaserScan ldsScan;
};

#endif