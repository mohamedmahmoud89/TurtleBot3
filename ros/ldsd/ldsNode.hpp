#ifndef LDS_NODE_HPP
#define LDS_NODE_HPP

#include "ldsDriver.hpp"
#include "nodeBase.hpp"
#include "common.hpp"

#define USB_PORT "/dev/ttyUSB0"

class LdsNode : public RosNodeBase{
    void update() override{
        sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);

        scan->header.frame_id = "laser";
        scan->angle_increment = (2.0*M_PI/360.0);
        scan->angle_min = 0.0;
        scan->angle_max = 2.0*M_PI-scan->angle_increment;
        scan->range_min = 0.12;
        scan->range_max = 3.5;
        scan->ranges.resize(360);
        scan->intensities.resize(360);

        ldsDriver.poll(scan);
        scan->header.stamp = ros::Time::now();

        ldsPub.publish(scan);
    }

public:
    LdsNode():
        RosNodeBase(SystemCfg::rate_hz),
        io(),
        ldsDriver(USB_PORT,baudRate,io),
        ldsPub(n.advertise<sensor_msgs::LaserScan>("LdsMeas",1000)){};
private:
    static const u32 baudRate{230400};
    boost::asio::io_service io;
    hls_lfcd_lds::LFCDLaser ldsDriver;
    Publisher ldsPub;
};

#endif