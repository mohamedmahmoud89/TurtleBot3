#include "hmiGatewayNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"HmiGateway");

    HmiGatewayNode node;
    node.run();

    return 0;
}
