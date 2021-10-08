#include "mapNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"MapNode");

    MapNode node;
    node.run();

    return 0;
}
