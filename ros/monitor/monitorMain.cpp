#include "monitorNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"MonNode");

    MonitorNode node;
    node.run();

    return 0;
}
