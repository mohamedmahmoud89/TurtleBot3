#include "ppNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"PpNode");

    PpNode node;
    node.run();

    return 0;
}
