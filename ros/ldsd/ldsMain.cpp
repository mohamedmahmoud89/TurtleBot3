#include "ldsNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"LdsNode");

    LdsNode node;
    node.run();

    return 0;
}