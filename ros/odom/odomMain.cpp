#include "odomNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"OdomNode");

    OdomNode node;
    node.run();

    return 0;
}
