#include "mclNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"MclNode");

    MclNode node;
    node.run();

    return 0;
}
