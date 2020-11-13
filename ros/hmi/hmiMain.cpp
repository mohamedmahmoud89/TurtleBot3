#include "ctrlCmdNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"ctrlCmd");

    CtrlCmdNode node;
    node.run();

    return 0;
}
