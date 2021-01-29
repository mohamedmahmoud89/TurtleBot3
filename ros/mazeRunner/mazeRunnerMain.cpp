#include "mazeRunnerNode.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"MazeRunnerNode");

    MazeRunnerNode node;
    node.run();

    return 0;
}
