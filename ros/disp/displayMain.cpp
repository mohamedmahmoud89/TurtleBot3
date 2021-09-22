#include "displayNode.hpp"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"LfxNode");
	
	DisplayNode node;
	node.init();
    node.run();

	return 0;
}
