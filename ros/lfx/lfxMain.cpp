#include "lfxNode.hpp"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"LfxNode");
	
	LfxNode node;
	node.run();

	return 0;
}
