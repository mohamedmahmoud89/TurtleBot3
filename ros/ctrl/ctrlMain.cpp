#include "ctrlNode.hpp"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"CtrlNode");
	
	CtrlNode node;
    node.run();

	return 0;
}
