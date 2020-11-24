#include "opencrComm.hpp"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"OpenCrCommNode");
	
	OpenCrCommNode node;
	node.run();

	return 0;
}
