#include "opencrComm.hpp"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"opencrCtrl");
	
	OpenCrCommNode node;
	node.run();

	return 0;
}
