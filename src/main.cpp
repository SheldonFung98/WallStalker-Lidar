#include "common.h"
#include "wallDetect.h"


int main (int argc, char** argv){
	ros::init(argc, argv, "laser_stickwall");
	WallDetect *wallDetect = new WallDetect();
	ros::spin();

}