#include <ros/ros.h>
#include "ros_pathtracking/ros_pathtracking"


#include <signal.h>
#include <cmath>

#include "yaml-cpp/yaml.h"


using namespace std;



void mySigIntHandler(int sig)
{
    ROS_INFO("close ros_pathtracking!\r\n");
    ros::shutdown();
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"ros_pathtracking",ros::init_options::NoSigintHandler); 
    ros::NodeHandle nh;
    signal(SIGINT, mySigIntHandler);
    ROSPathTracking path;
    ros::spin();
    exit(0) ;
}
