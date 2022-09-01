#include <signal.h>
#include <fstream>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <sys/stat.h>

#include "nlohmann/json.hpp"

#include "ros/ros.h" 
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

using json = nlohmann::json;
using namespace std;

void mySigIntHandler(int sig)
{
    ROS_INFO("close ros_pathrecord!\r\n");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ros_pathrecord",ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

    if(argc < 2)
    {
        cout << "Please enter file path to save!" << endl;
        return 0;
    } 
    for(int i = 0; i< argc; i++)
    {
        std::cout << argv[i] << "\r\n" << std::endl; 
    }
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    tf::TransformListener listener;
    tf::StampedTransform transform;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseArray posearray;
    ros::Publisher pose_pub = nh_private.advertise<geometry_msgs::PoseStamped>("pose", 10);
    ros::Publisher posearray_pub = nh_private.advertise<geometry_msgs::PoseArray>("posearray", 10);

    string rpath(argv[1]);
    if(*(rpath.cend() - 1) == '/')
    {
        cout << "not a filepath!" << endl;
        return 0;
    }
    auto pos = rpath.find_last_of('/');
    string pathpath(rpath.cbegin(), rpath.cbegin() + pos);
    cout << pathpath << endl;
    std::ofstream oj(rpath);
    if(!oj.good())
    {
        cout << "dir:\"" << rpath << "\" dosen't exit!" << endl;
        return 0;
    }

    json jdata;
    json array = json::array();
    json sub_array = json::array();

    ros::Time tf_now;
    bool tf_rev = 0;
    bool first_point = 1;
    double x = 0, y = 0, z = 0, yaw = 0;
    double x_last = 0, y_last = 0, z_last = 0, yaw_last = 0;
    uint32_t point_cnt = 0;
    // for(int i = 0; i < 10; ++i)
    // {
    //     sub_array = {i, 2*i, 3*i};
    //     array.push_back(sub_array);
    // }

    ros::Rate r(50);
    while(ros::ok())
    {
        try
        {
            tf_now = ros::Time::now();
            tf_rev = listener.waitForTransform("map", "base_footprint", tf_now, ros::Duration(0.2));
            listener.lookupTransform("map", "base_footprint", tf_now, transform);
            if(tf_rev)
            {
                x = transform.getOrigin().x();
                y = transform.getOrigin().y();
                z = transform.getOrigin().z();
                yaw = tf::getYaw(transform.getRotation());

                if(first_point)
                {
                    first_point = 0;
                    posearray.header.frame_id = "map";
                    posearray.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = z;
                    pose.pose.orientation.x = transform.getRotation().getX();
                    pose.pose.orientation.y = transform.getRotation().getY();
                    pose.pose.orientation.z = transform.getRotation().getZ();
                    pose.pose.orientation.w = transform.getRotation().getW();
                    point_cnt ++;
                    // pose_pub.publish(pose);
                    posearray.poses.push_back(pose.pose);
                    posearray_pub.publish(posearray);
                    sub_array = {x, y, z, \
                                transform.getRotation().getX(), transform.getRotation().getY(), \
                                transform.getRotation().getZ(), transform.getRotation().getW()};
                    array.push_back(sub_array);
                    x_last = x;
                    y_last = y;
                    z_last = z;
                    yaw_last = yaw;
                }
                else if(sqrt(pow(x_last - x, 2) + pow(y_last - y, 2)) >= 0.1)
                {
                    posearray.header.frame_id = "map";
                    posearray.header.stamp = ros::Time::now();
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = z;
                    pose.pose.orientation.x = transform.getRotation().getX();
                    pose.pose.orientation.y = transform.getRotation().getY();
                    pose.pose.orientation.z = transform.getRotation().getZ();
                    pose.pose.orientation.w = transform.getRotation().getW();
                    point_cnt ++;
                    // pose_pub.publish(pose);
                    posearray.poses.push_back(pose.pose);
                    posearray_pub.publish(posearray);
                    sub_array = {x, y, z, \
                                transform.getRotation().getX(), transform.getRotation().getY(), \
                                transform.getRotation().getZ(), transform.getRotation().getW()};
                    array.push_back(sub_array);
                    x_last = x;
                    y_last = y;
                    z_last = z;
                    yaw_last = yaw;
                }
            }
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        r.sleep();
    }

    if(sqrt(pow(x_last - x, 2) + pow(y_last - y, 2)) < 0.1)
    {
        (array.end() - 1)->at(0) = transform.getOrigin().x();
        (array.end() - 1)->at(1) = transform.getOrigin().y();
        (array.end() - 1)->at(2) = transform.getOrigin().z();

        (array.end() - 1)->at(3) = transform.getRotation().getX();
        (array.end() - 1)->at(4) = transform.getRotation().getY();
        (array.end() - 1)->at(5) = transform.getRotation().getW();
        (array.end() - 1)->at(6) = transform.getRotation().getZ();
    }
    else
    {
        sub_array = {transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), \
                    transform.getRotation().getX(), transform.getRotation().getY(), \
                    transform.getRotation().getZ(), transform.getRotation().getW()};
        array.push_back(sub_array);
        point_cnt ++;
    }

    jdata["path"] = array;
    std::string jsonstr = jdata.dump();
    oj << jdata << std::endl;
    oj.close();
    cout << "record " << point_cnt << " point!\r\n" << "save on " << rpath << endl;
    // cout << jsonstr << endl;
    exit(0);
}
