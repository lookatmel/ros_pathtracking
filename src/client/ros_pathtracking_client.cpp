#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "geometry_msgs/Pose.h"
#include "ros_pathtracking/pathtrackingAction.h"

#include <signal.h>

typedef actionlib::SimpleActionClient<ros_pathtracking::pathtrackingAction> Client;
Client *ac_;
ros_pathtracking::pathtrackingGoal goal;
ros_pathtracking::pathtrackingResultConstPtr result;




void activeCallback()
{
    ROS_INFO("Active!");
}

void doneCallback(const actionlib::SimpleClientGoalState &state, const ros_pathtracking::pathtrackingResultConstPtr &result)
{
    ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("Result: %d", result->result);
    ros::shutdown();
}


void feedbackCallback(const ros_pathtracking::pathtrackingFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback:%d", feedback->Step);
    // if(feedback->Step == 5 || feedback->Step == 6)
    // {
    //     goal.startmode = 2;
    //     ac_->sendGoal(goal, doneCallback, activeCallback, feedbackCallback);
    // }
}



void mySigIntHandler(int sig)
{
    ROS_INFO("close ros_autocharge!\r\n");
    // ac_->cancelAllGoals();
    goal.startmode = 1;
    ac_->sendGoal(goal, doneCallback, activeCallback, feedbackCallback);
    ros::shutdown();
    // exit(0);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"action_test_client",ros::init_options::NoSigintHandler); 
    ros::NodeHandle nh;
    Client ac("AutoCharge_Server", true);
    ac_ = &ac;
    ros::Duration t(10);
    ros::Rate r(1);
    signal(SIGINT, mySigIntHandler);

    // ac.waitForServer();
    while(!ac.isServerConnected() && ros::ok())
    {
        ROS_INFO("Waiting for action server to start.");
        r.sleep();
    }

    
    goal.startmode = 1;
    geometry_msgs::Pose pose;
    for(uint32_t i = 0; i < 100; i++)
    {
        pose.position.x = M_PI *4 / 100.0 * i;
        pose.position.y = 1 * sin(2 * pose.position.x);
        goal.path.poses.push_back(pose);
    }

    ac.sendGoal(goal, doneCallback, activeCallback, feedbackCallback);
    

    while(ros::ok())
    {
        
        r.sleep();
    }

    return 0;
}