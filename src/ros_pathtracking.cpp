#include "ros_pathtracking/ros_pathtracking"
#include "yaml-cpp/yaml.h"

#include <cmath>

using namespace std;

ROSPathTracking::ROSPathTracking()
    :as_(NULL),ctrl_frequency_(50),max_path_err_(0.1)
{
    ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("laser_topic", laser_topic_, "/scan");
	nh_private.param<std::string>("cmd_vel", cmd_vel_topic_, "/cmd_vel"); 
	nh_private.param<std::string>("pannel_id", pannel_frame_id_, "/pannel"); 
	nh_private.param<std::string>("base_id", base_frame_id_, "/base_link"); 
    nh_private.param<std::string>("laser_id", laser_frame_id_, "/laser"); 
    nh_private.param<std::string>("map_id", map_frame_id_, "/map"); 

    nh_private.param<int>("control_frequency", ctrl_frequency_, 50); 
    nh_private.param<double>("turn_index", turn_index_, 1.0); 
    nh_private.param<double>("path_err", max_path_err_, 0.1); 
    nh_private.param<double>("max_lspeed", max_lspeed_, 0.7); 
    nh_private.param<double>("min_lspeed", min_lspeed_, 0.01); 
    nh_private.param<double>("max_aspeed", max_aspeed_, 0.6); 
    nh_private.param<double>("lacc", lacc_, 0.2); 
    nh_private.param<double>("ldec", ldec_, 0.2); 
    nh_private.param<double>("max_lacc", max_lacc_, 0.2); 
    nh_private.param<double>("max_ldec", max_ldec_, 0.2); 

    nh_private.param<float>("visual_angle", visual_angle_, VISUAL_ANGLE); 
    nh_private.param<float>("round_distance", round_distance_, ROUND_DISTANCE); 
    nh_private.param<float>("round_min_distance", round_min_distance_, ROUND_MIN); 
    nh_private.param<float>("round_angle_max", round_angle_max_, ROUND_ANGLE_MAX); 
    nh_private.param<float>("pannel_length", pannel_length_, PANNEL_LENGTH); 
    nh_private.param<float>("obstacle_distance", obstacle_distance_, OBSTACLE_DISTANCE); 
    nh_private.param<float>("obstacle_angle_min", obstacle_angle_min_, OBSTACLE_ANGLE_MIN); 
    nh_private.param<float>("obstacle_angle_max", obstacle_angle_max_, OBSTACLE_ANGLE_MAX); 
    nh_private.param<float>("obstacle_deadzone", obstacle_deadzone_, OBSTACLE_DRADZONE); 
    nh_private.param<float>("pretouch_distance", pretouch_distance_, PRETOUCH_DISTANCE); 
    nh_private.param<float>("moveback_distance", moveback_distance_, MOVEBACK_DISTANCE);
    nh_private.param<std::string>("movebase_polygon", movebase_polygon_str_, "[[0.29,-0.22],[0.29,0.22],[-0.29,0.22],[-0.29,-0.22]]"); 
    nh_private.param<std::string>("movebase_polygon2", movebase_polygon2_str_, "[[0.49,-0.22],[0.49,0.22],[-0.49,0.22],[-0.49,-0.22]]"); 

    cout << "polygon init:" << movebase_polygon_str_ << endl;
    cout << "polygon2 init:" << movebase_polygon2_str_ << endl;

    YAML::Node node1 = YAML::Load(movebase_polygon_str_);
    YAML::Node node2 = YAML::Load(movebase_polygon2_str_);
    for (YAML::const_iterator i = node1.begin(); i != node1.end(); ++i)
    {
        float tmp[2];
        if(i->size() != 2)
        {
            ROS_ERROR("polygon1 param error!");
            ros::shutdown();
            return;
        }
        tmp[0] = i->as<vector<float>>().at(0);
        tmp[1] = i->as<vector<float>>().at(1);
        movebase_polygon1_.push_back({tmp[0], tmp[1]});
    }

    for (YAML::const_iterator i = node2.begin(); i != node2.end(); ++i)
    {
        float tmp[2];
        if(i->size() != 2)
        {
            ROS_ERROR("polygon2 param error!");
            ros::shutdown();
            return;
        }
        tmp[0] = i->as<vector<float>>().at(0);
        tmp[1] = i->as<vector<float>>().at(1);
        movebase_polygon2_.push_back({tmp[0], tmp[1]});
    }

    cout << "polygon1:" ;
    for(auto i = movebase_polygon1_.begin(); i != movebase_polygon1_.cend(); ++i)
    {
        cout << "[" << i->at(0) << "," << i->at(1) << "],";
    }
    cout << "\b \b\r\n" << endl;

    cout << "polygon2:" ;
    for(auto i = movebase_polygon2_.begin(); i != movebase_polygon2_.cend(); ++i)
    {
        cout << "[" << i->at(0) << "," << i->at(1) << "],";
    }
    cout << "\b \b\r\n" << endl;

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    pose_pub_ = nh_private.advertise<geometry_msgs::PoseArray>("posearray", 10);

    as_ = new PathTracking_Server(nh_, "PathTracking_Server", boost::bind(&ROSPathTracking::executeCB, this, _1), false);
    as_->start();
    ROS_INFO("PathTracking Action Created!");
}

ROSPathTracking::~ROSPathTracking()
{
    laser_scan_sub_.shutdown();
    cmd_pub_.shutdown();
    if(as_ != NULL)
    {
        delete as_;
    }
}

void ROSPathTracking::start()
{
    search_time_.fromSec(0);
    last_scan_time_.fromSec(0);
    last_time_.fromSec(0);

    map_tf_overtime_.fromSec(0);
    obstacle_overtime_.fromSec(0);
    scan_overtime_.fromSec(0);
    motion_waittime_.fromSec(0);
    stable_waittime_.fromSec(0);
    scan_overtime_.fromSec(0);

    x_ = 0; y_ = 0; yaw_ = 0;
    x_temp_ = 0; y_temp_ = 0; yaw_temp_ = 0;
    state_ = 0; step_ = STEP_INIT; tf_rev_ = 0; have_obstacle_ = 0; wait_stable_ = 0, moveback_ = 0;
    scan_ready_ = 0;

    point_now_ = 0;
    point_target_ = 0;
    point_end_ = 0;
    point_cal_ = 0;

    planning_distance_ = 0;
    dec_distance_ = 0;

    planning_time_ = 1.0;

    dec_distance_ = pow(max_lspeed_, 2) / (2.0 * ldec_);
    predict_distance_ = dec_distance_ + max_lspeed_ * planning_time_;

    laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &ROSPathTracking::LaserScanCallback, this);
    last_time_ = ros::Time::now();
}

void ROSPathTracking::stop()
{
    publishZeroSpeed();
    step_ = STEP_CANCEL;
}



void ROSPathTracking::cancel()
{
    publishZeroSpeed();
    step_ = STEP_CANCEL;
}

void ROSPathTracking::shutdown()
{
    laser_scan_sub_.shutdown();
}

unsigned int ROSPathTracking::getStep()
{
    return step_;
}

void ROSPathTracking::executeCB(const ros_pathtracking::pathtrackingGoalConstPtr &goal)
{
    newgoal_ = goal;
    ROS_INFO("Got a Goal:%d", newgoal_->startmode);
    start();
    setLSpeedAccParam(0.5,0.7);
    // setASpeedAccParam(3.14/8, 3.14/1);
    setASpeedAccParam(3.14/4, 3.14/1);

    ros::Rate r(ctrl_frequency_);
    while(1)
    {
        if(!ros::ok() || !as_->isActive())
        {
            publishZeroSpeed();
            break;
        }

        if(as_->isPreemptRequested())
        {
            if(as_->isNewGoalAvailable())
            {
                newgoal_ = as_->acceptNewGoal();
                if(newgoal_->startmode == 2)
                {
                    stop();
                    ROS_INFO("Action Stop!");
                }
                else
                {
                    cancel();
                    ROS_INFO("Action Canncel!");
                }
                
            }
            else
            {
                stop();
                ROS_INFO("Action Stop!");
            }
        }

        PathMotionHandler();
        feedback_.Step = getStep();
        as_->publishFeedback(feedback_);

        switch(getStep())
        {
            case STEP_SUCCESS :
                as_->setSucceeded(result_);
                shutdown();
                ROS_INFO("Action is Succeed!");
                break;
            case STEP_CANCEL :
                as_->setPreempted(result_);
                shutdown();
                break;
            case STEP_ERROR :
                as_->setAborted(result_);
                shutdown();
                ROS_INFO("Action is Failed!");
                break;
            default:break;
        }


        r.sleep();
    }
    shutdown();
}

void ROSPathTracking::PathMotionHandler()
{
    double tangle = 0, dangle = 0;
    double forward_x = 0, forward_y = 0;
    double forward_distance = 0, forward_v1x = 0, forward_v1y = 0, forward_v2x = 0, forward_v2y = 0, forward_vx = 0, forward_vy = 0;
    static double start_x, start_y, start_yaw;
    float aspeed = 0;
    geometry_msgs::Pose pp;
    tf::Quaternion quat;
    geometry_msgs::PoseArray posearray_tmp;
    try
    {
        tf_now_ = ros::Time::now();
        tf_rev_ = listener_.waitForTransform(map_frame_id_, base_frame_id_, tf_now_, ros::Duration(0.2));
        listener_.lookupTransform(map_frame_id_, base_frame_id_, tf_now_, transform_);
        if(tf_rev_)
        {
            x_temp_ = transform_.getOrigin().x();
            y_temp_ = transform_.getOrigin().y();
            yaw_temp_ = tf::getYaw(transform_.getRotation());
            if((fabs(x_temp_ - transform_.getOrigin().x()) > 0.1 || fabs(y_temp_ - transform_.getOrigin().y()) > 0.1) \
                && !(x_ == 0 && y_ == 0 && yaw_ == 0))
            {
                // tf_last = tf_now;
                tf_rev_ = 0;
            }
            else
            {
                tf_last_ = tf_now_;
                x_ = x_temp_;
                y_ = y_temp_;
                yaw_ = yaw_temp_;
            }
            // ROS_INFO("X:%0.3f Y:%0.3f Angle:%0.3f", x, y, yaw * 180 / 3.14159);
        }
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    
    period_ = ros::Time::now() - last_time_;
    // ROS_INFO("Period:%0.5f", period.toSec());

    switch(step_)
    {
        double stime;
        case STEP_INIT:
            if(!scan_ready_)
            {
                scan_overtime_ += period_;
                if(scan_overtime_.toSec() > 20)
                {
                    StepChange(STEP_ERROR);
                    ROS_INFO("Fail to get Laserscan!");
                }
            }
            else if(have_obstacle_)
            {
                obstacle_overtime_ += period_;
                if(obstacle_overtime_.toSec() > 20)
                {
                    StepChange(STEP_ERROR);
                    ROS_INFO("tracking fail because of obstacle!");
                }
            }
            else if(tf_rev_ != 1)
            {
                map_tf_overtime_ += period_;
                if(map_tf_overtime_.toSec() > 2)
                {
                    StepChange(STEP_ERROR);
                    search_time_ = ros::Time::now();
                    ROS_INFO("tf error!");
                }
            }
            else if(tf_rev_ == 1)
            {
                StepChange(STEP_PARAM_INIT);
                ROS_INFO("tracking start!");
            }
            setLSpeed(0);
            setASpeed(0);
            break;

        case STEP_PARAM_INIT:
            point_data_.clear();
            planning_distance_ = 0;
            dec_distance_ = 0;
            point_end_ = newgoal_->path.poses.size();
            start_x = x_;
            start_y = y_;
            start_yaw = yaw_;

            //************************publish path***************
            quat.setRPY(0, 0, atan2(newgoal_->path.poses.at(0).position.y - y_, newgoal_->path.poses.at(0).position.x - x_));
            pp.orientation.x = quat.getX();
            pp.orientation.y = quat.getY();
            pp.orientation.z = quat.getZ();
            pp.orientation.w = quat.getW();
            pp.position.x = x_;
            pp.position.y = y_;
            pp.position.z = 0.1;
            posearray_tmp.header.frame_id = "/map";
            posearray_tmp.poses.push_back(pp);
            posearray_tmp.poses.insert(posearray_tmp.poses.end(), newgoal_->path.poses.cbegin(), newgoal_->path.poses.cend());
            pose_pub_.publish(posearray_tmp);
            //*************************publish path end*************

            //*******Calculate distance and angle of now position to point 0
            point_data_.push_back((PointParam){sqrt(pow(newgoal_->path.poses.at(0).position.x - x_, 2) \
                                                    + pow(newgoal_->path.poses.at(0).position.y - y_, 2)), \
                                                0, 0, 0, \
                                                calPathAngle((Point){start_x, start_y},\
                                                                    (Point){newgoal_->path.poses.at(0).position.x, newgoal_->path.poses.at(0).position.y}), \
                                                0, 0, 0});
            //***********Calculate distance and angle of single path
            for(uint32_t i = 1; i < point_end_; i++)
            {
                point_data_.push_back((PointParam){sqrt(pow(newgoal_->path.poses.at(i).position.x - newgoal_->path.poses.at(i - 1).position.x, 2) \
                                                        + pow(newgoal_->path.poses.at(i).position.y - newgoal_->path.poses.at(i - 1).position.y, 2)), \
                                                    0, 0, 0,\
                                                    calPathAngle((Point){newgoal_->path.poses.at(i - 1).position.x, newgoal_->path.poses.at(i - 1).position.y},\
                                                                    (Point){newgoal_->path.poses.at(i).position.x, newgoal_->path.poses.at(i).position.y}), \
                                                    0, 0, 0});
            }
            cout << newgoal_->path.poses.size() << endl;
            cout << point_data_.size() << endl;
            //***********Calculate difference angle and point speed
            for(uint32_t i = 0; i < point_end_ - 1; ++i)
            {
                double path_err_sin = 0;
                double diff_tmp = 0, diff_limit = 0;
                if(i == 0)
                {
                    // point_data_.at(0).diff_angle = (point_data_.at(0).angle - start_yaw);
                    diff_tmp = point_data_.at(0).angle - start_yaw;
                }
                else
                {
                    // point_data_.at(i).diff_angle = (point_data_.at(i).angle - point_data_.at(i - 1).angle);
                    diff_tmp = point_data_.at(i).angle - point_data_.at(i - 1).angle;
                }
                if(diff_tmp > M_PI) diff_tmp -= 2 * M_PI;
                else if(diff_tmp < -M_PI) diff_tmp += 2 * M_PI;
                point_data_.at(i).diff_angle = diff_tmp;
                diff_limit = fabs(diff_tmp) > M_PI / 2 ? M_PI / 2 : fabs(diff_tmp);

                path_err_sin = 0.5 * sin(diff_limit) * tan(M_PI/2 - max_aspeed_ * 1 / ctrl_frequency_ / 2) - \
                                pow(sin(diff_limit / 2), 2);
                point_data_.at(i).path_err_sin = path_err_sin;
                point_data_.at(i).limit_of_turn = min(max_lspeed_, (path_err_sin < 0.01 ? max_lspeed_ : (max_path_err_ / (1.0 / ctrl_frequency_) / path_err_sin)));

                point_data_.at(i).point_speed = min(point_data_.at(i).limit_of_turn, point_data_.at(i).length * max_aspeed_ / max(0.01, fabs(diff_tmp)));
            }
            //***********Calculate raw path speed through dec (from end to start)
            for(int32_t i = point_end_ - 2; i >= 0; --i)
            {
                point_data_.at(i).raw_path_speed = min((double)point_data_.at(i).point_speed,\
                                                        sqrt(pow(point_data_.at(i+1).raw_path_speed, 2) + 2 * ldec_ * point_data_.at(i + 1).length));
            }
            //***********Calculate  path speed through acc (from start to end)
            for(uint32_t i = 0; i < point_end_ - 1; ++i)
            {
                if(i == 0)
                {
                    point_data_.at(i).path_speed = min((double)point_data_.at(i).raw_path_speed,\
                                                    (sqrt(2 * lacc_ * point_data_.at(i).length)));
                }
                else
                {
                    point_data_.at(i).path_speed = min((double)point_data_.at(i).raw_path_speed,\
                                                    (sqrt(pow(point_data_.at(i - 1).raw_path_speed, 2) + 2 * lacc_ * point_data_.at(i).length)));
                }
                
            }
            point_now_ = 0;
            for(auto i = point_data_.cbegin(); i != point_data_.cend(); i++)
            {
                printf("%0.4f  %0.4f  %0.4f %0.4f %0.4f %0.4f\r\n", i->angle, i->diff_angle, i->length, i->path_err_sin,i->limit_of_turn, i->path_speed);
            }
            printf("end point: x:%0.4f y:%0.4f\r\n", (newgoal_->path.poses.cend() - 1)->position.x, (newgoal_->path.poses.cend() - 1)->position.y);
            printf("start point speed:%0.4f path raw speed:%0.4f path speed:%0.4f\r\n", (point_data_.cbegin())->point_speed, (point_data_.cbegin())->raw_path_speed, (point_data_.cbegin())->path_speed);
            printf("end point speed:%0.4f path raw speed:%0.4f path speed:%0.4f\r\n", (point_data_.cend()-1)->point_speed, (point_data_.cend()-1)->raw_path_speed, (point_data_.cend()-1)->path_speed);
            StepChange(STEP_TRACKING);

            if(point_data_.at(point_now_).path_speed < min_lspeed_)
            {
                motion_state_ = 0;
                motion_lspeed_ = point_data_.at(point_now_ + 1).path_speed;
            }
            else
            {
                motion_state_ = 2;
                motion_lspeed_ = point_data_.at(point_now_).path_speed;
            }
            break;
        case STEP_TRACKING:

            //************************计算当前点是否过当前点,并更新当前点
            for(; point_now_ < point_end_ - 1;)
            {
                if(point_now_ == 0)
                {
                    if(!PointIsAheadOfLine((Line){{start_x, start_y},\
                                            {newgoal_->path.poses.at(point_now_).position.x, newgoal_->path.poses.at(point_now_).position.y}}, \
                                            (Point){x_, y_}))
                    {
                        break;
                    }
                }
                else if(!PointIsAheadOfLine((Line){{newgoal_->path.poses.at(point_now_ - 1).position.x, newgoal_->path.poses.at(point_now_ - 1).position.y},\
                                            {newgoal_->path.poses.at(point_now_).position.x, newgoal_->path.poses.at(point_now_).position.y}}, \
                                    (Point){x_, y_}))
                {
                    break;
                }

                point_now_++;
                if(point_now_ == point_end_ - 1)
                {
                    motion_state_ = 2;
                    motion_lspeed_ = min_lspeed_;
                }
                else if(point_now_ == 1)
                {
                    motion_state_ = 0;
                    motion_lspeed_ = point_data_.at(point_now_ + 1).path_speed;
                }
                else
                {
                    if(point_data_.at(point_now_).path_speed < min_lspeed_)
                    {
                        motion_state_ = 0;
                        motion_lspeed_ = point_data_.at(point_now_ + 1).path_speed;
                    }
                    else if(point_data_.at(point_now_ + 1).path_speed < point_data_.at(point_now_).path_speed && point_data_.at(point_now_ + 1).path_speed > min_lspeed_)
                    {
                        motion_state_ = 2;
                        motion_lspeed_ = point_data_.at(point_now_ + 1).path_speed;
                    }
                    else
                    {
                        motion_state_ = 2;
                        motion_lspeed_ = point_data_.at(point_now_).path_speed;
                    }
                }
                printf("point update!\r\n");
            }

            //*************************速度规划
            if(point_now_ != 0)
            {
                forward_x = newgoal_->path.poses.at(point_now_).position.x + \
                            (newgoal_->path.poses.at(point_now_).position.x - newgoal_->path.poses.at(point_now_ - 1).position.x) * \
                            0.2 / point_data_.at(point_now_).length;
                forward_y = newgoal_->path.poses.at(point_now_).position.y + \
                            (newgoal_->path.poses.at(point_now_).position.y - newgoal_->path.poses.at(point_now_ - 1).position.y) * \
                            0.2 / point_data_.at(point_now_).length;

                // forward_distance = sqrt(pow(newgoal_->path.poses.at(point_now_).position.x - x_, 2) + \
                //                     pow(newgoal_->path.poses.at(point_now_).position.y - y_, 2));
                // forward_v1x = (newgoal_->path.poses.at(point_now_).position.x - x_) / forward_distance;
                // forward_v1y = (newgoal_->path.poses.at(point_now_).position.y - y_) / forward_distance;

                // forward_v2x = (newgoal_->path.poses.at(point_now_).position.x - newgoal_->path.poses.at(point_now_ - 1).position.x) / \
                //                 point_data_.at(point_now_).length;
                // forward_v2y = (newgoal_->path.poses.at(point_now_).position.y - newgoal_->path.poses.at(point_now_ - 1).position.y) / \
                //                 point_data_.at(point_now_).length;
                // forward_x = newgoal_->path.poses.at(point_now_).position.x + \
                //             (forward_v1x + forward_v2x) * 2;
                // forward_y = newgoal_->path.poses.at(point_now_).position.y + \
                //             (forward_v1y + forward_v2y) * 2;
            }
            else
            {
                forward_x = newgoal_->path.poses.at(point_now_).position.x + \
                            (newgoal_->path.poses.at(point_now_).position.x - start_x) * 0.5 / point_data_.at(point_now_).length;
                forward_y = newgoal_->path.poses.at(point_now_).position.y + \
                            (newgoal_->path.poses.at(point_now_).position.y - start_y) * 0.5 / point_data_.at(point_now_).length;
            }

            if(have_obstacle_)
            {
                setWaitStable(1);
                setLSpeed(0);
                setASpeed(0);
                break;
            }
            if(!isStable())
            {
                setLSpeed(0);
                setASpeed(0);
                break;
            }
            
            tangle = atan2(forward_y - y_, forward_x - x_);

            if(tangle - yaw_ > M_PI) dangle = tangle - yaw_ - 2 * M_PI;
            else if(tangle - yaw_ < -M_PI) dangle = tangle - yaw_ + 2 * M_PI;
            else dangle = tangle - yaw_;

            aspeed = dangle * 1;
            if(aspeed > max_aspeed_) aspeed = max_aspeed_;
            else if(aspeed < -max_aspeed_) aspeed = -max_aspeed_;
            setASpeedWithAcc(aspeed);

            switch(motion_state_)
            {
                case 0 :
                    setLSpeedWithAcc(0);
                    if(fabs(dangle) < 0.1) motion_state_ = 1;
                    break;
                case 1:
                case 2:
                    if(have_obstacle2_)
                    {
                        setLSpeedWithAcc(min(0.2, motion_lspeed_ * 0.5));
                    }
                    else
                    {
                        setLSpeedWithAcc(motion_lspeed_);
                    }
                    break;
            }

            // if(point_now_ == point_end_ - 1)
            // {
            //     setLSpeedWithAcc(0.001);
            // }
            // else
            // {
            //     setLSpeedWithAcc(point_data_.at(point_now_).path_speed);
            // }

            if(point_now_ == point_end_ - 1 &&  PointIsAheadOfLine((Line){{newgoal_->path.poses.at(point_now_ - 1).position.x, newgoal_->path.poses.at(point_now_ - 1).position.y},\
                                            {newgoal_->path.poses.at(point_now_).position.x, newgoal_->path.poses.at(point_now_).position.y}}, \
                                    (Point){x_, y_}))
            {
                setAllSpeedZero();
                StepChange(STEP_WAITING);
            }
            printf("point:%d x_:%0.4f y_:%0.4f px:%0.4f py:%0.4f fx:%0.4f fy:%0.4f\r\n", \
                    point_now_, x_, y_, \
                    newgoal_->path.poses.at(point_now_).position.x, \
                    newgoal_->path.poses.at(point_now_).position.y,\
                    forward_x, forward_y);
            break;
        case STEP_WAITING:
            // printf("point:%d x_:%0.4f y_:%0.4f px:%0.4f py:%0.4f\r\n", \
            //         point_now_, x_, y_, \
            //         newgoal_->path.poses.at(point_now_).position.x, \
            //         newgoal_->path.poses.at(point_now_).position.y);

            printf("point:%d x_:%0.4f y_:%0.4f px:%0.4f py:%0.4f fx:%0.4f fy:%0.4f\r\n", \
                    point_now_, x_, y_, \
                    newgoal_->path.poses.at(point_now_).position.x, \
                    newgoal_->path.poses.at(point_now_).position.y,\
                    forward_x, forward_y);
            break;
        case STEP_ERROR:
            break;
        case STEP_CANCEL:
            break;
        case STEP_SUCCESS:
            break;
        default:break;
    }

    publishSpeed();

    last_time_ = ros::Time::now();
}

void ROSPathTracking::StepChange(StepEnum nextstep)
{
    motion_waittime_.fromSec(0);
    obstacle_overtime_.fromSec(0);
    map_tf_overtime_.fromSec(0);

    step_ = nextstep;
}

void ROSPathTracking::setWaitStable(unsigned int sec)
{
    last_stable_time_ = ros::Time::now();
    wait_stable_ = 1;
    stable_waittime_.fromSec(sec);
}

bool ROSPathTracking::isStable()
{
    if(ros::Time::now() > last_stable_time_ + stable_waittime_)
    {
        return 1;
    }
    return 0;
}

float ROSPathTracking::getAccSpeed(float facc , float fdec , float bacc , float bdec , float target , float now_set, float real_value, double period)
{
    if(period <= 0) return now_set;

    double new_set , err_temp;
    double facc_ = facc * period;
    double fdec_ = fdec * period;
    double bacc_ = bacc * period;
    double bdec_ = bdec * period;
    err_temp = target - now_set;
    
    if(target >= 0)
    {
        if(now_set >= 0)
        {
            if(err_temp > facc_)
            {
                if(now_set - real_value > 200)
                {
                    new_set = now_set;
                }
                else
                {
                    new_set = now_set + facc_;
                }
            }
            else if(err_temp < -fdec_)
            {
                new_set = now_set - fdec_;
            }
            else{new_set = target;}
        }
        else
        {
            if(err_temp >= bdec_)
            {
                if(-now_set <= bdec_)
                {
                    new_set = 0;
                }
                else
                {
                    new_set = now_set + bdec_;
                }
            }
            else
            {
                new_set = 0;
            }
        }
    }
    else
    {
        if(now_set <= 0)
        {
            if(err_temp < -bacc_)
            { 
                if(now_set - real_value < -200)
                {
                    new_set = now_set;
                }
                else
                {
                    new_set = now_set - bacc_;
                }
            }
            else if(err_temp > bdec_)
            {
                new_set = now_set + bdec_;
            }
            else{new_set = target;}
        }
        else
        {
            if(err_temp <= -fdec_)
            {
                if(now_set <= fdec_)
                {
                    new_set = 0;
                }
                else
                {
                    new_set = now_set - fdec_;
                }
            }
            else
            {
                new_set = 0;
            }
        }
    }
    
    
    return (float)new_set;
}




void ROSPathTracking::LaserScanCallback(const sensor_msgs::LaserScan &scan)
{
    // ROS_INFO("Scan Min Angle:%0.100f , Max Angle:%0.100f", scan_angle_min, scan_angle_max);
    static tf::StampedTransform laser_transform;
    static double laser_roll, laser_pitch, laser_yaw, laser_x, laser_y, laser_z;
    static vector<vector<float>>polygon, polygon2;
    static vector<float>obstacle_thr, obstacle_thr2;
    static unsigned int angle_division, scan_min_index, scan_max_index;
    static float scan_angle_min = 0, scan_angle_max = 0;
    static float laser_range_max = 20.0;
    if((scan_angle_min == 0 && scan_angle_max == 0) || (scan_angle_min != scan.angle_min || scan_angle_max != scan.angle_max) || scan_ready_ != 1)
    {
        try
        {
            auto laser_tf_now = ros::Time::now();
            laser_listener_.waitForTransform(base_frame_id_, laser_frame_id_, laser_tf_now, ros::Duration(0.2));
            laser_listener_.lookupTransform(base_frame_id_, laser_frame_id_, laser_tf_now, laser_transform);
            laser_x = laser_transform.getOrigin().getX();
            laser_y = laser_transform.getOrigin().getY();
            laser_z = laser_transform.getOrigin().getZ();
            tf::Matrix3x3(laser_transform.getRotation()).getRPY(laser_roll, laser_pitch, laser_yaw);
            ROS_INFO("Got Laser tf x:%0.2f y:%0.2f z:%0.2f roll:%0.2f pitch:%0.2f yaw:%0.2f \r\n", laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw);
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // scan_angle_min = std::max(scan.angle_min, obstacle_angle_min_);
        // scan_angle_max = std::min(scan.angle_max, obstacle_angle_max_);
        // angle_division = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        // scan_min_index = scan_angle_min > scan.angle_min ? (scan_angle_min - scan.angle_min) / scan.angle_increment + 1 : 0;
        // scan_max_index = scan_angle_max < scan.angle_max ? angle_division - (scan.angle_max - scan_angle_max) / scan.angle_increment + 1 : angle_division;
        // ROS_INFO("Scan Min Angle:%0.4f , Max Angle:%0.4f", scan_angle_min * 180 / M_PI, scan_angle_max * 180 /M_PI);
        // ROS_INFO("Scan Division:%d Scan Min Index:%d , Max Index:%d", angle_division, scan_min_index, scan_max_index);
        // ROS_INFO("Scan size:%ld Inc:%0.9f", scan.ranges.size(), scan.angle_increment);

        scan_angle_min = scan.angle_min;
        scan_angle_max = scan.angle_max;
        angle_division = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        ROS_INFO("Scan Min Angle:%0.4f , Max Angle:%0.4f , Division:%d , Scan size:%ld , Inc:%0.5f ", \
                    scan_angle_min * 180 / M_PI, scan_angle_max * 180 /M_PI, angle_division, scan.ranges.size(), scan.angle_increment);
        
        polygon.clear();
        polygon2.clear();
        obstacle_thr.clear();
        obstacle_thr2.clear();
        if(isEqual(fmod(fabs(laser_roll), 2 * M_PI), 0) || isEqual(fmod(fabs(laser_roll), 2 * M_PI), 2 * M_PI))
        {
            ROS_INFO("laser is upright!");
            for(auto i = movebase_polygon1_.cbegin(); i != movebase_polygon1_.cend(); ++i)
            {
                polygon.push_back({(float)(i->at(0) - laser_x), (float)(i->at(1) - laser_y)});
            }
            for(auto i = movebase_polygon2_.cbegin(); i != movebase_polygon2_.cend(); ++i)
            {
                polygon2.push_back({(float)(i->at(0) - laser_x), (float)(i->at(1) - laser_y)});
            }
        }
        else
        {
            ROS_INFO("laser is reversed!");
            for(auto i = movebase_polygon1_.cbegin(); i != movebase_polygon1_.cend(); ++i)
            {
                polygon.push_back({(float)(i->at(0) - laser_x), (float)(-(i->at(1) - laser_y))});
            }
            for(auto i = movebase_polygon2_.cbegin(); i != movebase_polygon2_.cend(); ++i)
            {
                polygon2.push_back({(float)(i->at(0) - laser_x), (float)(-(i->at(1) - laser_y))});
            }
        }

        cout << "obstacle polygon1:";
        for(auto i = polygon.cbegin(); i != polygon.cend(); ++i)
        {
            cout << "[" << *(i->cbegin()) << "," <<  *(i->cbegin() + 1) << "]" << endl;
        }

        cout << "obstacle polygon2:";
        for(auto i = polygon2.cbegin(); i != polygon2.cend(); ++i)
        {
            cout << "[" << *(i->cbegin()) << "," <<  *(i->cbegin() + 1) << "]" << endl;
        }

        for(int i = 0; i < scan.ranges.size(); ++i)
        {
            float angle_tmp = scan.angle_min + i * scan.angle_increment;
            Line l1 = {{0, 0}, {laser_range_max * cos(angle_tmp), laser_range_max * sin(angle_tmp)}};
            Line l2;
            Point crosspoint;
            double length;
            for (auto j = polygon.cbegin(); j != polygon.cend(); ++j)
            {
                l2.p1.x = *(j->cbegin());
                l2.p1.y = *(j->cbegin() + 1);
                if(j + 1 != polygon.cend())
                {
                    l2.p2.x = *((j + 1)->cbegin());
                    l2.p2.y = *((j + 1)->cbegin() + 1);
                }
                else
                {
                    l2.p2.x = *(polygon.cbegin()->cbegin());
                    l2.p2.y = *(polygon.cbegin()->cbegin() + 1);
                }

                int tmp = calCrossPoint(l1, l2, crosspoint, length);
                if(tmp == CROSS || tmp == COLLINEATION)
                {
                    obstacle_thr.push_back(length + obstacle_distance_); 
                    break;
                }
                else if(j + 1 == polygon.cend())
                {
                    obstacle_thr.push_back(0);
                    break;
                }
            }
        }

        for(int i = 0; i < scan.ranges.size(); ++i)
        {
            float angle_tmp = scan.angle_min + i * scan.angle_increment;
            Line l1 = {{0, 0}, {laser_range_max * cos(angle_tmp), laser_range_max * sin(angle_tmp)}};
            Line l2;
            Point crosspoint;
            double length;
            for (auto j = polygon2.cbegin(); j != polygon2.cend(); ++j)
            {
                l2.p1.x = *(j->cbegin());
                l2.p1.y = *(j->cbegin() + 1);
                if(j + 1 != polygon2.cend())
                {
                    l2.p2.x = *((j + 1)->cbegin());
                    l2.p2.y = *((j + 1)->cbegin() + 1);
                }
                else
                {
                    l2.p2.x = *(polygon2.cbegin()->cbegin());
                    l2.p2.y = *(polygon2.cbegin()->cbegin() + 1);
                }

                int tmp = calCrossPoint(l1, l2, crosspoint, length);
                if(tmp == CROSS || tmp == COLLINEATION)
                {
                    obstacle_thr2.push_back(length + obstacle_distance_); 
                    break;
                }
                else if(j + 1 == polygon2.cend())
                {
                    obstacle_thr2.push_back(0);
                    break;
                }
            }
        }

        ROS_INFO("laser size:%ld , obstacle size:%ld", scan.ranges.size(), obstacle_thr.size());
        // for(auto i = obstacle_thr.cbegin(); i != obstacle_thr.cend(); ++i)
        // {
        //     printf("%0.5f\r\n", *i); 
        // }
        scan_ready_ = 1;
    }

    bool obstacle_temp = 0;
    int obstacle_cnt = 0;
    bool obstacle_temp2 = 0;
    int obstacle_cnt2 = 0;
    for(auto j = obstacle_thr.cbegin(); j != obstacle_thr.cend(); ++j)
    {
        if(scan.ranges[obstacle_cnt] < *j)
        {
            obstacle_temp |= 1;
        }
        obstacle_cnt++;
    }

    for(auto j = obstacle_thr2.cbegin(); j != obstacle_thr2.cend(); ++j)
    {
        if(scan.ranges[obstacle_cnt2] < *j)
        {
            obstacle_temp2 |= 1;
        }
        obstacle_cnt2++;
    }

    have_obstacle_ = obstacle_temp;
    have_obstacle2_ = obstacle_temp2;
    if(have_obstacle_) ROS_INFO("obstacle stop!");
    if(have_obstacle2_) ROS_INFO("obstacle slowdown!");
    last_scan_time_ = ros::Time::now();
}




bool isEqual(double num1, double num2, double eps)
{
    return (std::fabs(num1 - num2) <= eps);
}

bool PointIsONLine(Point p, Line l)
{
    return(((p.x >= l.p1.x && p.x <= l.p2.x) || (p.x <= l.p1.x && p.x >= l.p2.x) || isEqual(p.x, l.p1.x) || isEqual(p.x, l.p2.x)) &&  \
            ((p.y >= l.p1.y && p.y <= l.p2.y) || (p.y <= l.p1.y && p.y >= l.p2.y) || isEqual(p.y, l.p1.y) || isEqual(p.y, l.p2.y)));
}

int calCrossPoint(Line L1, Line L2, Point& P, double& length)
{
    double   A1,   B1,   C1,   A2,   B2,   C2; 
    A1   =   L1.p2.y   -   L1.p1.y; 
    B1   =   L1.p1.x   -   L1.p2.x; 
    C1   =   L1.p2.x   *   L1.p1.y   -   L1.p1.x   *   L1.p2.y; 
    A2   =   L2.p2.y   -   L2.p1.y; 
    B2   =   L2.p1.x   -   L2.p2.x; 
    C2   =   L2.p2.x   *   L2.p1.y   -   L2.p1.x   *   L2.p2.y; 
    if(isEqual(A1 * B2, B1 * A2))
    {
        if(isEqual((A1 + B1) * C2, (A2 + B2) * C1)) 
        {
            double distance = 0, tmp;
            if(L1.p1.x != 0 || L1.p1.y != 0)
            {
                tmp = sqrt(pow(L1.p1.x, 2) + pow(L1.p1.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L1.p1.x;
                    P.y = L1.p1.y;
                }
            }
            if((L1.p2.x != 0 || L1.p2.y != 0))
            {
                tmp = sqrt(pow(L1.p2.x, 2) + pow(L1.p2.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L1.p2.x;
                    P.y = L1.p2.y;
                }
            }
            if((L2.p1.x != 0 || L2.p1.y != 0))
            {
                tmp = sqrt(pow(L2.p1.x, 2) + pow(L2.p1.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L2.p1.x;
                    P.y = L2.p1.y;
                }
            }
            if((L2.p2.x != 0 || L2.p2.y != 0))
            {
                tmp = sqrt(pow(L2.p2.x, 2) + pow(L2.p2.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L2.p2.x;
                    P.y = L2.p2.y;
                }
            }
            length = distance;
            return COLLINEATION;
        }
        else
        {
            length = 0;
            return PARALLEL;
        }
    }
    else
    {
        P.x = (B2 * C1 - B1 * C2) / (A2 * B1 - A1 * B2);
        P.y = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);
        if(PointIsONLine(P, L1) && PointIsONLine(P, L2))
        {
            length = sqrt(pow(P.x, 2) + pow(P.y, 2));
            return CROSS; 
        }
        else
        {
            length = 0;
            return NONE;
        }
        
    }
}


//dir p1->p2
double calPathAngle(Point p1, Point p2)
{
    double angle = 0;
    angle = atan2(p2.y - p1.y, p2.x - p1.x);
    return angle;
}

double DegreeToRad(double degree)
{
    return degree * M_PI / 180.0;
}

double RadToDegree(double rad)
{
    return rad * 180.0 / M_PI;
}


int8_t PointIsAheadOfLine(Line path, Point point)
{
    int8_t pos = 0;
    double tmp = 0;
    double x1 = path.p2.x - path.p1.x;
    double y1 = path.p2.y - path.p1.y;
    double x2 = point.x - path.p2.x;
    double y2 = point.y - path.p2.y;
    // if(sqrt(pow(point.x - path.p2.x, 2) + pow(point.y - path.p2.y, 2)) < POS_EPS) return 0;
    tmp = x1 * x2 + y1 * y2;
    if(tmp >= 0) return 1;
    return 0;
}


