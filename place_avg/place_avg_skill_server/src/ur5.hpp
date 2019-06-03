#ifndef UR5_HPP
#define UR5_HPP

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "Eigen/Dense"
#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace Eigen;

class UR5{
private:
    sensor_msgs::JointState state;
    ros::NodeHandle n;
    ros::Publisher arm;
    int status;
    control_msgs::FollowJointTrajectoryActionGoal msg;
public:
    UR5(ros::NodeHandle n_,int number_of_movements);
    sensor_msgs::JointState get_state(){ return state; };
    int get_status(){ return status; };
    void set_state(const sensor_msgs::JointState::ConstPtr& now);
    void set_status(const actionlib_msgs::GoalStatusArrayConstPtr& now);
    void publish();
    void move_arm(float* pos,float duration,int movement_number);
    void move_arm_speed(float* speed,float duration,int movement_number);
    void move_arm_acc(float* acc,float duration,int movement_number);
    void move_arm_planner(float* pos,int movement_number,float startTime,float endTime);
    void ik_move(float x,float y,float z,float theta,int movement_number,float startTime,float endTime);
    bool ik_move(float x,float y,float z,float thetax,float thetay,float thetaz,int movement_number,float startTime,float endTime);
    void ik_move2(float x,float y,float z,float theta,int movement_number,float startTime,float endTime);
    geometry_msgs::PoseStamped get_end_position();
    geometry_msgs::Point homotransform(geometry_msgs::Point point);
    geometry_msgs::PoseStamped homotransform(geometry_msgs::Point point,float ax,float ay,float az);
};


#endif
