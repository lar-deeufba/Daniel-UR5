#ifndef FINGER_HPP
#define FINGER_HPP

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <ros/ros.h>

class RFinger{
  private:
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_input status;
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output data;
    ros::NodeHandle n;
    ros::Publisher f;
    int speed=125,force=150;
  public:
    RFinger(ros::NodeHandle n_);
    bool is_subbed=false;
    void init();
    void set_state(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& now);
    void reset();
    void close();
    void open();
    void set_speed(int speedo);
    void set_force(int forceo);
    void set_position(int position);
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_input get_status();
    int get_speed();
    int get_force();
    int get_position();
    int has_obj();
};


#endif


