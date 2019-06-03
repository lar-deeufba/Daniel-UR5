#include "rfinger.hpp"

RFinger::RFinger(ros::NodeHandle n_){
  n=n_;
  f=n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 1);
}

void RFinger::set_state(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& now){
  status.gACT=now->gACT;
  status.gGTO=now->gGTO;
  status.gSTA=now->gSTA;
  status.gOBJ=now->gOBJ;
  status.gFLT=now->gFLT;
  status.gPR=now->gPR;
  status.gPO=now->gPO;
  status.gCU=now->gCU;
  is_subbed=true;
}

void RFinger::init(){
  if (status.gACT == 0){
    data.rACT=1;
    data.rGTO=1;
    data.rATR=0;
    data.rPR=0;
    data.rSP=255;
    data.rFR=150;
    speed=255;
    force=150;
    f.publish(data);
    ROS_INFO("Gripper initialized");
  }else if(status.gACT == 1){
    ROS_WARN("Gripper already on");
  }
}

void RFinger::reset(){
  if(status.gACT == 1){
    data.rACT=0;
    data.rGTO=0;
    data.rATR=0;
    data.rPR=0;
    data.rSP=0;
    data.rFR=0;
    f.publish(data);
    ROS_INFO("Gripper Restarted");
  }else{
    ROS_WARN("Gripper already off");
  }
}

void RFinger::close(){
  if(status.gACT == 1){
    data.rACT=1;
    data.rGTO=1;
    data.rATR=0;
    data.rPR=255;
    data.rSP=speed;
    data.rFR=force;
    f.publish(data);
    ROS_INFO("Gripper Closed");
  }else{
    ROS_ERROR("Init the Gripper");
  }
}

void RFinger::open(){
  if(status.gACT == 1){
    data.rACT=1;
    data.rGTO=1;
    data.rATR=0;
    data.rPR=0;
    data.rSP=speed;
    data.rFR=force;
    f.publish(data);
    ROS_INFO("Gripper Open");
  }else{
    ROS_ERROR("Init the Gripper");
  }
}

void RFinger::set_speed(int speedo){
  speed=speedo;
}

void RFinger::set_force(int forceo){
  force=forceo;
}

void RFinger::set_position(int position){
  if(status.gACT == 1){
    data.rACT=1;
    data.rGTO=1;
    data.rATR=0;
    data.rPR=position;
    data.rSP=speed;
    data.rFR=force;
    f.publish(data);
    ROS_INFO("Gripper In Position");
  }else{
    ROS_ERROR("Init the Gripper");
  }
}

robotiq_2f_gripper_control::Robotiq2FGripper_robot_input  RFinger::get_status(){
  return status;
}

int RFinger::get_position(){
  return status.gPR;
}

int RFinger::get_speed(){
  return speed;
}

int RFinger::get_force(){
  return force;
}

int RFinger::has_obj(){
  return status.gOBJ;
}
