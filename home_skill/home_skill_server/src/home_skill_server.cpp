#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <home_skill_server/home_skill_server.h>
#include "ur5.hpp"


void saveData(sensor_msgs::JointState state,geometry_msgs::PoseStamped p, double time){
    ofstream csv;
    csv.open("/home/fastenufba/logs UR/dados.csv", std::ios_base::app);

    csv << time << "," <<  3 <<  "," << state.position[0] << "," <<  state.position[1] << ","  << state.position[2] << "," <<  state.position[3] << ","  << state.position[4] << "," <<  state.position[5] << ","
           << state.velocity[0] << "," <<  state.velocity[1] << ","  << state.velocity[2] << "," <<  state.velocity[3]  << "," << state.velocity[4] << "," <<  state.velocity[5] << ","
           << p.pose.position.x << "," << p.pose.position.y << ","  << p.pose.position.z << "," << p.pose.orientation.x << "," << p.pose.orientation.y << ","  << p.pose.orientation.z << ","
           << 0  << ","  << 0  << "," << 0  << "," << 0  << "," << 0  << "," << 0  << ","
           << 0  << "," << 0  << "," << 0  << "," << 0 << "," << 0 << "," << endl;

    cout << "Dados Salvos" << endl;

}


HomeSkill::HomeSkill(std::string name) :
  as_(nh_, name, boost::bind(&HomeSkill::executeCB, this, _1), false),
  action_name_(name)
  {
    as_.start();
  }

HomeSkill::~HomeSkill()
{
}

void HomeSkill::executeCB(const home_skill_msgs::HomeSkillGoalConstPtr &goal)
{
    ros::NodeHandle n;
    UR5 ur(n,11);

    ros::Subscriber sub = n.subscribe("/joint_states", 1000, &UR5::set_state,&ur);
    //ros::Subscriber status= n.subscribe("/follow_joint_trajectory/status", 1000, &UR5::set_status,&ur);

    float progress=0;

    //float pos[]={-1.3444283644305628, 0.8068975210189819, -2.6561408678637903, -1.2787564436541956, 1.219605803489685, -0.026163880025045216};
    float pos[]={-0.033583943043844044, 0.8079999685287476, -2.7507532278644007, -1.191949192677633, -0.017171684895650685, -0.0053094069110315445};
    float actual_pos[6];
    bool is_pub=false;
    //float x=-0.2;
    //float y=0;
    //float z=0.5;
    ros::Time start_time;

    ros::Duration delta_t;
    double delta_t_sec=0;

    ros::Rate loop_rate(10);
    while(n.ok()){


        for(int i=0; i < 6;i++){
            actual_pos[i]= ur.get_state().position[i];
        }

        ur.move_arm(actual_pos,0,0);

        //ur.ik_move(x,y,z,1.5707600752459925,0,0,1,0,8); //0, -1.5707600752459925, 0, -1.570796314870016, 0, 0
        ur.move_arm_planner(pos,1,0,10);

        if (!is_pub && ur.get_state().position[1]!=0){
            start_time = ros::Time::now();
            ur.publish();
            is_pub=true;
        }


        delta_t= ros::Time::now() - start_time;
        delta_t_sec = delta_t.toSec();
        cout << "Time passed: " << delta_t_sec <<endl;
        /*
        if (delta_t_sec > 3 && delta_t_sec < 20 && ur.get_status() >= 4 && ur.get_status() < 10 ){
            ROS_ERROR("Trajectory was aborted");
            set_aborted();
            break;
        }
        */
        if (delta_t_sec > 11 && delta_t_sec < 20 && is_pub){
            set_succeeded();
            break;
        }

        cout << ur.get_end_position() << endl;

        if (delta_t_sec > 0 && delta_t_sec < 12){
            progress = delta_t_sec * 100 /11;
            if (progress > 100){
                progress = 100;
            }
            feedback(progress);
            saveData(ur.get_state(),ur.get_end_position(),delta_t_sec);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}

void HomeSkill::set_succeeded(std::string outcome)
{
  result_.percentage = 100;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Succeeded";
  result_.outcome = outcome;
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  as_.setSucceeded(result_);
}

void HomeSkill::set_aborted(std::string outcome)
{
  result_.percentage = 0;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Aborted";
  result_.outcome = outcome;
  ROS_INFO("%s: Aborted", action_name_.c_str());
  as_.setAborted(result_);
}
void HomeSkill::feedback(float percentage)
{
  feedback_.percentage = percentage;
  feedback_.skillStatus = action_name_.c_str();
  feedback_.skillStatus += " Executing";
  ROS_INFO("%s: Executing. Percentage: %f%%.", action_name_.c_str(), percentage);
  as_.publishFeedback(feedback_);
}

bool HomeSkill::check_preemption()
{
  if (as_.isPreemptRequested() || !ros::ok()){
    result_.percentage = 0;
    result_.skillStatus = action_name_.c_str();
    result_.skillStatus += ": Preempted";
    result_.outcome = "preempted";
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted(result_);
    return true;
  }
  else{
    return false;
  }
}
