#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <init_skill_server/init_skill_server.h>
#include "ur5.hpp"
#include "rfinger.hpp"

void saveData(sensor_msgs::JointState state,geometry_msgs::PoseStamped p, int gripper_pos,int obj,double time){


       ofstream csv;
       csv.open("/home/fastenufba/logs UR/dados.csv", std::ios_base::app);

       csv << time << "," <<  0 <<  "," << state.position[0] << "," <<  state.position[1] << ","  << state.position[2] << "," <<  state.position[3] << ","  << state.position[4] << "," <<  state.position[5] << ","
              << state.velocity[0] << "," <<  state.velocity[1] << ","  << state.velocity[2] << "," <<  state.velocity[3]  << "," << state.velocity[4] << "," <<  state.velocity[5] << ","
              << p.pose.position.x << "," << p.pose.position.y << ","  << p.pose.position.z << "," << p.pose.orientation.x << "," << p.pose.orientation.y << ","  << p.pose.orientation.z << ","
              << 0  << ","  << 0  << "," << 0  << "," << 0  << "," << 0  << "," << 0  << ","
              << 0  << "," << 0  << "," << 0  << "," << gripper_pos << "," << obj << "," << endl;

       cout << "Dados Salvos" << endl;

}


InitSkill::InitSkill(std::string name) :
  as_(nh_, name, boost::bind(&InitSkill::executeCB, this, _1), false),
  action_name_(name)
  {
    as_.start();
  }

InitSkill::~InitSkill()
{
}

void InitSkill::executeCB(const init_skill_msgs::InitSkillGoalConstPtr &goal)
{
    ros::NodeHandle n;
    UR5 ur(n,11);
    RFinger g(n);
    float actual_pos[6];
    float x_init=goal->x;
    float y_init=goal->y;
    float z_init=goal->z;
    float ox=goal->ox;
    float oy=goal->oy;
    float oz=goal->oz;
    bool is_saved=false;
    int tries=0;

    float progress=0;

    bool is_pub=false;
    bool is_pub2=false;
    bool is_init=false;

    ros::Subscriber gripper = n.subscribe("/Robotiq2FGripperRobotInput", 1, &RFinger::set_state,&g);
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, &UR5::set_state,&ur);
    //ros::Subscriber status= n.subscribe("/follow_joint_trajectory/status", 1000, &UR5::set_status,&ur);

    //float pos[]={-1.5944893995868128, -0.11695605913271123, -1.8250539938556116, -1.3067153135882776, 1.4160927534103394, -0.026128117238179982};
    float pos[]={-0.033667866383687794, 0.10607576370239258, -2.3340628782855433, -1.1878765265094202, 1.4660744667053223, -0.005333248768941701};

    ros::Time start_time;

    ros::Duration delta_t;
    double delta_t_sec=0;

    ros::Rate loop_rate(10);
    while(n.ok()){

        if (g.get_status().gACT == 0 && !is_pub && g.is_subbed && !is_init){
            g.init();
            is_init=true;
        }else if (g.get_status().gACT == 0 && is_init){
            tries++;
        }



        if (tries == 20){

            ROS_ERROR("Could not initiate the gripper");
            set_aborted();
            break;


        }


        if (g.get_status().gACT ==1 && !is_pub){

            tries=0;

            for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }
            start_time = ros::Time::now();

            ur.move_arm(actual_pos,0,0);
            ur.move_arm_planner(pos,1,0,9);
            /*
            auto is_ik=ur.ik_move(x_init,y_init,z_init,ox,oy,oz,1,0,5);
            if(!is_ik){
                ROS_ERROR("IK not found");
                set_aborted();
                break;
            }*/
            ur.publish();
            is_pub=true;
        }

        delta_t= ros::Time::now() - start_time;
        delta_t_sec = delta_t.toSec();
        cout << "Time passed: " << delta_t_sec <<endl;

        if (delta_t_sec > 3  && ur.get_status() < 10 && delta_t_sec > 3 && delta_t_sec < 20 && ur.get_status() >= 4){
            ROS_ERROR("Trajectory was aborted");
            set_aborted();
            break;
        }


        if (delta_t_sec > 11 && delta_t_sec < 20 && is_pub && !is_pub2){
            for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }
            //start_time = ros::Time::now();

            ur.move_arm(actual_pos,0,0);
            //ur.move_arm_planner(pos,1,0,5);

            auto is_ik=ur.ik_move(x_init,y_init,z_init,ox,oy,oz,1,0,9);
            if(!is_ik){
                ROS_ERROR("IK not found");
                set_aborted();
                break;
            }
            ur.publish();
            is_pub2=true;
            //
        }

         if (delta_t_sec > 21 && delta_t_sec < 30 && is_pub && is_pub2){
             set_succeeded();
             break;
         }

         if (!is_saved){
             ofstream csv;
             csv.open("/home/fastenufba/logs UR/dados.csv", std::ios_base::app);
             csv << "Time" << "," << "Skill" << "," << "Joint1" << "," << "Joint2" << "," << "Joint3" << "," << "Joint4" << "," << "Joint5" << "," << "Joint6" << ","
                    << "Velocity1" << "," << "Velocity2" << ","<< "Velocity3" << ","<< "Velocity4" << ","<< "Velocity5" << ","<< "Velocity6" << ","
                    << "x" << ","  << "y" << ","  << "z" << ","  << "ox" << "," << "oy" << "," << "oz" << ","
                       << "tx" << ","  << "ty" << ","  << "tz" << ","  << "cx" << "," << "cy" << "," << "cz" << ","
                          << "ax" << ","  << "ay" << ","  << "az" << ","  << "Gripper Pos" << "," << "Gripper Status" << endl;
             is_saved=true;
             csv.close();
         }

        if (delta_t_sec > 0 && delta_t_sec < 22){
            progress = delta_t_sec * 100 /21;
            if (progress > 100){
                progress = 100;
            }
            feedback(progress);
            saveData(ur.get_state(),ur.get_end_position(),g.get_position(),g.has_obj(),delta_t_sec);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void InitSkill::set_succeeded(std::string outcome)
{
  result_.percentage = 100;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Succeeded";
  result_.outcome = outcome;
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  as_.setSucceeded(result_);
}

void InitSkill::set_aborted(std::string outcome)
{
  result_.percentage = 0;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Aborted";
  result_.outcome = outcome;
  ROS_INFO("%s: Aborted", action_name_.c_str());
  as_.setAborted(result_);
}
void InitSkill::feedback(float percentage)
{
  feedback_.percentage = percentage;
  feedback_.skillStatus = action_name_.c_str();
  feedback_.skillStatus += " Executing";
  ROS_INFO("%s: Executing. Percentage: %f%%.", action_name_.c_str(), percentage);
  as_.publishFeedback(feedback_);
}

bool InitSkill::check_preemption()
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
