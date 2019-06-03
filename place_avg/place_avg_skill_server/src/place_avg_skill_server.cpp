#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <place_avg_skill_server/place_avg_skill_server.h>
#include "ur5.hpp"
#include "Kinect.hpp"
#include "ObjDetection.hpp"
#include "rfinger.hpp"


void saveData(sensor_msgs::JointState state,geometry_msgs::PoseStamped p, int gripper_pos,int obj,double time,geometry_msgs::Point pos_obj,geometry_msgs::Point pos_cam,float ax,float ay,float az){
    ofstream csv;
    csv.open("/home/fastenufba/logs UR/dados.csv", std::ios_base::app);

    csv << time << "," << 2 << "," << state.position[0] << "," <<  state.position[1] << ","  << state.position[2] << "," <<  state.position[3] << ","  << state.position[4] << "," <<  state.position[5] << ","
           << state.velocity[0] << "," <<  state.velocity[1] << ","  << state.velocity[2] << "," <<  state.velocity[3]  << "," << state.velocity[4] << "," <<  state.velocity[5] << ","
           << p.pose.position.x << "," << p.pose.position.y << ","  << p.pose.position.z << "," << p.pose.orientation.x << "," << p.pose.orientation.y << ","  << p.pose.orientation.z << ","
           << pos_obj.x  << ","  << pos_obj.y  << "," << pos_obj.z  << "," << pos_cam.x  << "," << pos_cam.y  << "," << pos_cam.z  << ","
           << ax  << "," << ay  << "," << az  << "," << gripper_pos << "," << obj << "," << endl;

    cout << "Dados Salvos" << endl;
}


PlaceAvgSkill::PlaceAvgSkill(std::string name) :
  as_(nh_, name, boost::bind(&PlaceAvgSkill::executeCB, this, _1), false),
  action_name_(name)
  {
    as_.start();
  }

PlaceAvgSkill::~PlaceAvgSkill()
{
}

void PlaceAvgSkill::executeCB(const place_avg_skill_msgs::PlaceAvgSkillGoalConstPtr &goal)
{
    ros::NodeHandle n;
    UR5 ur(n,11);
    RFinger g(n);
    float actual_pos[6];
    float x_init=-0.1;
    float y_init=-0.3;
    float z_init=0.65;
    float ox=-1.57;
    float oy=0;
    float oz=-3.14;
    Kinect cam;
    float progress=0;

    geometry_msgs::Point po,t_h,t_p,pf;

    bool is_pub=false,is_pub2=false, is_open=false,is_pub3 = false,is_pub4=false,is_pub5=false;
    int state=0;
    float pos[]={1.6664173603057861, -0.8940451780902308, -1.6315153280841272, -2.104358498250143, 1.486076831817627, -0.005740944539205373};
    float pos2[]={1.504563331604004, -0.13586122194399053, -1.5442407766925257, -1.544854466115133, 1.5493251085281372, -0.005273167287008107};
    ros::Subscriber gripper = n.subscribe("/Robotiq2FGripperRobotInput", 1, &RFinger::set_state,&g);
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, &UR5::set_state,&ur);
    ros::Subscriber status= n.subscribe("/follow_joint_trajectory/status", 1000, &UR5::set_status,&ur);

    ros::Subscriber camera = n.subscribe("/camera/color/image_raw", 1, &Kinect::set_image,&cam);
    ros::Subscriber depth = n.subscribe("/camera/depth/image_raw", 1, &Kinect::set_image_depth,&cam);
    ros::Subscriber cloud= n.subscribe("/camera/depth_registered/points", 1000, &Kinect::set_point,&cam);

    ros::Time start_time;

    ros::Duration delta_t;
    double delta_t_sec=0;

    ros::Rate loop_rate(10);
    while(n.ok()){
        if (g.get_status().gACT ==1 && !is_pub && state==0){
            for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }
            start_time = ros::Time::now();

            ur.move_arm(actual_pos,0,0);

            ur.move_arm_planner(pos,1,0,8);
            ur.publish();
            is_pub=true;
            state = 1;
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
        if (delta_t_sec > 9 && delta_t_sec < 20 && !is_pub2 && is_pub && state == 1){
            for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }

            ur.move_arm(actual_pos,0,0);

            //ur.move_arm_planner(pos2,1,0,5);
            auto is_ik=ur.ik_move(x_init,y_init,z_init,ox,oy,oz,1,0,9);
            if(!is_ik){
                ROS_ERROR("IK not found");
                set_aborted();
                break;
            }
            ur.publish();
            is_pub2=true;
            state = 2;
        }

        if (state == 2 && delta_t_sec > 20 && is_pub2 && !is_pub3){
            auto tag= ObjDetec::aprilDetect(cam,2,false);
            int xo=tag.x;
            int yo=tag.y;

            auto po=ObjDetec::pixelTo3DPoint(cam.get_cloud(),xo,yo);
            auto testenan=po.z;
            cout << "Posicao tag:  " << endl << po << endl;
            if(is_pub3==false && !isnan(testenan)){

                for(int i=0; i < 6;i++){
                    actual_pos[i]= ur.get_state().position[i];
                }

                ur.move_arm(actual_pos,0,0);

                po.x=po.x - 0.028;
                po.y=po.y - 0.085;
                po.z=po.z - 0.42;
                //eox=ObjDetec::oy;
                pf=ur.homotransform(po);

                auto get_pos_z=ur.get_end_position().pose.position.z;

                auto is_ik=ur.ik_move(pf.x,pf.y,get_pos_z,ox,oy,oz,1,0,5); //po.z-get_pos_z+0.39
                if(!is_ik){
                    ROS_ERROR("IK not found");
                    set_aborted();
                    break;
                }
                ur.publish();
                is_pub3=true;
                state=3;


               }

        }

        if (delta_t_sec > 26 && state == 3 && !is_pub4 && is_pub3){
            for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }

            ur.move_arm(actual_pos,0,0);


            auto get_pos_x=ur.get_end_position().pose.position.x;
            auto get_pos_y=ur.get_end_position().pose.position.y;

            auto is_ik=ur.ik_move(get_pos_x,get_pos_y,pf.z,ox,oy,oz,1,0,5); //po.z-get_pos_z+0.39
            if(!is_ik){
                ROS_ERROR("IK not found");
                set_aborted();
                break;
            }
            ur.publish();
            is_pub4=true;
            state=4;

        }

        if (delta_t_sec > 32 && delta_t_sec < 40 && !is_open && is_pub4){
            g.open();
            is_open=true;
        }

        if (delta_t_sec > 34 && delta_t_sec < 40 && is_open && !is_pub5){
            if (g.has_obj() == 2){
                ROS_ERROR("Gripper didnt open");
                set_aborted();
                break;
            }else{
                for(int i=0; i < 6;i++){
                    actual_pos[i]= ur.get_state().position[i];
                }

                ur.move_arm(actual_pos,0,0);

                ur.move_arm_planner(pos2,1,0,12);
                ur.publish();
                is_pub5=true;

            }
        }

        if (delta_t_sec > 47 && is_pub5){
            set_succeeded();
            break;
        }

        if (delta_t_sec > 0 && delta_t_sec < 47){
            progress = delta_t_sec * 100 /47;
            if (progress > 100){
                progress = 100;
            }
            feedback(progress);
            saveData(ur.get_state(),ur.get_end_position(),g.get_position(),g.has_obj(),delta_t_sec,t_h,t_p,ObjDetec::ox,ObjDetec::oy,ObjDetec::oz);
        }
        cam.show_image();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PlaceAvgSkill::set_succeeded(std::string outcome)
{
  result_.percentage = 100;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Succeeded";
  result_.outcome = outcome;
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  as_.setSucceeded(result_);
}

void PlaceAvgSkill::set_aborted(std::string outcome)
{
  result_.percentage = 0;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Aborted";
  result_.outcome = outcome;
  ROS_INFO("%s: Aborted", action_name_.c_str());
  as_.setAborted(result_);
}
void PlaceAvgSkill::feedback(float percentage)
{
  feedback_.percentage = percentage;
  feedback_.skillStatus = action_name_.c_str();
  feedback_.skillStatus += " Executing";
  ROS_INFO("%s: Executing. Percentage: %f%%.", action_name_.c_str(), percentage);
  as_.publishFeedback(feedback_);
}

bool PlaceAvgSkill::check_preemption()
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
