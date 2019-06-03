#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pick_printer_skill_server/pick_printer_skill_server.h>
#include "ur5.hpp"
#include "Kinect.hpp"
#include "ObjDetection.hpp"
#include "rfinger.hpp"


void saveData(sensor_msgs::JointState state,geometry_msgs::PoseStamped p, int gripper_pos,int obj,double time,geometry_msgs::Point pos_obj,geometry_msgs::Point pos_cam,float ax,float ay,float az){
    ofstream csv;
    csv.open("/home/fastenufba/logs UR/dados.csv", std::ios_base::app);

    csv << time << "," << 1 << "," << state.position[0] << "," <<  state.position[1] << ","  << state.position[2] << "," <<  state.position[3] << ","  << state.position[4] << "," <<  state.position[5] << ","
           << state.velocity[0] << "," <<  state.velocity[1] << ","  << state.velocity[2] << "," <<  state.velocity[3]  << "," << state.velocity[4] << "," <<  state.velocity[5] << ","
           << p.pose.position.x << "," << p.pose.position.y << ","  << p.pose.position.z << "," << p.pose.orientation.x << "," << p.pose.orientation.y << ","  << p.pose.orientation.z << ","
           << pos_obj.x  << ","  << pos_obj.y  << "," << pos_obj.z  << "," << pos_cam.x  << "," << pos_cam.y  << "," << pos_cam.z  << ","
           << ax  << "," << ay  << "," << az  << "," << gripper_pos << "," << obj << "," << endl;

    cout << "Dados Salvos" << endl;
}


PickPrinterSkill::PickPrinterSkill(std::string name) :
  as_(nh_, name, boost::bind(&PickPrinterSkill::executeCB, this, _1), false),
  action_name_(name)
  {
    as_.start();
  }

PickPrinterSkill::~PickPrinterSkill()
{
}

void PickPrinterSkill::executeCB(const pick_printer_skill_msgs::PickPrinterSkillGoalConstPtr &goal)
{
    ros::NodeHandle n;
    Kinect cam;
    UR5 ur(n,11);
    float testenan=0;
    int states=0;
    float actual_pos[6];
    bool is_pub = false;
    bool is_pub2 = false;
    bool is_pubm=false;
    bool is_pub3 = false;
    bool is_pub4 = false;
    bool is_pub5=false;
    float x_init=0;
    float y_init=0;
    float z_init=0;
    float ox=0;
    float oy=0;
    float oz=0;
    geometry_msgs::Point po,t_h,t_p;
    //float angulo_rad=0;
    //float angulo;
    RFinger g(n);
    g.init();
    //g.set_position(127);
    int xo,yo;

    float progress=0;

    ros::Subscriber gripper = n.subscribe("/Robotiq2FGripperRobotInput", 1, &RFinger::set_state,&g);
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, &UR5::set_state,&ur);
    ros::Subscriber camera = n.subscribe("/camera/color/image_raw", 1, &Kinect::set_image,&cam);
    ros::Subscriber depth = n.subscribe("/camera/depth/image_raw", 1, &Kinect::set_image_depth,&cam);
    ros::Subscriber cloud= n.subscribe("/camera/depth_registered/points", 1000, &Kinect::set_point,&cam);
    ros::Subscriber status= n.subscribe("/follow_joint_trajectory/status", 1000, &UR5::set_status,&ur);

    ros::Time start_time;

    ros::Duration delta_t;
    double delta_t_sec=0;

    ros::Rate loop_rate(10);
    while(n.ok()){
        unsigned int tags[]={goal->tag1,goal->tag2};
        Point out[2];

        ObjDetec::multAprilDetect(cam,tags,out,2,false);
        int x=out[0].x;
        int y=out[0].y;
        int x2=out[1].x;
        int y2=out[1].y;
        auto peca= ObjDetec::aprilDetect(cam,goal->tagO,false);
        cout << peca << endl;

        if (cam.cloud_filled && peca.x > 0 && peca.y > 0){
            int xh=peca.x;
            int yh=peca.y;
            t_p=ObjDetec::pixelTo3DPoint(cam.get_cloud(),xh,yh);
            t_p.x=t_p.x - 0.028;
            t_p.y=t_p.y - 0.11;
            t_p.z=t_p.z -0.32;
            t_h=ur.homotransform(t_p);
        }else{
            t_h.x=0;
            t_h.y=0;
            t_h.z=0;
            t_p.x=0;
            t_p.y=0;
            t_p.z=0;
            ObjDetec::ox=0;
            ObjDetec::oy=0;
            ObjDetec::oz=0;
        }

        //Estado 0: Posição inicial

        if (states == 0 && is_pub == false && ur.get_state().position[1] != 0 && g.get_status().gACT ==1){
            g.set_speed(25);
            g.set_force(25);
            g.set_position(127);
            x_init=ur.get_end_position().pose.position.x;
            y_init=ur.get_end_position().pose.position.y;
            z_init=ur.get_end_position().pose.position.z;
            ox=ur.get_end_position().pose.orientation.x;
            oy=ur.get_end_position().pose.orientation.y;
            oz=ur.get_end_position().pose.orientation.z;
            /*
            for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }


            ur.move_arm(actual_pos,0,0);
            auto has_ik=ur.ik_move(x_init,y_init,z_init,ox,oy,oz,1,0,5);
            if(!has_ik){
                set_aborted();
                break;
            }
            ur.publish();
            */

            start_time = ros::Time::now();
            is_pub=true;

        }

        delta_t= ros::Time::now() - start_time;
        delta_t_sec = delta_t.toSec();
        cout << "Time passed: " << delta_t_sec <<endl;

        if (ur.get_status() >= 4 && ur.get_status() < 10 && delta_t_sec > 3 && delta_t_sec < 44){
            ROS_ERROR("Trajectory was aborted");
            set_aborted();
            break;
        }

        if (delta_t_sec > 7 && states==0 && delta_t_sec < 20){
            if (g.get_position() != 127){
                ROS_ERROR("Gripper Error");
                set_aborted();
                break;
            }else if((x == 0 && y == 0) || (x2 == 0 && y2 == 0)){
                ROS_ERROR("Printer not found");
                set_aborted();
                break;
            }else{
                states=1;
                cout << "Mudei o estado" << endl;
            }


        }
        //Estado 1 Move para perto da impressora

        if (x > 0 && y > 0 && (x < cam.get_image().cols) && (y < cam.get_image().rows) && cam.cloud_filled && states==1){
            auto p=ObjDetec::pixelTo3DPoint(cam.get_cloud(),x,y);
            auto p2=ObjDetec::pixelTo3DPoint(cam.get_cloud(),x2,y2);
            testenan=p.z;
            cout << "Posicao tag1:  " << endl <<p << endl;
            cout << "Posicao tag2:  " << endl <<p2 << endl;


            if(is_pub2==false && !isnan(testenan)){
                auto pm=p;
                pm.x=(p.x+p2.x)/2;
                pm.y=(p.y+p2.y)/2+0.1;
                pm.z=(p.z+p2.z)/2 -0.45;
                for(int i=0; i < 6;i++){
                    actual_pos[i]= ur.get_state().position[i];
                }

                ur.move_arm(actual_pos,0,0);
                //auto get_pos_x=ur.get_end_position().pose.position.x;
                //auto get_pos_y=ur.get_end_position().pose.position.y;
                //auto get_pos_z=ur.get_end_position().pose.position.z;
                auto pf=ur.homotransform(pm);
                cout << pf << endl;
                auto has_ik=ur.ik_move(pf.x,pf.y,pf.z,ox,oy,oz,1,0,5);
                if(!has_ik){
                    set_aborted();
                    break;
                }
                ur.publish();
                is_pubm=true;
                states=2;

               }


        }

        //Estado m Detecta o objeto e alinha

        if (states ==2 && delta_t_sec > 14 && is_pubm){

            //auto obj=ObjDetec::objDetec(cam,"cone2.png");
            //auto obj=ObjDetec::aprilDetect(cam,13,false);
            xo=peca.x;
            yo=peca.y;
            /*
            angulo=ObjDetec::ox*180/M_PI;
            angulo_rad=ObjDetec::ox;
            if (angulo < 0){
                angulo = - angulo;
                angulo_rad= - angulo_rad;
            }
            if (angulo > 90){
                 angulo = angulo - 90;
                 angulo_rad= angulo_rad - 1.5707963268;
            }

            if (angulo > 180){
                angulo = angulo - 180;
                angulo_rad= angulo_rad - M_PI;
            }

            if (angulo > 270){
                angulo = angulo - 270;
                angulo_rad= angulo_rad - 3*1.5707963268;

            }


            cout << "Nova rotação: " << angulo << endl;

            if ( angulo > 10 && angulo < 80){
                cout << "Rotacionar em:" << angulo<< endl;
                cout << "Rotacionar em(rad):" << angulo_rad << endl;
            }else{
                angulo_rad=0;
            }
            */
            if (xo == 0 && yo == 0){
                ROS_ERROR("Object Tag not Found");
                set_aborted();
                break;
            }

            if (xo > 0 && yo > 0 && (xo < cam.get_image().cols) && (yo < cam.get_image().rows) && cam.cloud_filled ){
              po=ObjDetec::pixelTo3DPoint(cam.get_cloud(),xo,yo);
              testenan=po.z;
              cout << "Posicao Objeto:  " << endl << po << endl;
              if(ur.get_state().position[1] != 0 && is_pub3==false && !isnan(testenan)){
                for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
                }

                ur.move_arm(actual_pos,0,0);
                auto get_pos_x=ur.get_end_position().pose.position.x;
                //auto get_pos_y=ur.get_end_position().pose.position.y;
                //auto get_pos_z=ur.get_end_position().pose.position.z;
                //ur.ik_move(get_pos_x,get_pos_y-po.x -0.012,z_init,angulo_rad,0,1.5708,1,0,5);
                po.x=po.x - 0.045;
                po.y=po.y - 0.11;
                po.z=po.z - 0.32;
                auto pf=ur.homotransform(po);
                cout << pf << endl;
                auto has_ik=ur.ik_move(get_pos_x,pf.y,pf.z,ox,oy,oz,1,0,5);
                if(!has_ik){
                    set_aborted();
                    break;
                }
                ur.publish();
                is_pub2=true;
                states=3;
              }

            }


        }

        //Estado 2 Move para o objeto

        if (states ==3 && delta_t_sec > 21 && is_pub2){


              testenan=po.z;

              if(ur.get_state().position[1] != 0 && is_pub3==false && !isnan(testenan)){
                for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
                }

                ur.move_arm(actual_pos,0,0);
                //auto get_pos_x=ur.get_end_position().pose.position.x;
                auto get_pos_y=ur.get_end_position().pose.position.y;
                auto get_pos_z=ur.get_end_position().pose.position.z;
                //ur.ik_move(get_pos_x+po.z -0.33,get_pos_y,get_pos_z,0,0,1.5708,1,0,5);
                auto pf=ur.homotransform(po);
                cout << pf << endl;
                auto has_ik=ur.ik_move(pf.x,get_pos_y,get_pos_z,ox,oy,oz,1,0,5);
                if(!has_ik){
                    set_aborted();
                    break;
                }
                ur.publish();
                is_pub3=true;
                states=4;
              }

            //}

        }
        //Estado 3 Pega o objeto

        if(states == 4 && delta_t_sec > 28 && is_pub3){

            g.close();
            states = 5;
        }

        //Estado 4 Levanta o objeto

        if (states == 5 && delta_t_sec > 32){
          for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }
           ur.move_arm(actual_pos,0,0);
           auto get_pos_x=ur.get_end_position().pose.position.x;
           auto get_pos_y=ur.get_end_position().pose.position.y;
           auto get_pos_z=ur.get_end_position().pose.position.z;
           auto has_ik=ur.ik_move(get_pos_x,get_pos_y,get_pos_z+0.03,ox,oy,oz,1,0,5);
           if(!has_ik){
               set_aborted();
               break;
           }
           ur.publish();
           is_pub4=true;
           states=6;
        }

        //Estado 5 Retira o objeto

        if (states == 6 && delta_t_sec > 38 && is_pub4){
          for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }
           auto get_pos_z=ur.get_end_position().pose.position.z;
           ur.move_arm(actual_pos,0,0);
           auto has_ik=ur.ik_move(x_init,y_init,get_pos_z,ox,oy,oz,1,0,5);
           if(!has_ik){
               set_aborted();
               break;
           }
           ur.publish();
           is_pub5=true;
           states=7;
        }

        //Estado 6 Checa se pegou o objeto

        if (states == 7 && delta_t_sec > 44 && is_pub5){
          if (g.has_obj() == 2){
              cout << "Got the obj" << endl;
              set_succeeded();
              break;
          }else{
              ROS_ERROR("OBJ LOST");
              set_aborted();
              break;
          }
          /*
          for(int i=0; i < 6;i++){
                actual_pos[i]= ur.get_state().position[i];
            }
           ur.move_arm(actual_pos,0,0);
           ur.ik_move(x_init+0.1,y_init,z_init-0.2,1.5708,-1.5708,0,1,0,5);
           ur.publish();
           break;
          */
        }
        if (delta_t_sec > 0 && delta_t_sec < 50){
            progress = delta_t_sec * 100 /44;
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

void PickPrinterSkill::set_succeeded(std::string outcome)
{
  result_.percentage = 100;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Succeeded";
  result_.outcome = outcome;
  ROS_INFO("%s: Succeeded", action_name_.c_str());
  as_.setSucceeded(result_);
}

void PickPrinterSkill::set_aborted(std::string outcome)
{
  result_.percentage = 0;
  result_.skillStatus = action_name_.c_str();
  result_.skillStatus += ": Aborted";
  result_.outcome = outcome;
  ROS_INFO("%s: Aborted", action_name_.c_str());
  as_.setAborted(result_);
}
void PickPrinterSkill::feedback(float percentage)
{
  feedback_.percentage = percentage;
  feedback_.skillStatus = action_name_.c_str();
  feedback_.skillStatus += " Executing";
  ROS_INFO("%s: Executing. Percentage: %f%%.", action_name_.c_str(), percentage);
  as_.publishFeedback(feedback_);
}

bool PickPrinterSkill::check_preemption()
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
