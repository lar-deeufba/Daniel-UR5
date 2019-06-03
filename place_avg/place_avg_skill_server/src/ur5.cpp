#include "ur5.hpp"

UR5::UR5(ros::NodeHandle n_,int number_of_movements){
    n=n_;
    arm=n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/follow_joint_trajectory/goal", 1);
    msg.header.stamp = ros::Time::now();

    msg.goal.trajectory.joint_names.push_back("ur_arm_shoulder_pan_joint");
    msg.goal.trajectory.joint_names.push_back("ur_arm_shoulder_lift_joint");
    msg.goal.trajectory.joint_names.push_back("ur_arm_elbow_joint");
    msg.goal.trajectory.joint_names.push_back("ur_arm_wrist_1_joint");
    msg.goal.trajectory.joint_names.push_back("ur_arm_wrist_2_joint");
    msg.goal.trajectory.joint_names.push_back("ur_arm_wrist_3_joint");

    msg.goal.trajectory.points.resize(number_of_movements);
    for (int i=0;i < number_of_movements;i++){
        msg.goal.trajectory.points[i].positions.resize(6);
        msg.goal.trajectory.points[i].velocities.resize(6);
        msg.goal.trajectory.points[i].accelerations.resize(6);
    }
    state.name.resize(6);
    state.velocity.resize(6);
    state.position.resize(6);
    state.effort.resize(6);
}

void UR5::set_state(const sensor_msgs::JointState::ConstPtr& now){
    for (int i=0; i < 6; i++){
        state.name[i]= now->name[i];
        state.velocity[i]=now->velocity[i];
        state.position[i]=now->position[i];

    }
}

void UR5::set_status(const actionlib_msgs::GoalStatusArrayConstPtr& now){
    int size=now->status_list.size();
    status=now->status_list[size - 1].status;
}

void UR5::publish(){
    arm.publish(msg);
}

void UR5::move_arm(float* pos,float duration,int movement_number){
    for (int i=0; i < 6; i++){
        msg.goal.trajectory.points[movement_number].positions[i]=pos[i];
    }
    msg.goal.trajectory.points[movement_number].time_from_start = ros::Duration(duration);
}

void UR5::move_arm_speed(float* speed,float duration,int movement_number){
    for (int i=0; i < 6; i++){
        msg.goal.trajectory.points[movement_number].velocities[i]=speed[i];
    }
    msg.goal.trajectory.points[movement_number].time_from_start = ros::Duration(duration);
}

void UR5::move_arm_acc(float* acc,float duration,int movement_number){
    for (int i=0; i < 6; i++){
        msg.goal.trajectory.points[movement_number].accelerations[i]=acc[i];
    }
    msg.goal.trajectory.points[movement_number].time_from_start = ros::Duration(duration);
}

void UR5::move_arm_planner(float* pos,int movement_number,float startTime,float endTime){
    float qq[10],vq[10],aq[10];
    float t0=startTime;
    float tf=endTime-startTime;
    float t=tf/10;
    const float ta=tf/10;
    float q0=0;
    float qf=0;
    float v0=0;
    float vf=0;
    float ai=0;
    float af=0;
    float a0,a1,a2,a3,a4,a5;

    Matrix<float, 6, 1> b[6];

    Matrix<float, 6, 1> a[6];
    Matrix<float, 6, 6> m;

    for (int i=0;i < 6; i++){
        q0=get_state().position[i];

        qf=pos[i];
        b[i] << q0,v0,ai,qf,vf,af;
        m << 1, t0, t0*t0, t0*t0*t0,t0*t0*t0*t0,t0*t0*t0*t0,
            0, 1, 2*t0, 3*t0*t0,4*t0*t0*t0,5*t0*t0*t0*t0,
            0,0,2,6*t0,12*t0*t0,20*t0*t0*t0,
            1, tf, tf*tf, tf*tf*tf,tf*tf*tf*tf,tf*tf*tf*tf*tf,
            0, 1, 2*tf, 3*tf*tf,4*tf*tf*tf,5*tf*tf*tf*tf,
            0,0,2,6*tf,12*tf*tf,20*tf*tf*tf;
        a[i]=m.inverse()*b[i];
    }
    for (int i=movement_number; i < movement_number+10; i++){
            for(int j=0;j < 6; j++){
                a0=a[j].coeff(0,0);
                a1=a[j].coeff(1,0);
                a2=a[j].coeff(2,0);
                a3=a[j].coeff(3,0);
                a4=a[j].coeff(4,0);
                a5=a[j].coeff(5,0);
                qq[j]= a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t;
                vq[j]= a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t;
                aq[j]= 2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t;

            }
            move_arm(qq,startTime+t,i);
            move_arm_speed(vq,startTime+t,i);
            move_arm_acc(aq,startTime+t,i);
            t=t+ta;
     }

}

void UR5::ik_move(float x,float y,float z,float theta,int movement_number,float startTime,float endTime){
    string chain_start="ur_arm_base_link";
    string chain_end="ur_arm_wrist_3_link";
    string urdf_param="/robot_description";
    double timeout=0.005;
    double eps = 1e-5;
    TRAC_IK::SolveType type=TRAC_IK::Distance;

    Matrix3f m;
    m << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;
    Matrix3f mx;
    mx << 1, 0 ,0,
    0, cos(theta), -sin(theta),
    0, sin(theta), cos(theta);
    Matrix3f R;
    R= m*m*mx;

    //cout << mx << endl;
    //cout << R << endl;

    KDL::Vector vector(x,y,z);
    //KDL::Vector rx(1,0,0);
    //KDL::Vector ry(0,1,0);
    //KDL::Vector rz(0,0,1);
    KDL::Vector rx;
    KDL::Vector ry;
    KDL::Vector rz;
    rx(0)=R(0,0);
    rx(1)=R(0,1);
    rx(2)=R(0,2);
    ry(0)=R(1,0);
    ry(1)=R(1,1);
    ry(2)=R(1,2);
    rz(0)=R(2,0);
    rz(1)=R(2,1);
    rz(2)=R(2,2);

    KDL::Rotation rotation(rx,ry,rz);

    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps,type);
    KDL::Chain chain;
    KDL::JntArray ll, ul;
    bool valid = tracik_solver.getKDLChain(chain);
    float pos[9];

    if (!valid){
        ROS_ERROR("No valid chain found");
        return;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid) {
        ROS_ERROR("There were no valid joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    KDL::JntArray nominal(chain.getNrOfJoints());
    KDL::JntArray result;

    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = get_state().position[j];
    }

    KDL::Frame end_effector_pose(rotation,vector);
    int rc;

    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    if (rc >=0){

        for (int i=0;i<6;i++){
            pos[i]=result(i);
        }
        //pos[5]=pos[5]+1.5708;
        move_arm_planner(pos,movement_number,startTime,endTime);

        }else{
            ROS_ERROR("IK solution not found");
            exit (EXIT_FAILURE);
            return;
        }
}


bool UR5::ik_move(float x,float y,float z,float thetax,float thetay,float thetaz,int movement_number,float startTime,float endTime){
    string chain_start="ur_arm_base_link";
    string chain_end="ur_arm_wrist_3_link";
    string urdf_param="/robot_description";
    double timeout=0.005;
    double eps = 1e-5;
    TRAC_IK::SolveType type=TRAC_IK::Distance;
    /*
    Matrix3f m;
    m << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;
    */
    Matrix3f mx;
    mx << 1, 0 ,0,
    0, cos(thetax), -sin(thetax),
    0, sin(thetax), cos(thetax);

    Matrix3f my;
    my << cos(thetay), 0 , sin(thetay),
    0, 1, 0,
    -sin(thetay), 0, cos(thetay);

    Matrix3f mz;
    mz << cos(thetaz), -sin(thetaz), 0,
     sin(thetaz), cos(thetaz), 0,
    0 , 0, 1;

    Matrix3f R;


    R= mz*my*mx;

    //cout << "mx: " << endl <<mx << endl;
    //cout << "my: " << endl <<my << endl;
    //cout << "mz: " << endl <<mz << endl;
    //cout << R << endl;

    KDL::Vector vector(x,y,z);
    //KDL::Vector rx(1,0,0);
    //KDL::Vector ry(0,1,0);
    //KDL::Vector rz(0,0,1);
    KDL::Vector rx;
    KDL::Vector ry;
    KDL::Vector rz;
    rx(0)=R(0,0);
    rx(1)=R(0,1);
    rx(2)=R(0,2);
    ry(0)=R(1,0);
    ry(1)=R(1,1);
    ry(2)=R(1,2);
    rz(0)=R(2,0);
    rz(1)=R(2,1);
    rz(2)=R(2,2);

    KDL::Rotation rotation(rx,ry,rz);

    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps,type);
    KDL::Chain chain;
    KDL::JntArray ll, ul;
    bool valid = tracik_solver.getKDLChain(chain);
    float pos[9];

    if (!valid){
        ROS_ERROR("No valid chain found");
        return false;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid) {
        ROS_ERROR("There were no valid joint limits found");
        return false;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    KDL::JntArray nominal(chain.getNrOfJoints());
    KDL::JntArray result;

    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = get_state().position[j];
    }

    KDL::Frame end_effector_pose(rotation,vector);
    int rc;

    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    if (rc >=0){

        for (int i=0;i<6;i++){
            pos[i]=result(i);
        }
        //pos[5]=pos[5]+1.5708;
        move_arm_planner(pos,movement_number,startTime,endTime);
        return true;

        }else{
            ROS_ERROR("IK solution not found");
            //exit (EXIT_FAILURE);
            return false;
        }
}

void UR5::ik_move2(float x,float y,float z,float theta,int movement_number,float startTime,float endTime){
    string chain_start="ur_arm_base_link";
    string chain_end="ur_arm_wrist_3_link";
    string urdf_param="/robot_description";
    double timeout=0.005;
    double eps = 1e-5;
    TRAC_IK::SolveType type=TRAC_IK::Distance;

    Matrix3f m;
    m << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;
    Matrix3f mx;
    mx << 1, 0 ,0,
    0, cos(theta), -sin(theta),
    0, sin(theta), cos(theta);
    Matrix3f R;
    R= m*m*mx;
    //cout << R << endl;

    KDL::Vector vector(x,y,z);
    //KDL::Vector rx(1,0,0);
    //KDL::Vector ry(0,1,0);
    //KDL::Vector rz(0,0,1);
    KDL::Vector rx;
    KDL::Vector ry;
    KDL::Vector rz;
    rx(0)=R(0,0);
    rx(1)=R(0,1);
    rx(2)=R(0,2);
    ry(0)=R(1,0);
    ry(1)=R(1,1);
    ry(2)=R(1,2);
    rz(0)=R(2,0);
    rz(1)=R(2,1);
    rz(2)=R(2,2);

    KDL::Rotation rotation(rx,ry,rz);

    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps,type);
    KDL::Chain chain;
    KDL::JntArray ll, ul;
    bool valid = tracik_solver.getKDLChain(chain);
    float pos[9];

    if (!valid){
        ROS_ERROR("No valid chain found");
        return;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid) {
        ROS_ERROR("There were no valid joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    KDL::JntArray nominal(chain.getNrOfJoints());
    KDL::JntArray result;

    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = get_state().position[j];
    }

    KDL::Frame end_effector_pose(rotation,vector);
    int rc;

    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    if (rc >=0){

        for (int i=0;i<6;i++){
            pos[i]=result(i);
        }
        pos[5]=pos[5]-1.5708;
        move_arm_planner(pos,movement_number,startTime,endTime);

        }else{
            ROS_ERROR("IK solution not found");
            exit (EXIT_FAILURE);
            return;
        }
}


geometry_msgs::PoseStamped UR5::get_end_position(){
    string chain_start="ur_arm_base_link";
    string chain_end="ur_arm_wrist_3_link";
    string urdf_param="/robot_description";
    double timeout=0.005;
    double eps = 1e-5;
    TRAC_IK::SolveType type=TRAC_IK::Distance;
    KDL::Chain chain;
    geometry_msgs::PoseStamped p;


    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps,type);
    bool valid = tracik_solver.getKDLChain(chain);
     if (!valid){
        ROS_ERROR("No valid chain found");
        return p;
    }
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame end_effector_pose;
    KDL::JntArray result;
    result.resize(6);
    for (int i=0; i < 6; i++){
        result(i)=get_state().position[i];
    }

    fk_solver.JntToCart(result,end_effector_pose);

    p.pose.position.x= end_effector_pose(0,3);
    p.pose.position.y= end_effector_pose(1,3);
    p.pose.position.z= end_effector_pose(2,3);

    /*
    double m00 = end_effector_pose(0,0);
    double m02 = end_effector_pose(0,2);
    double m10 = end_effector_pose(1,0);
    double m11 = end_effector_pose(1,1);
    double m12 = end_effector_pose(1,2);
    double m20 = end_effector_pose(2,0);
    double m22 = end_effector_pose(2,2);

    double bank, attitude, heading;

    if (m10 > 0.998) { // singularity at north pole
            bank = 0;
            attitude = M_PI/2;
            heading = atan2(m02,m22);
        }
        else if (m10 < -0.998) { // singularity at south pole
            bank = 0;
            attitude = -M_PI/2;
            heading = atan2(m02,m22);
        }
        else
        {
            bank = atan2(-m12,m11);
            attitude = asin(m10);
            heading = atan2(-m20,m00);
        }
    p.pose.orientation.y= bank;
    p.pose.orientation.z= attitude;
    p.pose.orientation.x= heading;
    */



    auto sy=sqrt(end_effector_pose(0,0)*end_effector_pose(0,0) + end_effector_pose(1,0)*end_effector_pose(1,0));

    if ((!sy) < 1e-6){
        p.pose.orientation.x=-atan2(end_effector_pose(2,1),end_effector_pose(2,2));
        p.pose.orientation.y=-atan2(-end_effector_pose(2,0),sy);
        p.pose.orientation.z=-atan2(end_effector_pose(1,0),end_effector_pose(0,0));
    }else{
        p.pose.orientation.x = atan2(-end_effector_pose(1,2), end_effector_pose(1,1));
        p.pose.orientation.y = atan2(-end_effector_pose(2,0), sy);
        p.pose.orientation.z = 0;
    }


    return p;
}

geometry_msgs::Point UR5::homotransform(geometry_msgs::Point point){
    auto pos=get_end_position();

    float oz=pos.pose.orientation.z;
    float ox=pos.pose.orientation.x;
    float thetax=-1.5708 - ox; //Rotação fixa da camera
    float thetay=0;
    float thetaz=-oz; // - oz do braço

    Matrix3f mx;
    mx << 1, 0 ,0,
    0, cos(thetax), -sin(thetax),
    0, sin(thetax), cos(thetax);

    Matrix3f my;
    my << cos(thetay), 0 , sin(thetay),
    0, 1, 0,
    -sin(thetay), 0, cos(thetay);

    Matrix3f mz;
    mz << cos(thetaz), -sin(thetaz), 0,
     sin(thetaz), cos(thetaz), 0,
    0 , 0, 1;

    Matrix3f R;

    R= mz*my*mx;

    Matrix4f trans;
    trans << R(0,0) , R(0,1),R(0,2), pos.pose.position.x,
    R(1,0),R(1,1),R(1,2),pos.pose.position.y,
    R(2,0),R(2,1),R(2,2),pos.pose.position.z,
    0,0,0,1;

    Vector4f campos;
    campos << point.x ,point.y , point.z, 1;

    Vector4f final_pos;
    final_pos= trans * campos;
    geometry_msgs::Point solution;
    solution.x= final_pos(0);
    solution.y= final_pos(1);
    solution.z= final_pos(2);
    return solution;

}

geometry_msgs::PoseStamped UR5::homotransform(geometry_msgs::Point point,float ax,float ay,float az){

    auto pos=get_end_position();

    float oz=pos.pose.orientation.z;
    float ox=pos.pose.orientation.x;
    float thetax=-1.5708 - ox; //Rotação fixa da camera
    float thetay=0;
    float thetaz=-oz; // - oz do braço



    //Matriz de rotação do braço
    Matrix3f mx;
    mx << 1, 0 ,0,
    0, cos(thetax), -sin(thetax),
    0, sin(thetax), cos(thetax);

    Matrix3f my;
    my << cos(thetay), 0 , sin(thetay),
    0, 1, 0,
    -sin(thetay), 0, cos(thetay);

    Matrix3f mz;
    mz << cos(thetaz), -sin(thetaz), 0,
     sin(thetaz), cos(thetaz), 0,
    0 , 0, 1;

    Matrix3f R;

    R= mz*my*mx;

    //Matriz de rotação do objeto com relaçao a camera
    Matrix3f max;
    max << 1, 0 ,0,
    0, cos(ax), -sin(ax),
    0, sin(ax), cos(ax);

    Matrix3f may;
    may << cos(ay), 0 , sin(ay),
    0, 1, 0,
    -sin(ay), 0, cos(ay);

    Matrix3f maz;
    maz << cos(az), -sin(az), 0,
     sin(az), cos(az), 0,
    0 , 0, 1;

    Matrix3f Ra;

    Ra= maz*may*max;


    //Matrizes finais
    Matrix4f trans;
    trans << R(0,0) , R(0,1),R(0,2), pos.pose.position.x,
    R(1,0),R(1,1),R(1,2),pos.pose.position.y,
    R(2,0),R(2,1),R(2,2),pos.pose.position.z,
    0,0,0,1;

    Matrix4f campos;
    campos << Ra(0,0) , Ra(0,1),Ra(0,2), point.x,
            Ra(1,0),Ra(1,1),Ra(1,2),point.y,
            Ra(2,0),Ra(2,1),Ra(2,2),point.z,
            0,0,0,1;

    Matrix4f final_pos;
    final_pos= trans * campos;

    //Solução final
    geometry_msgs::PoseStamped solution;
    solution.pose.position.x= final_pos(0,3);
    solution.pose.position.y= final_pos(1,3);
    solution.pose.position.z= final_pos(2,3);
    
    auto sy=sqrt(final_pos(0,0)*final_pos(0,0) + final_pos(1,0)*final_pos(1,0));



    bool singular = sy < 1e-6;

    if (!singular){
        solution.pose.orientation.x=-atan2(final_pos(2,1),final_pos(2,2));
        solution.pose.orientation.y=-atan2(-final_pos(2,0),sy);
        solution.pose.orientation.z=-atan2(final_pos(1,0),final_pos(0,0));
    }else{
        solution.pose.orientation.x = atan2(-final_pos(1,2), final_pos(1,1));
        solution.pose.orientation.y = atan2(-final_pos(2,0), sy);
        solution.pose.orientation.z = 0;
    }
    

    /*
    if (final_pos(2,0) < 1){
                        if (final_pos(2,0) > -1){
                            solution.pose.orientation.z = atan2(final_pos(1,0), final_pos(0,0));
                            solution.pose.orientation.y = asin(-final_pos(2,0));
                            solution.pose.orientation.x = atan2(final_pos(2,1), final_pos(2,2));
                        }else{
                            solution.pose.orientation.z = -atan2(-final_pos(1,2), final_pos(1,1));
                            solution.pose.orientation.y = M_PI/2;
                            solution.pose.orientation.x = 0;
                        }
                    }else{
                        solution.pose.orientation.z = atan2(-final_pos(1,2), final_pos(1,1));
                        solution.pose.orientation.y = -M_PI/2;
                        solution.pose.orientation.x = 0;
                    }

    */
    return solution;

}

