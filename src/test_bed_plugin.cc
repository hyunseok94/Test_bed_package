#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace gazebo
{

    class TEST_BED : public ModelPlugin
    {
        physics::LinkPtr BASE_BED;
        physics::LinkPtr PITCH_BED;
        physics::LinkPtr ROLL_BED;

        physics::JointPtr PITCH_JOINT;
        physics::JointPtr ROLL_JOINT;

        physics::ModelPtr model;

        VectorXd tar_angle = VectorXd::Zero(2);
        VectorXd goal_angle = VectorXd::Zero(2);
        VectorXd tar_angle_dot = VectorXd::Zero(2);

        VectorXd angle = VectorXd::Zero(2);
        VectorXd init_angle = VectorXd::Zero(2);
        VectorXd pre_angle = VectorXd::Zero(2);
        VectorXd angle_dot = VectorXd::Zero(2);
        VectorXd angle_err = VectorXd::Zero(2);
        VectorXd angle_dot_err = VectorXd::Zero(2);
        VectorXd torque = VectorXd::Zero(2);
        VectorXd kp = VectorXd::Zero(2);
        VectorXd kd = VectorXd::Zero(2);

        //setting for getting <dt>(=derivative time)
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;
        double Duration=4.0;

        bool tar_flag=false;

        unsigned int cnt=0;
        int Step=0;
        ros::NodeHandle n;
        ros::Publisher P_angle_err;
        ros::Subscriber S_target;

        std_msgs::Float64MultiArray m_angle_err;


        //For model load
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void Callback(const std_msgs::Int16MultiArray &msg);


    };
    GZ_REGISTER_MODEL_PLUGIN(TEST_BED);
}

void gazebo::TEST_BED::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    // model = link + joint +sensor
    this->model = _model;

    //LINK DEFINITION
    this->BASE_BED = this->model->GetLink("BASE_BED");
    this->PITCH_BED = this->model->GetLink("PITCH_BED");
    this->ROLL_BED = this->model->GetLink("ROLL_BED");

    //JOINT DEFINITION
    this->PITCH_JOINT = this->model->GetJoint("PITCH_JOINT");
    this->ROLL_JOINT = this->model->GetJoint("ROLL_JOINT");

    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TEST_BED::UpdateAlgorithm, this));

    //ROS Communication setting
    P_angle_err = n.advertise<std_msgs::Float64MultiArray>("angle_err", 1);
    m_angle_err.data.resize(2);

    S_target = n.subscribe("target_angle", 1, &gazebo::TEST_BED::Callback, this);

    kp<<20000,20000;
    kd<<300,300;
    tar_angle<<0,0;
    tar_angle_dot<<0,0;

}

void gazebo::TEST_BED::UpdateAlgorithm()
{
    //* Calculate time
    common::Time current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    //Get angle & angle_dot
    angle[0]=this->PITCH_JOINT->GetAngle(1).Radian();
    angle[1]=this->ROLL_JOINT->GetAngle(1).Radian();
    for(int i=0; i<2;++i){
     angle_dot[i]=(angle[i]-pre_angle[i])/dt;
     pre_angle[i]=angle[i];
     std::cout<<"A"<<std::endl;
   }

    if(tar_flag==true){
        if(cnt==0){
            for(int i=0; i<2; ++i){
             tar_angle[i]=0;
             init_angle[i]=0;
            }
            std::cout<<"A"<<std::endl;
            cnt++;
          }
        else if(cnt <= (unsigned int) (Duration / dt)){
            for(int i=0; i<2; ++i){
             tar_angle[i]=init_angle[i]+(goal_angle[i]-init_angle[i])/2.0 * (1-cos(PI/Duration*(double)(cnt)*dt));
            }
            cnt++;
            std::cout<<"B"<<std::endl;
        }
        else{
          
          //cnt=0;
          //tar_flag=false;
          for(int i=0; i<2; ++i){
             init_angle[i]=tar_angle[i];
          }
            Step++;
         if(Step==1){
           goal_angle[0]=10*D2R;
           goal_angle[1]=0*D2R;
           std::cout<<"C"<<std::endl;
         }
         else if(Step==2){
           goal_angle[0]=-10*D2R;
           goal_angle[1]=0*D2R;
           std::cout<<"D"<<std::endl;
         }
         else if(Step==3){
           goal_angle[0]=0*D2R;
           goal_angle[1]=10*D2R;
           std::cout<<"E"<<std::endl;
         }
         else if(Step==4){
           goal_angle[0]=0*D2R;
           goal_angle[1]=-10*D2R;
           std::cout<<"F"<<std::endl;
         }
         else if(Step==5){
           goal_angle[0]=-10*D2R;
           goal_angle[1]=-10*D2R;
           std::cout<<"G"<<std::endl;
         }
         else if(Step==6){
           goal_angle[0]=0*D2R;
           goal_angle[1]=0*D2R;
           std::cout<<"Ha"<<std::endl;
         }
         else{
           tar_flag=false;
           cnt=0;
           Step=0;
         }
            cnt=1;
        }
    }

    //Calculate Angle error
    angle_err[0] = tar_angle[0]-angle[0];
    angle_err[1] = tar_angle[1]-angle[1];
    angle_dot_err[0]=tar_angle_dot[0]-angle_dot[0];
    angle_dot_err[1]=tar_angle_dot[1]-angle_dot[1];

    //Calculate Torque
     torque[0]=kp[0]*angle_err[0]+kd[0]*angle_dot_err[0];
     torque[1]=kp[1]*angle_err[1]+kd[1]*angle_dot_err[1];

    //* Apply torque to joint
    this->PITCH_JOINT->SetForce(1, torque[0]);
    this->ROLL_JOINT->SetForce(1, torque[1]);

    //*setting for getting dt
    this->last_update_time = current_time;

    m_angle_err.data[0] = angle_err[0]*R2D;
    m_angle_err.data[1] = angle_err[1]*R2D;

    P_angle_err.publish(m_angle_err);
}

void gazebo::TEST_BED::Callback(const std_msgs::Int16MultiArray &msg)
{
    goal_angle[0]=msg.data[0]*D2R;
    goal_angle[1]=msg.data[1]*D2R;
    tar_flag=true;
}

