#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius){
  trajDuration_ = _trajDuration;
  trajInit_ = _trajInit;
  trajRadius_ = _trajRadius;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

void KDLPlanner::trapezoidal_vel(double time, double t_c, double& s, double& s_dot, double& s_ddot){

    double ddot_traj_c = -1.0/(std::pow(t_c,2)-trajDuration_*t_c);

    if(time <= t_c)
  {
    s =  0.5*ddot_traj_c*std::pow(time,2);
    s_dot = ddot_traj_c*time;
    s_ddot = ddot_traj_c;
  }
  else if(time <= trajDuration_-t_c) 
  {
    s = ddot_traj_c*t_c*(time-t_c/2);
    s_dot = ddot_traj_c*t_c;
    s_ddot = 0;
  }
  else
  {
    s = 1 - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    s_dot = ddot_traj_c*(trajDuration_-time);
    s_ddot = -ddot_traj_c;
  }
}

void KDLPlanner::cubic_polinomial(double time, double& s, double& s_dot, double& s_ddot){
  double a_0,a_1,a_2,a_3;
  //double trajEnd1_, trajInit1_;
  /*trajEnd1_=1;
  //trajInit1_=0;
  //Assuming initial velocity equal to 0 and final velocity equal to 0
  //and we also assuming (trajInit_ = trajectory initial point) and (trajEnd_  = trajectory final point)
  a_0=trajInit1_;
  a_1=0; 
  a_2=-3/2*((trajEnd1_-trajInit1_)/(std::pow(trajDuration_,3)-3/2*std::pow(trajDuration_,2)));
  a_3=(trajEnd1_-trajInit1_)/(std::pow(trajDuration_,3)-3/2*std::pow(trajDuration_,2));
  */ 
  a_0=0;
  a_1=0; 
  a_2=3/(std::pow(trajDuration_,2));
  a_3=-2/(std::pow(trajDuration_,3));


  s = a_3*std::pow(time,3)+a_2*std::pow(time,2)+a_1*time+a_0;
  s_dot = 3*a_3*std::pow(time,2)+2*a_2*time+a_1;
  s_ddot = 6*a_3*time+2*a_2;
}

trajectory_point KDLPlanner::compute_trajectory_circular(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;
  double pi = 3.14;

  double s, s_dot, s_ddot;
  double t_c = 0.5;
  //cubic_polinomial(time, s, s_dot, s_ddot);
  trapezoidal_vel(time, t_c, s, s_dot, s_ddot);
  double x, x_dot, x_ddot;
  double y, y_dot, y_ddot;
  double z, z_dot, z_ddot;

  x=trajInit_(0);
  y=trajInit_(1)-trajRadius_*cos(2*pi*s);
  z=trajInit_(2)-trajRadius_*sin(2*pi*s);

  x_dot=0;
  y_dot=trajRadius_*sin(2*pi*s)*2*pi*s_dot;
  z_dot=-trajRadius_*cos(2*pi*s)*2*pi*s_dot;

  x_ddot=0;
  y_ddot=trajRadius_*2*pi*(2*pi*std::pow(s_dot,2)*cos(2*pi*s)+sin(2*pi*s)*s_ddot);
  z_ddot=-trajRadius_*2*pi*(-2*pi*std::pow(s_dot,2)*sin(2*pi*s)+cos(2*pi*s)*s_ddot);

  Eigen::Vector3d pos(x,y,z);
  traj.pos=pos;
  Eigen::Vector3d vel(x_dot,y_dot,z_dot);
  traj.vel=vel;
  Eigen::Vector3d acc(x_ddot,y_ddot,z_ddot);
  traj.acc=acc;

  return traj;
}
trajectory_point KDLPlanner::compute_trajectory_linear(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  double s, s_dot, s_ddot;
  double t_c = 0.5;
  //cubic_polinomial(time, s, s_dot, s_ddot);
  trapezoidal_vel(time, t_c, s, s_dot, s_ddot);
  double x, x_dot, x_ddot;
  double y, y_dot, y_ddot;
  double z, z_dot, z_ddot;

  x=trajInit_(0);
  y=trajInit_(1);
  z=trajInit_(2)+s/7;

  x_dot=0;
  y_dot=0;
  z_dot=s_dot/7;

  x_ddot=0;
  y_ddot=0;
  z_ddot=s_ddot/7;


  Eigen::Vector3d pos(x,y,z);
  traj.pos=pos;
  Eigen::Vector3d vel(x_dot,y_dot,z_dot);
  traj.vel=vel;
  Eigen::Vector3d acc(x_ddot,y_ddot,z_ddot);
  traj.acc=acc;

  return traj;
}

trajectory_point KDLPlanner::compute_trajectory(double time, int traj_circular)
{
  trajectory_point traj;

  if(traj_circular == 0) {
    traj=compute_trajectory_linear(time);
  } else {
    traj=compute_trajectory_circular(time);
  }
  
  return traj;
}

/* OLD
trajectory_point KDLPlanner::compute_trajectory(double time)
{
   trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point 

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;
}*/
