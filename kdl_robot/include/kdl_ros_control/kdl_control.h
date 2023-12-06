#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);

    /*Added function*/
    //Eigen::Matrix<double,3,3> shCntr(Eigen::Vector3d lin_acc);
    Eigen::VectorXd idCntr(KDL::Frame &_desPos,KDL::Twist &_desVel, KDL::Twist &_desAcc, 
        double _Kpp, double _Kpo, double _Kdp, double _Kdo, Eigen::Matrix<double,3,1> e_o, Eigen::Matrix<double,3,1> e_o_w);

private:

    KDLRobot* robot_;

};

#endif
