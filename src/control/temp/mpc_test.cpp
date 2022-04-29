// /*
//  * @Description: 
//  * @Autor: 
//  * @Date: 2022-01-11 10:41:29
//  * @LastEditors: Please set LastEditors
//  * @LastEditTime: 2022-01-12 10:15:49
//  */


#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "control/mpc.h"

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;



int main(int argc, char** argv){
    ROS_INFO("HELLO MPC");

    ros::init(argc, argv, "mpc_test_node");
    ros::NodeHandle nh;

    // DiffDrive diff_drive;
    // Problem problem;

    int x_state = 3;
    int mpc_horizon = 10;
     
    Eigen::Matrix<double, 3, 10> ref_traj;
    for(int i = 0;i<10;i++){
        ref_traj.col(i)<<0.1,0.0,0.1;

    }

    Eigen::Matrix<double, 3, 1>x0;
    x0<<0.0,0.0,0.0;


    Vector2d u;u<<0.0,0.0;

    Mpc(x0,ref_traj,u);
    ROS_WARN_STREAM("control input is \n"<<u);



    return 0;
}
