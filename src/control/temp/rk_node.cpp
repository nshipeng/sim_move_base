/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-10 17:47:33
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-12 11:05:28
 */

#include "ros/ros.h"
#include "control/rk4.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


using namespace Eigen;


//for diff dirve model . we have x_dot = Ax + Bu. A is zero matrix. 
void model(const Ref<VectorXd> x0, const Ref<VectorXd> u,  Ref<VectorXd> x_out)
{
    Eigen::Matrix<double,3,3> A; A.setZero();
    //A(0,0) = 2.0; A(1,1) = 2.0; A(2,2) = 2.0;
    Eigen::Matrix<double,3,2> B; 
    B << 1.0,    0.0,
         1.0,    0.0,
         0,      0.0;
    x_out = A * x0 + B*u;
}


int main(int argc, char** argv){
    ROS_INFO("HELLO RK4");

    ros::init(argc, argv, "rk_node");
    ros::NodeHandle nh;

    double dt = 0.1;
    double t = 2;
    int n  = int(t/dt);

    RK4 rk4(dt);
    rk4.registerODE(&model);


    VectorXd x0; x0.resize(3,1);
    x0<<0.0, 0.0, 0.0;

    MatrixXd u; u.resize(2,n);
    for(int i = 0;i<n;i++){
        u.col(i)<<1,0;
        

    }

    MatrixXd res = rk4.solve(x0,u,t);
    std::cout<<res<<std::endl;

    return 0;
}
