/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-11 14:54:38
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-11 17:39:59
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;

class Problem
{

    public:
        Problem():Q_({10,10,10}),R_({1,1}){
            u_size_ = 2;
            x_size_ = 3;
            xmin_.resize(3); xmin_<<-10, -10, -10;
            xmax_.resize(3); xmax_<< 10,  10,  10;
            umin_.resize(2); umin_<<-0.15, -0.5;
            umax_.resize(2); umax_<< 0.15,  0.5;

            //set weight para
            qx = 10; qy = 10; q_theta = 10;
            rv = 1;  rw = 1;
            // Q_.diagonal()<<qx,qy,q_theta;
            // R_.diagonal()<<rv,rw;
        }
        ~Problem(){};

        static constexpr int q_rows = 3;
        static constexpr int r_rows = 2;
        static constexpr int xref_rows = 3;
        static constexpr int xref_cols = 20;
        static constexpr int a_rows = 3;
        static constexpr int a_cols = 3;
        static constexpr int b_rows = 3;
        static constexpr int b_cols =2;
        static constexpr int xmax_rows = 3;
        static constexpr int umax_rows = 2;
        static constexpr int x0_rows = 3;

        
    
        VectorXd xmin()const {return xmin_;}
        VectorXd xmax()const {return xmax_;}
        VectorXd umin()const {return umin_;}
        VectorXd umax()const {return umax_;}
        int u_size()const {return u_size_;}
        int x_size()const {return x_size_;}

        Eigen::DiagonalMatrix<double, 3>  Q()const {return Q_;}
        Eigen::DiagonalMatrix<double, 2> R()const {return R_;}

        //setQ()  setR().....

    private:
        VectorXd xmin_;
        VectorXd xmax_;
        VectorXd umin_;
        VectorXd umax_;
        VectorXd x0_;                            //init state
        // static constexpr int q_size = 3;
        // static constexpr int r_size = 2;
        int u_size_;                            //the size of control vector
        int x_size_;                            //the size of state vector

        const Eigen::DiagonalMatrix<double, q_rows> Q_;    //Q weight matrix
        double qx, qy, q_theta;
        const Eigen::DiagonalMatrix<double, r_rows> R_;    //R wight matrix.
        double rv,rw;
};


