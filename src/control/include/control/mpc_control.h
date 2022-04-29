/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-11 17:44:27
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-11 18:24:34
 */
/*
 * @Description: mpc control for traj track.
 * @Autor: 
 * @Date: 2022-01-10 14:54:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-11 17:40:31
 */

#ifndef _MPC_CONTROL_H_
#define _MPC_CONTROL_H_

#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include "utilities.h"

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Vector3d;


class mpc_control
{
    public:
        mpc_control();
        ~mpc_control(){}

        void setA();
        void setB(Vector3d x0);
        void setInequalityConstraints();
        void setWeightMatrices();

        size_t slove(Vector2d& u);


    private:
        // set the preview window
        int mpcWindow = 20;

        // allocate the dynamics matrices
        Eigen::Matrix<double, 3, 3> A;
        Eigen::Matrix<double, 3, 2> B;

        // allocate the constraints vector
        Eigen::Matrix<double, 3, 1> xMax;
        Eigen::Matrix<double, 3, 1> xMin;
        Eigen::Matrix<double, 3, 1> uMax;
        Eigen::Matrix<double, 3, 1> uMin;

        // allocate the weight matrices
        Eigen::DiagonalMatrix<double, 3> Q;
        Eigen::DiagonalMatrix<double, 2> R;

        // allocate the initial and the reference state space
        Eigen::Matrix<double, 3, 1> x0;
        Eigen::Matrix<double, 3, 20> xRef;

        // allocate QP problem matrices and vectores
        Eigen::SparseMatrix<double> hessian;
        Eigen::VectorXd gradient;
        Eigen::SparseMatrix<double> linearMatrix;
        Eigen::VectorXd lowerBound;
        Eigen::VectorXd upperBound;

        OsqpEigen::Solver solver;

};
 


#endif