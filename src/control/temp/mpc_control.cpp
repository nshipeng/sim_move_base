/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-11 17:52:59
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-11 18:26:53
 */

#include "control/mpc_control.h"

mpc_control::mpc_control(){
    
}

void mpc_control::setA(){
    A.setIdentity();
}

void mpc_control::setB(Vector3d x0){
    
    double theta = x0(2);
    B << std::cos(theta),    0,
         std::sin(theta),    0,
         0,                  1;
}

void mpc_control::setInequalityConstraints(){
     uMin << -0.1, -1;
     uMax << 0.1, 1;
     xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -3.14;
     xMax << OsqpEigen::INFTY,  OsqpEigen::INFTY,  3.14;
 }

void mpc_control::setWeightMatrices(){
     Q.diagonal() << 10,10,10;
     R.diagonal() << 0.1, 0.1;
 }

size_t mpc_control::slove(Vector2d& u){
      // cast the MPC problem as QP problem
    // castMPCToQPHessian<3, 2>(Q, R, mpcWindow, hessian);
    // castMPCToQPGradient<3, 3, 20>(Q, xRef, mpcWindow, gradient);
    // castMPCToQPConstraintMatrix<3, 3, 2, 2>(A, B, mpcWindow, linearMatrix);
    // castMPCToQPConstraintVectors<3, 2, 3>(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // solver.settings()->setWarmStart(true);

    // // set the initial data of the QP solver
    // solver.data()->setNumberOfVariables(3 * (mpcWindow + 1) + 2 * mpcWindow);
    // solver.data()->setNumberOfConstraints(2 * 3 * (mpcWindow + 1) +2 * mpcWindow);
    // if (!solver.data()->setHessianMatrix(hessian)) return 1;
    // if (!solver.data()->setGradient(gradient)) return 1;
    // if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    // if (!solver.data()->setLowerBound(lowerBound)) return 1;
    // if (!solver.data()->setUpperBound(upperBound)) return 1;

    // // instantiate the solver
    // if (!solver.initSolver()) return 1;

    // // controller input and QPSolution vector
    // Eigen::Vector2d ctr;



    // return 0;
     
 }


 




