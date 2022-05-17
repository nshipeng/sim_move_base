/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-11 18:37:12
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-02-11 10:22:16
 */
#ifndef _MPC_H_
#define _MPC_H_

#include <eigen3/Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "control/utilities.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Ref;


//since don't use template yet. the dim of u and others ytility functios dim should change prop.
static size_t MPC(Eigen::Matrix<double, 3, 1>x0, Eigen::Matrix<double, 3, 10> xRef, Vector3d& u){  //if use huanyu u is 3 dim.

     Eigen::Matrix<double, 3, 3> A; A.setIdentity();
     Eigen::Matrix<double, 3, 3> B; 
     double theta = x0(2,0);
     double dt = 0.1;
    B << dt* std::cos(theta),    -dt* std::sin(theta),   0,
         dt* std::sin(theta),     dt* std::cos(theta),   0,
         0,                       0,                     dt*1;

    Eigen::Matrix<double, 3, 1> O = -A *x0;

    Eigen::Matrix<double, 3, 1> xMax;
    Eigen::Matrix<double, 3, 1> xMin;
    Eigen::Matrix<double, 3, 1> uMax;
    Eigen::Matrix<double, 3, 1> uMin;
    xMin <<  -OsqpEigen::INFTY, -OsqpEigen::INFTY,-OsqpEigen::INFTY;
    xMax <<  OsqpEigen::INFTY,   OsqpEigen::INFTY, OsqpEigen::INFTY;
    uMin << -0.2,  -0.2,    -0.5;
    uMax <<  0.2,   0.2,     0.5;


    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 3> Q;
    Eigen::DiagonalMatrix<double, 3> R;
    Q.diagonal() << 100,  100, 1;
    R.diagonal() << 1,     1,  1;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    int mpcWindow = 10;


    //cast the MPC problem as QP problem
    castMPCToQPHessian<3, 3>(Q, R, mpcWindow, hessian);
    castMPCToQPGradient<3, 3, 10>(Q, xRef, mpcWindow,gradient);
    castMPCToQPConstraintMatrix<3, 3,3,3>(A, B, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors<3, 3, 3>(xMax, xMin, uMax, uMin, x0,O, mpcWindow, lowerBound, upperBound);
    //  std::cout<<"Hessian \n"<<hessian<<std::endl;
    //  std::cout<<"gradient \n"<<gradient<<std::endl;
    //  std::cout<<"linearMatrix \n"<<linearMatrix<<std::endl;
    //  std::cout<<"lowerBound \n"<<lowerBound<<std::endl;


     OsqpEigen::Solver solver;

    // settings
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(3 * (mpcWindow + 1) + 3 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 3 * (mpcWindow + 1) +3 * mpcWindow);

    if (!solver.data()->setHessianMatrix(hessian)) return 1;
    if (!solver.data()->setGradient(gradient)) return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if (!solver.data()->setLowerBound(lowerBound)) return 1;
    if (!solver.data()->setUpperBound(upperBound)) return 1;

    // instantiate the solver
    if (!solver.initSolver()) return 1;

    Eigen::Vector3d ctr;
    Eigen::VectorXd QPSolution;
    if (!solver.solve()) return 1;

    // get the controller input
    QPSolution = solver.getSolution();
    ctr = QPSolution.block(3 * (mpcWindow + 1), 0, 3, 1);
    //std::cout<<"QPSolution  \n"<<QPSolution<<std::endl;
    u = ctr;
    return 0;

}

static size_t DIFF_MPC(Eigen::Matrix<double, 3, 1>x0, Eigen::Matrix<double, 3, 10> xRef, Vector2d& u0, Vector2d& u){  
     Eigen::Matrix<double, 3, 3> A;   
     Eigen::Matrix<double, 3, 2> B;
     double dt = 0.1;
     double theta = x0(2,0);

     A<<1,   0,   -u0(0)* sin(theta) * dt,
        0,   1,    u0(0)* cos(theta )* dt,
        0,   0,     1;
 
     B<< dt* std::cos(theta),    0,
         dt* std::sin(theta),    0,
         0,                      dt*1;


    //Eigen::Matrix<double, 3, 1> O = -A *x0 * dt;
    Eigen::Matrix<double, 3, 1> O ;
    O.col(0)<<0.0,0.0,0.0;

    Eigen::Matrix<double, 3, 1> xMax;
    Eigen::Matrix<double, 3, 1> xMin;
    Eigen::Matrix<double, 2, 1> uMax;
    Eigen::Matrix<double, 2, 1> uMin;
    xMin <<  -OsqpEigen::INFTY, -OsqpEigen::INFTY,-OsqpEigen::INFTY;
    xMax <<  OsqpEigen::INFTY,   OsqpEigen::INFTY, OsqpEigen::INFTY;
    uMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    uMax <<  OsqpEigen::INFTY,  OsqpEigen::INFTY;
    //uMin << -0.2, -0.3;
    //uMax <<  0.2,  0.3;


    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 3> Q;
    Eigen::DiagonalMatrix<double, 2> R;
    Q.diagonal() << 100,  100, 0.1;
    R.diagonal() << 0.1, 20;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    int mpcWindow = 10;
    int u_size = 2;

    //cast the MPC problem as QP problem
    castMPCToQPHessian<3, 2>(Q, R, mpcWindow, hessian);
    castMPCToQPGradient<3, 3, 10>(Q, xRef, mpcWindow,gradient);
    castMPCToQPConstraintMatrix<3,3,3,2>(A, B, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors<3, 2, 3>(xMax, xMin, uMax, uMin, x0, O, mpcWindow,lowerBound, upperBound);
//     std::cout<<"Hessian \n"<<hessian<<std::endl;
//     std::cout<<"gradient \n"<<gradient<<std::endl;
//     std::cout<<"linearMatrix \n"<<linearMatrix<<std::endl;
//     std::cout<<"lowerBound \n"<<lowerBound<<std::endl;


     OsqpEigen::Solver solver;

    // settings
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(3 * (mpcWindow + 1) + 2 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 3 * (mpcWindow + 1) +2 * mpcWindow);

    if (!solver.data()->setHessianMatrix(hessian)) return 1;
    if (!solver.data()->setGradient(gradient)) return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if (!solver.data()->setLowerBound(lowerBound)) return 1;
    if (!solver.data()->setUpperBound(upperBound)) return 1;

    // instantiate the solver
    if (!solver.initSolver()) return 1;

    Eigen::Vector2d ctr;
    Eigen::VectorXd QPSolution;
    if (!solver.solve()) return 1;

    // get the controller input
    QPSolution = solver.getSolution();
    ctr = QPSolution.block(3 * (mpcWindow + 1), 0, 2, 1);
    //std::cout<<"QPSolution  \n"<<QPSolution<<std::endl;
    u = ctr;
    return 0;


}



#endif