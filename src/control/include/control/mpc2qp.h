/*
 * @Description: convert a MPC model to a QP model. 
 * @Autor: 
 * @Date: 2022-01-10 15:16:42
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-02-11 09:54:32
 */

#pragma once
/// \brief Utilities Library for Quadratic Programming

#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>
#include "OsqpEigen/OsqpEigen.h"


using Eigen::DiagonalMatrix;
using Eigen::Dynamic;
using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::SparseMatrix;
using Eigen::Vector2d;
using Eigen::VectorXd;


void print_hello(){
  printf("hello\n");
}

//https://robotology.github.io/osqp-eigen/md_pages_mpc.html
template <int Q_rows, int R_rows>
void castMPCToQPHessian(const DiagonalMatrix<double, Q_rows> &Q,
                        const DiagonalMatrix<double, R_rows> &R, 
                        int mpcWindow,
                        SparseMatrix<double> &hessianMatrix) {

  // Return size of diagonal matrices: size = row * col so we use sqrt.
  const int Q_size = std::sqrt(Q.size());
  const int R_size = std::sqrt(R.size());

  //the size of hessian matrix.
  int n = Q_size * (mpcWindow + 1) + R_size * mpcWindow;
  hessianMatrix.resize(n,n);

  // populate hessian matrix
  for (auto i = 0; i < n; i++) {
    if (i < Q_size * (mpcWindow + 1)) {
      int posQ = i % Q_size;
      float value = Q.diagonal()[posQ];
      if (value != 0) hessianMatrix.insert(i, i) = value;
    } else {
      int posR = i % R_size;
      float value = R.diagonal()[posR];
      if (value != 0) hessianMatrix.insert(i, i) = value;
    }
  }
}


template <int Q_rows, int xRef_rows, int xRef_cols>
void castMPCToQPGradient(
    const Eigen::DiagonalMatrix<double, Q_rows> &Q, 
    const Eigen::Matrix<double, xRef_rows, xRef_cols> &xRef, 
    int mpcWindow,
    int u_size,
    Eigen::VectorXd &gradient) {

  // Return size of diagonal matrices: size = row * col so we use sqrt.
  const int Q_size = std::sqrt(Q.size());

  Eigen::Matrix<double, xRef_rows, xRef_cols> Qx_ref;
  Qx_ref = Q * (-xRef);

  // populate the gradient vector
  gradient = Eigen::VectorXd::Zero(Q_size * (mpcWindow + 1) +  u_size* mpcWindow, 1);
  for (auto i = 0; i < Q_size * (mpcWindow + 1); i++) {
    int posQ = i % Q_size;
    float value = Qx_ref(posQ, 0);
    gradient(i, 0) = value;
  }
}



template <int dyn_rows, int dyn_cols, int ctrl_rows, int ctrl_cols>
void castMPCToQPConstraintMatrix(
    const Eigen::Matrix<double, dyn_rows, dyn_cols> &dynamicMatrix,
    const Eigen::Matrix<double, ctrl_rows, ctrl_cols> &controlMatrix,
    int mpcWindow, 
    Eigen::SparseMatrix<double> &constraintMatrix) {
      
  // Get Rows and Cols from dynamicMatrix and controlMatrix.
  const int dynamicMatrix_rows = dynamicMatrix.rows();
  const int dynamicMatrix_cols = dynamicMatrix.cols();
  const int controlMatrix_rows = controlMatrix.rows();
  const int controlMatrix_cols = controlMatrix.cols();

  constraintMatrix.resize(dynamicMatrix_rows * (mpcWindow + 1) +dynamicMatrix_cols * (mpcWindow + 1) + controlMatrix_cols * mpcWindow,
                          controlMatrix_rows * (mpcWindow + 1) + controlMatrix_cols * mpcWindow);

  // populate linear constraint matrix
  for (auto i = 0; i < dynamicMatrix_rows * (mpcWindow + 1); i++) {
    constraintMatrix.insert(i, i) = -1;
  }

  for (auto i = 0; i < mpcWindow; i++)
    for (int j = 0; j < dynamicMatrix_rows; j++)
      for (int k = 0; k < dynamicMatrix_rows; k++) {
        float value = dynamicMatrix(j, k);
        if (value != 0) {
          constraintMatrix.insert(dynamicMatrix_rows * (i + 1) + j,
                                  dynamicMatrix_rows * i + k) = value;
        }
      }

  for (auto i = 0; i < mpcWindow; i++)
    for (int j = 0; j < dynamicMatrix_rows; j++)
      for (int k = 0; k < controlMatrix_cols; k++) {
        float value = controlMatrix(j, k);
        if (value != 0) {
          constraintMatrix.insert(dynamicMatrix_rows * (i + 1) + j,
                                  controlMatrix_cols * i + k +
                                      dynamicMatrix_rows * (mpcWindow + 1)) =
              value;
        }
      }

  for (auto i = 0; i < dynamicMatrix_rows * (mpcWindow + 1) +
                           controlMatrix_cols * mpcWindow;
       i++) {
    constraintMatrix.insert(i + (mpcWindow + 1) * dynamicMatrix_rows, i) = 1;
  }
}



template <int x_rows, int u_rows, int x0_rows>
void castMPCToQPConstraintVectors(const Eigen::Matrix<double, x_rows, 1> &xMax,
                                  const Eigen::Matrix<double, x_rows, 1> &xMin,
                                  const Eigen::Matrix<double, u_rows, 1> &uMax,
                                  const Eigen::Matrix<double, u_rows, 1> &uMin,
                                  const Eigen::Matrix<double, x0_rows, 1> &x0,
                                  const Eigen::Matrix<double, x0_rows, 1> &O,
                                  int mpcWindow, 
                                  int u_size,
                                  Eigen::VectorXd &lowerBound,
                                  Eigen::VectorXd &upperBound) {
                                    
  std::cout<<"........."<<std::endl;
  // Get argument vector lenghts
  const int xMax_size = xMax.size();
  const int xMin_size = xMin.size();
  const int uMax_size = uMax.size();
  const int uMin_size = uMin.size();
  const int x0_size = x0.size();
 
  std::cout<<"........."<<std::endl;

  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lowerInequality =
      Eigen::MatrixXd::Zero(x0_size * (mpcWindow + 1) + u_size * mpcWindow, 1);
  Eigen::VectorXd upperInequality =
      Eigen::MatrixXd::Zero(x0_size * (mpcWindow + 1) + u_size * mpcWindow, 1);
  for (auto i = 0; i < mpcWindow + 1; i++) {
    lowerInequality.block(xMin_size * i, 0, xMin_size, 1) = xMin;
    upperInequality.block(xMax_size * i, 0, xMax_size, 1) = xMax;
  }
  for (auto i = 0; i < mpcWindow; i++) {
    lowerInequality.block(uMin_size * i + x0_size * (mpcWindow + 1), 0,
                          uMin_size, 1) = uMin;
    upperInequality.block(uMax_size * i + x0_size * (mpcWindow + 1), 0,
                          uMax_size, 1) = uMax;
  }
  std::cout<<"........."<<std::endl;

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(x0_size * (mpcWindow + 1), 1);
  Eigen::VectorXd upperEquality;

  lowerEquality.block(0, 0, x0_size, 1) = -x0;
  for(int i = 1;i<=mpcWindow;++i){
    lowerEquality.block(x0_size * i, 0, x0_size,1) = -O;
  }


                                               
  upperEquality = lowerEquality;
  lowerEquality = lowerEquality;

  // merge inequality and equality vectors
  lowerBound = Eigen::MatrixXd::Zero(
      2 * x0_size * (mpcWindow + 1) + uMax_size * mpcWindow, 1);
  lowerBound << lowerEquality, lowerInequality;

  upperBound = Eigen::MatrixXd::Zero(
      2 * x0_size * (mpcWindow + 1) + uMax_size * mpcWindow, 1);
  upperBound << upperEquality, upperInequality;
}


template <int rows>
void updateConstraintVectors(const Eigen::Matrix<double, rows, 1> &x0,
                             Eigen::VectorXd &lowerBound,
                             Eigen::VectorXd &upperBound) {
  // Get Size
  const int x0_size = x0.size();
  lowerBound.block(0, 0, x0_size, 1) = -x0;
  upperBound.block(0, 0, x0_size, 1) = -x0;
}


template <int x_rows, int xRef_rows>
double getErrorNorm(const Eigen::Matrix<double, x_rows, 1> &x,
                    const Eigen::Matrix<double, xRef_rows, 1> &xRef) {
  // Get Size
  const auto x_size = x.size();
  // evaluate the error
  Eigen::Matrix<double, x_size, 1> error = x - xRef;

  // return the norm
  return error.norm();
}

