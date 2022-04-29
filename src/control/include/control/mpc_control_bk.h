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
#include "mpc2qp.h"

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;

// Model
// Ref_Traj
// State
// Control
template <typename Model, typename Problem,typename Ref_Traj,typename State>
class mpc_control
{

    public:
        mpc_control(const Model& model, const int& mpc_window):m_model(model),mpc_horizon(mpc_window){
        }
        ~mpc_control(){}

        size_t setup(const Problem& problem, const Ref_Traj& ref_traj, const State& _x0);
        //size_t solve(const Model& model, const Ref_Traj& ref_traj, const State& _x0, Ref<VectorXd> u);


    private:
        Model m_model;
        int mpc_horizon;

        int max_slove_steps;      //max number of optimation steps before giving up.        
        double slove_threshold;  //cost threshold for stop optimization.

        /// OSQP Solver with Eigen C++ Wrapper.
        OsqpEigen::Solver m_solver;
        
};


template <typename Model, typename Problem,typename Ref_Traj,typename State>
size_t mpc_control<Model,Problem, Ref_Traj, State>::setup(const Problem& problem, const Ref_Traj& ref_traj, const State& _x0){

    // Grab model data.
    const auto& Q    =    problem.Q();
    const auto& R    =    problem.R();
    const auto& xMax =    problem.xmax();
    const auto& xMin =    problem.xmin();
    const auto& uMax =    problem.umax();
    const auto& uMin =    problem.umin();
    const auto& u_size =  problem.u_size();
    const auto& x_size =  problem.x_size();

    const auto& x0   =    _x0;
    const auto& xRef =    ref_traj;

    // Grab model data. 
    const auto a     =    m_model.A();
    const auto b     =    m_model.B(x0);



    


    // print_hello();

    // // cast the MPC model as QP model
    castMPCToQPHessian<problem.q_rows, problem.r_rows>(Q, R, mpc_horizon, m_model.hessian());
    castMPCToQPGradient<problem.q_rows, problem.xref_rows, problem.xref_cols>(Q, xRef, mpc_horizon, u_size, m_model.gradient());
    castMPCToQPConstraintMatrix<problem.a_rows , problem.a_cols, problem.b_rows, problem.b_cols>(a, b, mpc_horizon, m_model.linear_constraint_matrix());
    castMPCToQPConstraintVectors<problem.xmax_rows, problem.umax_rows,problem.x0_rows>(xMax, xMin, uMax, uMin, x0, mpc_horizon, u_size, m_model.lower_bound(),m_model.upper_bound());

    // // instantiate the solver
    // OsqpEigen::Solver solver;

    // // settings
    // solver.settings()->setVerbosity(false);
    // solver.settings()->setWarmStart(true);

    // // set the initial data of the QP solver
    // solver.data()->setNumberOfVariables(m_model.num_variables(mpc_horizon));
    // solver.data()->setNumberOfConstraints(m_model.num_constraints(mpc_horizon));
    // if (!solver.data()->setHessianMatrix(m_model.hessian())) return 1;
    // if (!solver.data()->setGradient(m_model.gradient())) return 1;
    // if (!solver.data()->setLinearConstraintsMatrix(
    //         m_model.linear_constraint_matrix()))
    //   return 1;
    // if (!solver.data()->setLowerBound(m_model.lower_bound())) return 1;
    // if (!solver.data()->setUpperBound(m_model.upper_bound())) return 1;

    // // instantiate the solver
    // if (!solver.initSolver()) return 1;

    // m_solver = solver;
    return 0;
}


// template <typename Model, typename Problem,typename Ref_Traj,typename State>
// size_t mpc_control<Model, Ref_Traj, State>::solve(Model& model, const Ref_Traj& ref_traj, const State& _x0, Ref<VectorXd> u){

//     // Grab updated model parameters
//     const auto& Q = model.Q;
//     const auto u_size =   model.u_size;
//     auto& x0   =  x0;
//     const auto& xRef = ref_traj;

//     // Grab solver
//     auto& solver = m_solver;

//     // Update Gradient based on new xRef
//     castMPCToQPGradient<Q.rows(), xRef.rows(), xRef.cols()>( Q, xRef, mpc_horizon,u_size, model.gradient());
//     if (!solver.updateGradient(model.gradient())) return 1;

//     // Update Constraint Vector based on new initial state
//     updateConstraintVectors<x0.rows()>(x0, model.lower_bound(), model.upper_bound());
//     if (!solver.updateBounds(model.lower_bound(), model.upper_bound()))
//       return 1;

//     // controller input and QPSolution vector
//     Eigen::VectorXd ctr;

//     // Assign zero control to action in case of failure.
//     u = ctr;

//     Eigen::VectorXd QPSolution;
//     // solve the QP model
//     if (!solver.solve()) return 1;

//     // get the controller input
//     QPSolution = solver.getSolution();
//     ctr = QPSolution.block(model.xMax().rows() * (mpc_horizon + 1), 0, model.uMax().rows(), 1);

//     u= ctr;
//     return 0;
    
// }



#endif