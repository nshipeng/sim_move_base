/*
 * @Description: Differential Drive
 * @Autor: 
 * @Date: 2022-01-07 14:27:58
 * @LastEditTime: 2022-01-11 15:04:23
 */

#ifndef _DIFF_DRIVE_H_
#define __DIFF_DRIVE_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;

//x(k+1) = Ax(k) + Bu;

class DiffDrive
{
    public:
        
        DiffDrive(){
            A_.setIdentity();
        }

        ~DiffDrive(){};

        MatrixXd A(){
            return A_;
        }

        MatrixXd B(const MatrixXd& state) {

            //state => (x,y, theta)
            double theta = state(2);
            /**
             * th: robot heading
             * | cos(th),  0 |     | v |
             * | sin(th),  0 |  X  |   |
             * | 0,        1 |     | w |
             */

            B_ << std::cos(theta),    0,
                 std::sin(theta),    0,
                 0,                  1;

            return B_;
        }

        /**
         * @brief Returns the number of variables in this optimization.
         *
         * @param mpcWindow Timesteps for which we perform MPC.
         */
        int num_variables(const int& mpcWindow) {
            return A_.rows() * (mpcWindow + 1) + B_.cols() * mpcWindow;
        }

        /**
         * @brief Returns the number of constraints in this optimization.
         *
         * @param mpcWindow Timesteps for which we perform MPC.
         */
        int num_constraints(const int& mpcWindow) {
            return 2 * A_.rows() * (mpcWindow + 1) + B_.cols() * mpcWindow;
        }

        /**
         * @brief
         *
         * @return Eigen::SparseMatrix<double>&
         */
        Eigen::SparseMatrix<double>& hessian() { return m_hessian; }

        /**
         * @brief
         *
         * @return VectorXd&
         */
        VectorXd& gradient() { return m_gradient; }

        /**
         * @brief
         *
         * @return MatrixXd&
         */
        MatrixXd& linear_constraint_matrix() { return m_linear_constraint_matrix; }

        /**
         * @brief
         *
         * @return VectorXd&
         */
        VectorXd& lower_bound() { return m_lower_bound; }

        /**
         * @brief
         *
         * @return VectorXd&
         */
        VectorXd& upper_bound() { return m_upper_bound; }

 

    private:
        //x(k+1) = Ax+Bu      
        Eigen::Matrix<double,3,3> A_;
        Eigen::Matrix<double,3,2> B_;

        //QP parameter.
        VectorXd m_gradient;
        Eigen::SparseMatrix<double> m_hessian;
        MatrixXd m_linear_constraint_matrix;
        VectorXd m_lower_bound, m_upper_bound;
};




#endif