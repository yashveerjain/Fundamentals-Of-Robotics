#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>


class MPCController {
    Eigen::MatrixXd _Q; // weights for input cost
    Eigen::MatrixXd _P; // weights for target cost

    Eigen::MatrixXd _M; // Lifted form of D matrix (Y=CX+DU)
    Eigen::MatrixXd _O; // Lifted form of C matrix (Y=CX+DU)

    Eigen::MatrixXd _gain_matrix;
    
    public:
        /**
         * @brief Construct a new MPC Controller object
         * 
         * @param Q weights for input cost
         * @param P weights for target cost 
         * @param M Lifted form of D matrix (Y=CX+DU)
         * @param O Lifted form of C matrix (Y=CX+DU)
        */
        MPCController(Eigen::MatrixXd Q, Eigen::MatrixXd P, Eigen::MatrixXd M, Eigen::MatrixXd O): _Q(Q), _P(P), _M(M), _O(O) {
            _gain_matrix = (_M.transpose()*_P*_M + _Q).inverse() * (_M.transpose()*_P);
        }

        /**
         * @brief Solves the Linear optimization problem using Linear Least Squares method
         * @param stacked_X stacked state vector of shape (prediction_horizon*n, 1)
         * @param stacked_u stacked input vector of shape (prediction_horizon, 1)
         * @param desired_target desired target vector of shape (n, 1)
         * 
         * @return Eigen::MatrixXd d vector of shape (n, 1), the optimal input
         */
        Eigen::MatrixXd solve(Eigen::MatrixXd stacked_X, Eigen::MatrixXd stacked_u, Eigen::MatrixXd desired_target){
            // stacked_X = [X1, X2, ..., XN], shape = (prediction_horizon*n, 1)
            // stacked_u = [u1, u2, ..., uN], shape = (prediction_horizon, 1) input is scalar 

            double sum_error = 0;
            // int n = _O.cols();

            Eigen::MatrixXd error(desired_target.rows(), 1);
            int n = _O.cols();
            for (int i=0; i<desired_target.rows(); i++){
                error.block(i,0,1,1) = desired_target.block(i,0,1,1) - _O.block(i,0,1,n)*stacked_X.block(i*n,0,n,1); // (Eigen::seq(i*n, (i+1)*n));
            }

            std::cout<<"error : "<<error.mean()<<std::endl;
            assert(_M.rows() == _P.cols() && _M.cols() == _M.rows());
            // assert(_M.rows() == _P.rows());
            assert(_M.rows() == _Q.rows() && _M.cols() == _Q.cols());
            assert(_P.cols()==error.rows());


            Eigen::MatrixXd d = _gain_matrix*error;

            return d; 
 
        }
};