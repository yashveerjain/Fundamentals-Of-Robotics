#pragma once

#include<iostream>
#include<vector>
#include<Eigen/Dense>
#include<string>


// System Model we are using can be found here: https://aleksandarhaber.com/introduction-to-subspace-system-identification-system-identification-tutorial/

class DynamicModel{
    // Mass Spring System Model
    double _m1{20},_m2{20}; // mass
    double _k1{1000}, _k2{2000}; // spring
    double _d1 = 1, _d2 = 5; // damping

    Eigen::MatrixXd _Ac {{0,1,0,0},
                        {-(_k1+_k2)/_m1, -(_d1+_d2)/_m1, _k2/_m1, _d2/_m1}, 
                        {0,0,0,1},
                        {_k2/_m2, _d2/_m2, -_k2/_m2, -_d2/_m2}}; 
    Eigen::MatrixXd _Bc {{0},{0},{0},{1/_m2}};

    Eigen::MatrixXd _C {{1,0,0,0}};

    Eigen::MatrixXd _A, _B;

    double _sampling_time=0.01;
    int _prediction_horizon = 10;

    // because we have already defined the A, B and C matrices, so we are defining the there dimensions here:
    
    public:
        int n = _Ac.rows(), m = _Bc.cols(), r = _C.rows();
        // n is the number of states, m is the number of inputs, r is the number of outputs


        DynamicModel(){
            initializeParameters();
        }

        DynamicModel(double sampling_time): _sampling_time(sampling_time){
            initializeParameters();
        
        }

        DynamicModel(int prediction_horizon): _prediction_horizon(prediction_horizon){
            initializeParameters();
        
        }

        DynamicModel(double sampling_time, int prediction_horizon): _sampling_time(sampling_time), _prediction_horizon(prediction_horizon){
        
            initializeParameters();
        
        }

        void initializeParameters(){

            _A=(Eigen::MatrixXd::Identity(4,4) - _sampling_time *  _Ac).inverse();
            _B=_sampling_time * _A * _Bc;

            std::cout<<"Dynamic Model initialized"<<std::endl;
            std::cout<<"Sampling Time : "<<_sampling_time<<std::endl;
            std::cout<< "A : "<<_A.rows()<<", "<<_A.cols()<<std::endl;
            std::cout<< "B : "<<_B.rows()<<", "<<_B.cols()<<std::endl;
        }

        Eigen::MatrixXd getA() {return _A;}
        Eigen::MatrixXd getB() {return _B;}
        Eigen::MatrixXd getC() {return _C;}

        Eigen::MatrixXd getState(Eigen::MatrixXd x, double u){
            // X shape (4,1), U shape (1,1)
            return _A * x + _B * u;
        }

        Eigen::MatrixXd getOutput(Eigen::MatrixXd x){
            // X shape (4,1)
            return _C * x;
        }

        /**
         * @brief Calculated the lifted form of O matrix (Y=CX), and by lifted form means stack of O matrix
         * for given prediction horizon
         * @return Eigen::MatrixXd O
         */
        Eigen::MatrixXd getO(){
            Eigen::MatrixXd O = Eigen::MatrixXd::Zero(_prediction_horizon*r,n);
            Eigen::MatrixXd powA(n,n);
            // std::cout<<"O shape : "<<O.rows()<<", "<<O.cols()<<std::endl;
            for (int i=0;i<_prediction_horizon;i++){
                if (i==0) powA = _A;
                else powA = powA * _A;

                O(i,Eigen::all) = _C * powA;
            }
            return O;
        }

        /**
         * @brief Calculated the lifted form of D matrix (Y=CX+DU), and by lifted form means stack of D matrix
         * for given prediction horizon
         * @return Eigen::MatrixXd M 
         */
        Eigen::MatrixXd getM(){
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(_prediction_horizon, _prediction_horizon);
            Eigen::MatrixXd powA(n,n);
            for (int i=0;i<_prediction_horizon;i++){
                if (i==0){
                    Eigen::MatrixXd res = _C * _B;
                    // M(Eigen::seq(i,i),0) = _C*_B;
                    M.block(i,0,1,1) = res;
                    
                }
                else{
                    powA = powA * _A;
                    M(Eigen::seq(i,i),0) = _C * powA * _B;
                    // M(i,Eigen::seq(1,i)) = M(i-1,Eigen::seq(0,i-1)) ;
                    M.block(i,1,1,i) = M.block(i-1,0,1,i);
                }
            }
            // std::cout<<"M shape : "<<M.rows()<<", "<<M.cols()<<std::endl;
            // std::cout<<"M : "<<M<<std::endl;
            return M;
        }

        /**
         * @brief Given the initial state x and the input u, stack the state for the given prediction horizon
         * @param x initial state
         * @param u input
         * @return Eigen::MatrixXd stacked_X, the stacked state for the given prediction horizon
         */

        Eigen::MatrixXd getStackedX(Eigen::MatrixXd x, double u){
            // X shape (4,1), U shape (1,1)
            Eigen::MatrixXd stacked_X = Eigen::MatrixXd::Zero(_prediction_horizon*n,1);
            for (int i=0;i<_prediction_horizon;i++){
                stacked_X.block(i * n, 0, n, 1) = x;
                x = getState(x,u);
                // stacked_X(Eigen::seq(i*n, (i+1)*(n-1)),0) = x(Eigen::all,0);
            }
            return stacked_X;

        }

        Eigen::MatrixXd getNextState(Eigen::MatrixXd stacked_X, double u){
            // X shape (4,1), U shape (1,1)
            Eigen::MatrixXd x = stacked_X.block(0, 0, n, 1);
            x = getState(x, u);
            return x;
        }

        /**
         * @brief Given the input u, stack the input for the given prediction horizon
         * @param u input
         * @return Eigen::MatrixXd stacked_U, the stacked input for the given prediction horizon
         */
        Eigen::MatrixXd getStackedU(double u){
            // U shape (1,1)
            Eigen::MatrixXd stacked_U = Eigen::MatrixXd::Ones(_prediction_horizon,1) * u;
            return stacked_U;
        }

};