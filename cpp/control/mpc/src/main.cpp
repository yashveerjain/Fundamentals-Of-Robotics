#include<iostream>
#include<vector>
#include<Eigen/Dense>
#include "mpc_driver.hpp"
#include "dynamic_model.hpp"

using std::cout;
using std::endl;
using namespace Eigen;

// using Eigen::MatrixXd;

int main(int argc, char** argv) {
    // MPCController mpc_controller = MPCController(0.01, 10);

    int prediction_horizon = 100;

    Eigen::MatrixXd x = MatrixXd::Zero(4,1);
    double u = 5.0;

    DynamicModel dynamic_model = DynamicModel(0.001, prediction_horizon);
    Eigen::MatrixXd M = dynamic_model.getM();
    Eigen::MatrixXd O = dynamic_model.getO();
    

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(prediction_horizon, prediction_horizon);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(prediction_horizon, prediction_horizon);

    Q(0,0) = 0.0001;
    P(0,0) = 10;
    for (int i=1; i<prediction_horizon; i++) {
        Q(i,i) = 0.001;
        P(i,i) = 10;
    }

    Eigen::MatrixXd stacked_X = dynamic_model.getStackedX(x, u);
    Eigen::MatrixXd stacked_u = dynamic_model.getStackedU(u);


    cout<< "Stacked X : "<<stacked_X.mean()<<endl<<"Stacked U : "<<stacked_u.mean()<<endl;

    MPCController mpc_controller = MPCController(Q, P, M, O);

    int total_timesteps = 300;
    int high_timesteps = 100;

    Eigen::MatrixXd desired_traj = Eigen::MatrixXd::Zero(total_timesteps,1);
    Eigen::MatrixXd temp1 = Eigen::MatrixXd::Ones(high_timesteps,1);


    // This will give 2 peaks for 100 timesteps of desired trajectory, which controller need to track
    desired_traj.block(0,0,high_timesteps,1) = temp1;
    desired_traj.block(200,0,high_timesteps,1) = temp1;

    for (int i =0; i<total_timesteps-prediction_horizon; i++) {
        
        Eigen::MatrixXd sub_desired_traj = Eigen::MatrixXd::Zero(prediction_horizon,1);

        sub_desired_traj.block(0,0,prediction_horizon,1) = desired_traj.block(i,0,prediction_horizon,1);
        Eigen::MatrixXd d = mpc_controller.solve(stacked_X, stacked_u, sub_desired_traj);
        Eigen::MatrixXd nextState = dynamic_model.getState(x, d(0,0));
        Eigen::MatrixXd output = dynamic_model.getOutput(nextState);

        x = dynamic_model.getNextState(stacked_X, d(0,0));
        stacked_X = dynamic_model.getStackedX(x, d(0,0));
        cout<<"Optimized Control input : "<<d(0,0)<<", Current Control input : "<<stacked_u(0,0)<<endl;
        stacked_u = d;

        cout<< "Stacked X : "<<stacked_X.mean()<<endl<<"Stacked U : "<<stacked_u.mean()<<endl;

        cout<<"Current output : "<<output << ", Desired output : "<<desired_traj.block(i,0,1,1)<<endl;
        // cout<< "d mean : "<<d.mean()<<", d shape : "<<d.rows()<<", "<<d.cols()<<endl;
    }
    return 0;
}