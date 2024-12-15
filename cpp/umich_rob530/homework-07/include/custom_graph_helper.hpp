#pragma once

#include<iostream>
#include<vector>
#include<string>
#include<Eigen/Dense>

/*references: 
* https://gtsam.org/doxygen/a01596.html
* https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExample.cpp
*/

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// We will also use ISAM2 to perform incremental nonlinear optimization
#include <gtsam/nonlinear/ISAM2.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>



using std::vector;
using std::cout;
using std::endl;

class CustomGraph{
    
    gtsam::Values _initial; // state variables (need to optimize)
    vector<vector<double>> _vertices, _edges;
    public:
        /**
         * @brief Constructor for the CustomGraph class.
         * 
         * Initializes the object with a list of vertices and edges, and creates an initial estimate of the graph's state.
         * 
         * @param vertices A 2D vector of doubles representing the vertices of the graph.
         * @param edges A 2D vector of doubles representing the edges of the graph.
         */
        CustomGraph(vector<vector<double>> vertices, vector<vector<double>> edges);
        
        /**
         * @brief Batch optimization of graph using Gauss-Newton nonlinear optimizer.
         * 
         * This function creates a factor graph with a prior node and between factor nodes and optimizes the graph using a Gauss-Newton nonlinear optimizer.
         * The optimization parameters are set to stop iterating when the change in error between steps is less than 1e-5 and to perform at most 100 iteration steps.
         * The optimized values are returned.
         * 
         * @return gtsam::Values The optimized values of the graph's state.
         */
        gtsam::Values BatchOptimization();
        
        /**
         * @brief Incremental optimization of graph using ISAM2.
         * 
         * This function performs incremental optimization of the graph using the ISAM2 algorithm.
         * The graph is updated sequentially with each vertex, and the optimization is performed
         * using the ISAM2 (Incremental Smoothing and Mapping) algorithm. The optimized values are returned.
         * 
         * @return gtsam::Values The optimized values of the graph's state.
         */
        gtsam::Values IncrementalOptimization();


        /**
         * @brief Get the initial estimate of the graph's state.
         * 
         * This function returns a gtsam::Values object representing the initial estimate of the graph's state.
         * The initial estimate is set when the CustomGraph object is created.
         * 
         * @return a gtsam::Values object representing the initial estimate of the graph's state.
         */    
        gtsam::Values getInitialEstimate();     
};
