
#include "custom_graph_helper.hpp"

/**
 * Constructor for CustomGraph class.
 * 
 * @param vertices List of vertices. Each vertex is a list of [index, x, y, theta]
 * @param edges List of edges. Each edge is a list of [parent_idx child_idx x y theta q11 q12 q13 q22 q23 q33]
 * [q11 q12 q13 q22 q23 q33] This are the elements of upper triangular noise covariance matrix (3x3)
 * 
 * The constructor will add the prior node to the factor graph and then add all the factor nodes to the graph.
 */
CustomGraph::CustomGraph(vector<vector<double>> vertices, vector<vector<double>> edges){
            
    for (auto v: vertices) _initial.insert(static_cast<int>(v[0]),gtsam::Pose2(v[1],v[2],v[3]));
    // Reference for Pose2 : https://gtsam.org/doxygen/a03264.html
    
    // Adding prior node in factor graph
    /**
     * Assuming 0 node is prior, and its odometry initial mean is (0,0,0) and variance is same 0-1 factor node.
     */
    auto pre_e = edges[0];
    Eigen::Matrix3d cov_matrix;
    cov_matrix << pre_e[5], pre_e[6], pre_e[7],
            0, pre_e[8], pre_e[9],
            0, 0, pre_e[10];
    
    // Create noise model based on gaussian covariance
    auto cov_model = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);
    _graph.addPrior(0,gtsam::Pose2(0,0,0),cov_model);
    
    for (auto e: edges){
        /**
         * e contains list of [parent_idx child_idx x y theta q11 q12 q13 q22 q23 q33]
     * [q11 q12 q13 q22 q23 q33] This are the elements of upper triangular noise covariance matrix (3x3)
         */
        int pi=e[0], ci=e[1];
        double x=e[2],y=e[3],theta=e[4];

        cov_matrix << e[5], e[6], e[7],
                0, e[8], e[9],
                0, 0, e[10];
        
        // Create noise model based on gaussian covariance
        cov_model = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);
        
        // add the factor nodes to the graph.
        _graph.add(gtsam::BetweenFactor<gtsam::Pose2>(pi, ci, gtsam::Pose2(x, y, theta), cov_model));
    }
}

/**
 * Optimizes the factor graph using the Gauss-Newton nonlinear optimizer.
 *
 * This function performs optimization on the initial values of the factor graph
 * by setting up a Gauss-Newton optimizer with specified parameters such as 
 * relative error tolerance and maximum iterations. It returns the optimized 
 * values after convergence.
 *
 * @return The optimized values of the factor graph.
 */
gtsam::Values CustomGraph::Optimize(){
        // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
    // The optimizer accepts an optional set of configuration parameters,
    // controlling things like convergence criteria, the type of linear
    // system solver to use, and the amount of information displayed during
    // optimization. We will set a few parameters as a demonstration.
    gtsam::GaussNewtonParams parameters;
    // Stop iterating once the change in error between steps is less than this value
    parameters.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    parameters.maxIterations = 100;
    // Create the optimizer ...
    gtsam::GaussNewtonOptimizer optimizer(_graph, _initial, parameters);
    // ... and optimize
    gtsam::Values result =  optimizer.optimize();
    // result.print("Final Result:\n");

    return result;
}

gtsam::Values CustomGraph::getInitialEstimate(){
    return _initial;
}