
#include "custom_graph_helper.hpp"


/**
 * @brief Constructor for the CustomGraph class.
 * 
 * Initializes the object with a list of vertices and edges, and creates an initial estimate of the graph's state.
 * 
 * @param vertices A 2D vector of doubles representing the vertices of the graph.
 * @param edges A 2D vector of doubles representing the edges of the graph.
 */
CustomGraph::CustomGraph(vector<vector<double>> vertices, vector<vector<double>> edges): _edges(edges), _vertices(vertices){
    for (auto v: vertices) _initial.insert(static_cast<int>(v[0]),gtsam::Pose2(v[1],v[2],v[3]));
}


/**
 * @brief Batch optimization of graph using Gauss-Newton nonlinear optimizer.
 * 
 * This function creates a factor graph with a prior node and between factor nodes and optimizes the graph using a Gauss-Newton nonlinear optimizer.
 * The optimization parameters are set to stop iterating when the change in error between steps is less than 1e-5 and to perform at most 100 iteration steps.
 * The optimized values are returned.
 * 
 * @return gtsam::Values The optimized values of the graph's state.
 */
gtsam::Values CustomGraph::BatchOptimization(){

    gtsam::NonlinearFactorGraph _graph;

    std::cout<<"Creating Graph...."<<std::endl;

    // Reference for Pose2 : https://gtsam.org/doxygen/a03264.html
    
    // Adding prior node in factor graph
    /**
     * Assuming 0 node is prior, and its odometry initial mean is (0,0,0) and variance is same 0-1 factor node.
     */
    auto pre_e = _edges[0];
    
    // Create noise model based on gaussian covariance
    // auto cov_model = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
    _graph.addPrior(0,gtsam::Pose2(_vertices[0][1],_vertices[0][2],_vertices[0][3]), noise_model);
    
    for (auto e: _edges){
        /**
         * e contains list of [parent_idx child_idx x y theta q11 q12 q13 q22 q23 q33]
     * [q11 q12 q13 q22 q23 q33] This are the elements of upper triangular noise covariance matrix (3x3)
         */
        int pi=e[0], ci=e[1];
        double x=e[2],y=e[3],theta=e[4];
        Eigen::Matrix3d info_matrix {{e[5], e[6], e[7]},
                                    {0, e[8], e[9]},
                                    {0, 0, e[10]}};
        Eigen::Matrix3d cov_matrix = info_matrix.inverse().array().sqrt();
        
        // Create noise model based on gaussian covariance
        auto cov_model = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);
        // Create noise model based on gaussian covariance
        cov_model = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);
        
        // add the factor nodes to the graph.
        _graph.add(gtsam::BetweenFactor<gtsam::Pose2>(pi, ci, gtsam::Pose2(x, y, theta), cov_model));
    }
    std::cout<<"Graph Successfully Created!!!"<<std::endl;


    std::cout<<" Running Batch Optimization...."<<std::endl;

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
    std::cout<<"Optimization Completed!!!"<<std::endl;
    return result;
}


/**
 * @brief Incremental optimization of graph using ISAM2.
 * 
 * This function performs incremental optimization of the graph using the ISAM2 algorithm.
 * The graph is updated sequentially with each vertex, and the optimization is performed
 * using the ISAM2 (Incremental Smoothing and Mapping) algorithm. The optimized values are returned.
 * 
 * @return gtsam::Values The optimized values of the graph's state.
 */
gtsam::Values CustomGraph::IncrementalOptimization(){
    // My assumpttion is that _vertices will have first index as prior node 

    auto iSam = gtsam::ISAM2();
    gtsam::Values result;
    vector<int> seen_indices;

    std::cout<<" Running Incremental Optimization...."<<std::endl;

    for (auto v: _vertices){
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_est;
        seen_indices.push_back(v[0]);
        // cout<<"Vertex: "<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<endl;
        if (v[0]==0){
            // std::cout<<"Adding prior node"<<std::endl;
            // Add the estimation of the prior node
            // if (initial_est.exists(v[0])) initial_est.erase(v[0]);
            initial_est.insert(v[0],gtsam::Pose2(v[1],v[2],v[3]));
            // if (initial_est.exists(v[0])) std::cout<<"Initial Estimation for node "<<v[0]<<" is "<<initial_est.at<gtsam::Pose2>(v[0])<<std::endl;
    
            // Random noise model
            auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
            
            // Add the prior factor
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(0,gtsam::Pose2(v[1],v[2],v[3]),noise_model));
        }
        else {
            // Add the estimation of the previous node
            // initial_est.erase(v[0]);
            // if (initial_est.exists(v[0])) initial_est.erase(v[0]);
            // initial_est.insert(v[0],result.at<gtsam::Pose2>());
            // cout<<"Result Estimation for previous node "<<v[0]-1<<" is "<<result.at<gtsam::Pose2>(v[0]-1)<<std::endl;
            initial_est.insert(v[0], result.at<gtsam::Pose2>(v[0]-1));//gtsam::Pose2(v[1],v[2],v[3]));
            // std::cout<<"Initial Estimation for node "<<v[0]<<" is "<<initial_est.at<gtsam::Pose2>(v[0])<<std::endl;
    
        
            // initial_est.update(static_cast<int>(v[0]),result.at<gtsam::Pose2>(v[0]-1));
            
            for (auto e: _edges){
                int pi=e[0], ci=e[1];
                if (ci==v[0]){
                    // std::cout<<"Adding edge between "<<pi<<" and "<<ci<<std::endl;
                    // auto it = std::find(seen_indices.begin(), seen_indices.end(), pi);
                    // if (it == seen_indices.end()){
                    //     // avoid using edges whose vertices have not been seen
                    //     continue;
                    // }

                    double dx=e[2],dy=e[3],dtheta=e[4];
                    Eigen::Matrix3d info_matrix {{e[5], e[6], e[7]},
                                                {0, e[8], e[9]},
                                                {0, 0, e[10]}};
                    Eigen::Matrix3d cov_matrix = info_matrix.inverse().array().sqrt();
                    
                    // Create noise model based on gaussian covariance
                    auto cov_model = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);
                    
                    // add the factor nodes to the graph.
                    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(pi, ci, gtsam::Pose2(dx, dy, dtheta), cov_model));
                }
            }
        }
        // cout<<"Adding factor graph for node "<<v[0]<<endl;
        // Purpose: Updates the ISAM2 factor graph with new factors and/or variables and performs a single step of the incremental optimization process.
        iSam.update(graph, initial_est);
        // cout<<"Updated factor graph for node "<<v[0]<<endl;

        // Typical Use Case: To query the optimized state of the system after calling update.
        result = iSam.calculateEstimate();
    }

    std::cout<<"Optimization Completed!!!"<<std::endl;
    return iSam.calculateEstimate();
}



/**
 * @brief Get the initial estimate of the graph's state.
 * 
 * This function returns a gtsam::Values object representing the initial estimate of the graph's state.
 * The initial estimate is set when the CustomGraph object is created.
 * 
 * @return a gtsam::Values object representing the initial estimate of the graph's state.
 */
gtsam::Values CustomGraph::getInitialEstimate(){
    return _initial;
}