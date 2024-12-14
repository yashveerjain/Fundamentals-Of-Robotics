#include <iostream>

#include "data_handler.hpp"
#include "custom_graph_helper.hpp"

using std::cout;
using std::endl;

/**
 * Extracts a sequence of floating-point numbers from a given string.
 *
 * This function parses the input string starting from the specified index,
 * searching for whitespace-separated substrings that represent floating-point
 * numbers. It converts each of these substrings into a double and stores them
 * in a vector. The parsing continues until the end of the string is reached.
 *
 * @param line The string to parse, containing space-separated numbers.
 * @param start The starting index within the string from which to begin parsing.
 * @return A vector of doubles containing the parsed numbers from the string.
 */
vector<double> get_values_from_line(std::string line, size_t start){
    vector<double>val;
    size_t end = line.size();
    /*Search for the spaces and split the string based on whitespaces and 
    convert them to float and push it to vector (val)
    */
    while ((end = line.find(' ', start)) != std::string::npos) {
        // cout<<line.substr(start, end - start)<<endl;
        val.push_back(std::stod(line.substr(start,end-start)));
        start = end + 1;
    }
    // Add last value to vector val (as there is no space after last value, loop will terminate abruptly)
    // cout<<line.substr(start, end - start)<<endl;
    val.push_back(std::stof(line.substr(start,end-start)));
    return val;
}

int main(int argc, char* argv[]){
    string filepath="";
    if (argc>1){
        filepath = argv[1];
    }
    
    DataHandler data_handler(filepath);
    
    // data_handler.print_file();

    std::vector<string> lines = data_handler.read_file();

    // size_t start =0, end=0;
    std::vector<std::vector<double>> vertices;
    std::vector<std::vector<double>> edges;
    for (auto line: lines){
        
        if (line[0]=='V'){
            // start = 11; because after 11 index the values will start
            /**
             * will store the list of : index, x, y, theta
             */
            vertices.push_back(get_values_from_line(line,11));
            
        }
        else {
            // start = 9; because after 9 index the values will start
            /**
             * will store the list of [parent_idx child_idx x y theta q11 q12 q13 q22 q23 q33]
             * [q11 q12 q13 q22 q23 q33] This are the elements of upper triangular noise covariance matrix (3x3)
             * 
             */
            edges.push_back(get_values_from_line(line,9));
            
        }
    }

    CustomGraph custom_graph(vertices,edges);

    gtsam::Values res = custom_graph.Optimize();


    Eigen::MatrixXd outmat(res.size(),3);

    for (auto r : res){
        // r is a keyvalue pair
        auto v = res.at<gtsam::Pose2>(r.key);
        // cout<<r.key<<endl;
        int row = static_cast<int>(r.key); // Assuming key is convertible to int
        outmat(row, 0) = v.x();
        outmat(row, 1) = v.y();
        outmat(row, 2) = v.theta();
        // cout<<v.x()<<endl;
    }

    std::string model_path = "saved.bin";
    std::string csv_path = "saved.csv";

    data_handler.saveBinaryData(model_path, outmat);
    
    data_handler.loadBinaryData(model_path);

    data_handler.saveCsvData(csv_path, outmat);

    // Get the GT also and save it for plotting purposes
    gtsam::Values gt = custom_graph.getInitialEstimate();
    Eigen::MatrixXd gtmat(gt.size(),3);

    for (auto r : gt){
        // r is a keyvalue pair
        auto v = res.at<gtsam::Pose2>(r.key);
        // cout<<r.key<<endl;
        int row = static_cast<int>(r.key); // Assuming key is convertible to int
        gtmat(row, 0) = v.x();
        gtmat(row, 1) = v.y();
        gtmat(row, 2) = v.theta();
        // cout<<v.x()<<endl;
    }

    std::string gt_csv_path = "gt.csv";
    data_handler.saveCsvData(gt_csv_path, gtmat);

    return 0;
}