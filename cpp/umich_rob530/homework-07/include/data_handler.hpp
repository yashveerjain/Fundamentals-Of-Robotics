# pragma once

#include<iostream>
#include<string>
#include<fstream>
#include<vector>
#include<Eigen/Dense>

using std::string;
using std::cout;
using std::endl;

class DataHandler{
    string _filepath = "";
    public:

        /**
         * @brief This Constructor will not initialize with filepath to read the file, but instead can create
         * data handler object to save and load the results data to specified file in the method argument.
         */
        DataHandler(){}

        /**
         * @brief Constructs a DataHandler object with the specified file path.
         *
         * This constructor initializes the DataHandler with a given file path and 
         * displays information about the expected file format and how it should be processed.
         * 
         * The expected file is `input_INTEL_g2o.g2o`, which contains 2D data in G2O format.
         * The pose (state variable) is represented as [VERTEX_SE2 i x y theta] and the 
         * edge as [EDGE_SE2 i j x y theta info(x, y, theta)], where info(x, y, theta) is 
         * a 1 × 6 vector [q11 q12 q13 q22 q23 q33].
         * 
         * The elements in the vector represent the upper-triangle matrix of the 3 × 3 
         * information matrix Ω:
         * 
         * Ω = [q11 q12 q13]
         *     [q12 q22 q23]
         *     [q13 q23 q33]
         *
         * By inverting this information matrix, the covariance matrix for the noise model 
         * can be obtained.
         *
         * @param filepath The path to the file to be processed.
         */
        DataHandler(string filepath): _filepath(filepath){
            cout<<"========================="<<endl;
            cout<<"File can be downloaded from : https://www.dropbox.com/s/vcz8cag7bo0zlaj/input_INTEL_g2o.g2o?dl=0"<<endl;
            cout<<"The Model is expecting only this file `input_INTEL_g2o.g2o`, for processing"<<endl;
            cout<<"Details about this file ->"<<endl;
            cout << "For 2D data, the pose (state variable) in G2O format is [VERTEX_SE2 i x y theta] and the edge in G2O format is [EDGE_SE2 i j x y theta info(x, y, theta)], where info(x, y, theta) is a 1 × 6 vector [q11 q12 q13 q22 q23 q33]." << endl;
            cout << "The elements represent the upper-triangle matrix of the 3 × 3 information matrix Ω:" << endl;
            cout << "Ω = [q11 q12 q13]" << endl;
            cout << "    [q12 q22 q23]" << endl;
            cout << "    [q13 q23 q33]" << endl;

            cout << "By inverting this information matrix = (Ω), you can obtain the covariance matrix = (Sqrt(Ω^-1)) for the noise model." << endl;
            cout<<"========================="<<endl;
        }

        void print_file();

        std::vector<string> read_file();

        void saveBinaryData(std::string model_path, Eigen::MatrixXd& outmat);

        Eigen::MatrixXd loadBinaryData(std::string model_path);

        void saveCsvData(std::string model_path, Eigen::MatrixXd& outmat);
        Eigen::MatrixXd loadCsvData(std::string model_path);

};