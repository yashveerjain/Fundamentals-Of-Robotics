#include "data_handler.hpp"

/**
 * Prints the contents of the file specified by `_filepath` line by line.
 *
 * If the file cannot be opened, prints an error message.
 */
void DataHandler::print_file(){
    std::ifstream file(_filepath);

    if (file.is_open()){
        cout<<"Printing file: "<<_filepath<<endl;
        std::string line;
        while (std::getline(file, line)){
            cout<<line<<endl;
            // if ('V'==line[0])cout<<line.substr(10,line.size())<<endl;
            // else cout<<line.substr(8,line.size())<<endl;
        }
        file.close();
    }
    else {
        std::cerr << "Unable to open file: " << _filepath << endl;
    }
}

/**
 * Reads the contents of the file specified by `_filepath` line by line and returns a vector of strings, each representing a line from the file.
 * If the file cannot be opened, prints an error message and returns an empty vector.
 */
std::vector<string> DataHandler::read_file(){
    std::vector<string> lines;
    std::ifstream file(_filepath);
    if (file.is_open()){
        cout<<"Reading file: "<<_filepath<<endl;
        std::string line;
        while (std::getline(file, line)){
            lines.push_back(line);
        }
        file.close();
    }
    else {
        std::cerr << "Unable to open file: " << _filepath << endl;
    }
    return lines;
}

/**
 * Saves the given matrix to a binary file specified by the model path.
 *
 * This function writes the dimensions (rows and columns) of the provided
 * Eigen matrix `outmat` to the specified file `model_path` in binary format,
 * followed by the matrix data itself. The matrix is stored in column-major
 * order as per Eigen's default storage order.
 *
 * @param model_path The file path where the matrix should be saved.
 * @param outmat The Eigen matrix to be saved to the file.
 */
void DataHandler::saveBinaryData(std::string model_path, Eigen::MatrixXd& outmat){
    std::ofstream ofile(model_path, std::ios_base::binary | std::ios_base::out);

    int row = outmat.rows(), col = outmat.cols();
    // reference : https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html
    ofile.write(reinterpret_cast<char*>(&row),sizeof(int));
    ofile.write(reinterpret_cast<char*>(&col),sizeof(int));
    ofile.write(reinterpret_cast<char*>(outmat.data()), sizeof(double)*outmat.size());

    printf("Saving the weight with mean %f in file %s\n",outmat.mean(),model_path);
    ofile.close();
}

/**
 * Loads a matrix from a binary file specified by the given model path.
 *
 * This function opens the file in binary read mode and reads the matrix
 * dimensions (rows and columns) from the file, followed by the matrix data.
 * It then constructs and returns an Eigen::MatrixXd with the loaded data.
 * If the file cannot be opened, it throws a runtime error.
 *
 * @param model_path The path to the binary file containing the matrix data.
 * @return An Eigen::MatrixXd containing the loaded matrix.
 * @throws std::runtime_error if the file cannot be opened.
 */
Eigen::MatrixXd DataHandler::loadBinaryData(std::string model_path){
    
    std::ifstream ifile(model_path, std::ios_base::binary | std::ios_base::in);

    if (!ifile.is_open()){
        throw std::runtime_error("Model path doesn't exist"+model_path);
    }
    int row, col;

    // int row,col;
    ifile.read(reinterpret_cast<char*>(&row),sizeof(int));
    ifile.read(reinterpret_cast<char*>(&col),sizeof(int));
    Eigen::MatrixXd mat(row,col);
    std::cout<<"Rows : "<<mat.rows()<<" Cols : "<<mat.cols()<<std::endl;
    
    // need to know the size of the weight before hand to get the data from the file.
    ifile.read(reinterpret_cast<char*>(mat.data()), sizeof(double)*mat.size());

    printf("Loading the weight with mean %f from file %s\n",mat.mean(),model_path);

    ifile.close();
    return mat;
}

void DataHandler::saveCsvData(std::string model_path, Eigen::MatrixXd& outmat){
    std::ofstream ofile(model_path);
    int row = outmat.rows(), col = outmat.cols();

    for (int i=0;i<row;i++){
        for (int j=0;j<col;j++){
            ofile<<outmat(i,j)<<",";
        }
        ofile<<"\n";
    }
    ofile.close();
}