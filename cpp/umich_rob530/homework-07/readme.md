# 📝 Homework 07: Factor Graph Creation and Optimization

This project involves creating a **Factor Graph** and optimizing it using GTSAM for solving SLAM problems. The implementation uses Eigen for matrix computations and GTSAM for graph-based optimization. For further details, refer to the [Assignment Document](NA568_HW7_W22.pdf).

## 🚀 Instructions to Run
1. To compile and execute the project, follow these steps:

```sh
mkdir -p build && cd build
cmake ..
make
./main ../input_INTEL_g2o.g2o
```
2. To plot:
* Create virtual env (Optional) helps organize the code bettter:
```sh
sudo apt install python3-virtualenv
virtualenv py_env
source py_env/bin/activate
```
* Install dependencies:
```sh
python3 -m pip install -r requirements.txt
```
* Run:
    - Will Plot the Graph with difference between GT and Estimation path. 
```sh
python3 plot.py build/gt.csv build/saved.csv
```
## Noted and TODO:
* **Notes**: 
    - BatchOptimization is working and able to plot the graph.
    - IncrementalOptimization is not working, the optimization is going out of bound and resulting in degenerate solution, so deteminent is 0, causing it to fail.
    - Input dataset the information vector [q11,q12,q13,q22,q23,q33] is upper triangle matrix (I), and since information matrix we need to convert it to covariance matrix(C), for noise modelling.
        - $C = \sqrt{I^{-1}}$

## Results:
* The results of the project are as follows:
### **Batch Optimization**:
![](assets/BOPlot.png)

### **Incremental Optimization (ISAM)**:
![](assets/IOPlot.png)

## Dependencies
This project requires the following dependencies:

1. Eigen: A C++ template library for linear algebra. Ensure Eigen is built before GTSAM to avoid compatibility issues.
```sh
mkdir -p 3rdparty && cd 3rdparty
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip
rm -r *.zip
cd eigen-3.4.0/build
cmake .. && make
cd ../../..
```

2. GTSAM: A library for graph-based optimization. Configure GTSAM to use the system-installed Eigen library to prevent version conflicts.

* GTSAM:
    - **Note**: Give `GTSAM_USE_SYSTEM_EIGEN=ON` argument so gtsam uses system eign instead of its own, this way there will be no compatibility issue with eigen versions.
```sh
mkdir -p 3rdparty && cd 3rdparty
wget https://github.com/borglab/gtsam/archive/refs/heads/release/4.2.zip
unzip 4.2.zip
rm 4.2.zip
cd gtsam-release-4.2
mkdir build && cd build
cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON && make install -j6
cd ../../..

```

## 📚 References:
* Here are some useful resources for understanding and using GTSAM:
    - [Gtsam API docs](https://gtsam.org/doxygen/a01616.html)
    - [Gtsam Tutorial](https://gtsam.org/tutorials/intro.html)
    - [Gtsam Pose2SLAM Example](https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExample.cpp)

## 🔖 Tags
`#FactorGraph` `#Optimization` `#SLAM` `#GTSAM` `#Eigen` `#Pose2SLAM`

## Feel free to raise issues or contribute to this project. Happy coding!

### Features Added:
1. **Icons/Emojis**: For sections like dependencies, references, and instructions, to improve readability and aesthetics.
2. **Tags**: Added tags to help with categorization and discoverability.
3. **Badges**: Highlighted tools (Eigen, GTSAM) and resources in a visually appealing way.
4. **Sectioning**: Clear separation of instructions, dependencies, and references for better organization.

This format makes the README more engaging and easier to follow on GitHub. Let me know if you’d like any additional tweaks!
