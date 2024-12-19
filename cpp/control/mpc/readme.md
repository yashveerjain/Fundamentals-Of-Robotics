# MPC Controller for simple Linear System

This is a simple Linear System Controller that uses MPC to control the system.

## Dependencies
This project requires the following dependencies:
* Eigen: A C++ template library for linear algebra.

## Instructions to Run
1. To compile and execute the project, follow these steps:

```sh
mkdir -p build && cd build
cmake ..
make
./mpc
```

## TODO:
- [ ] MPC Controller still has some issues, need to fix it.
- [ ] Create a plotting pipeline to plot the results.
- [ ] Add non-linear system and non-linear MPC controller.

## References
* [MPC Implementation](https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-2-unconstrained-solution-for-linear-systems-and-implementation-in-c-from-scratch-by-using-eigen-c-library/)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)