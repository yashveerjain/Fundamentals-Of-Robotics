## Homework 4

Implementation of Right Invariant Extended Kalman Filter (RIEKF)

## Implementation details
* RIEKF uses only gyroscope and accelerometer data
* Gyroscope as state R_dot = R * omega + noise
* Accelerometer as measurement z = R * b + noise


## Model
* Process model:
    - $\dot R = R * (\omega - \omega_g) $
* Observation model:
    - Y = R * g + V
* Where,
    - R is the rotation matrix
    - g is the gravity vector, in RIEKF term $g = b$
    - V is the noise
    - $\omega_g$ is the gyro noise
    - $\omega$ is the gyroscope reading


## RIEKF Has 2 step implementation

### Prediction step
- Predict the next state
    - $\dot R = R * (\omega)$
- Predict the next state covariance
    - $\dot P = \Phi P \Phi^T + Ad_x \Phi Q \Phi^T Ad_x^T$ 

- where,
    - $\Phi$ is the state transition matrix = $exp(A_t^r * dt)$ -> linearized state transition matrix = $I + A_t^r * dt$
    - $A_t^r$ is the error dynamics matrix
    - $dt$ is the time step
    - $Q$ is the process noise covariance
    - $Ad_x$ is the Adjoint matrix of the state, for our case $Ad_x = R$
    - $R$ is the rotation matrix (state)
    - $P$ is the state covariance
    - $\omega$ is the gyroscope reading from the IMU

### Correction step 
- N = R * cov\[V\] * R.T
- S = H * P * H.T + N
- K = P * H.T * inv(S)

- Update state:
    - x = exp(K * (R * Y - b))
- Update Error covariance:
    - P = (I - K * H) * P * (I - K * H).T + K * N * K.T

## References
* http://arxiv.org/pdf/1904.09251
* https://github.com/UMich-CURLY-teaching/UMich-ROB-530-public/blob/main/slides/09_Invariant_EKF.pdf
