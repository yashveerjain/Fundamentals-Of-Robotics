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
    - g is the gravity vector
    - V is the noise
    - $\omega_g$ is the gyro noise
    - $\omega$ is the gyroscope reading


## RIEKF Has 2 step implementation

1. Prediction step
    - Predict the next state
        - $\dot R = R * (\omega)$
    - Predict the next state covariance
        - P_dot = A * P * A.T + Ad_x * Q * Ad_x.T 
2. Correction step 
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
