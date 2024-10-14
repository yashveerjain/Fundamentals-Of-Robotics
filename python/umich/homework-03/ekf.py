import numpy as np


def EKF(P, cov, H, Q, R, z, pred_z):
    """
    Implements the Extended Kalman filter (EKF) correction step.

    Parameters
    ----------
    P : array_like
        The predicted state.
    cov : array_like
        The predicted state covariance.
    H : array_like
        The measurement model Jacobian.
    Q : array_like
        The process noise covariance.
    R : array_like
        The measurement noise covariance.
    z : array_like
        The measurement.
    pred_z : array_like
        The predicted measurement.

    Returns
    -------
    P : array_like
        The corrected state.
    cov : array_like
        The corrected state covariance.
    """
    # getting next predicted state and covariance
    pred_P = P.reshape(-1,1)
    pred_cov = cov + Q

    # getting next predicted measurement and subtracting it from measurement
    V = z.reshape(-1,1) - pred_z.reshape(-1,1)

    # getting measurement Jacobian
    S =  H @ pred_cov @ H.T + R

    # Kalman gain
    K = pred_cov @ H.T @ np.linalg.inv(S)
    
    # correct the predicted state statistics
    P = pred_P + K @ V
    cov = (np.eye(3) - K @ H) @ pred_cov @ (np.eye(3) - K @ H).T + K @ R @ K.T

    return P, cov


