
import pandas as pd
import numpy as np


def load_data_camera(K_csv, C_csv):
    """Loads data from the specified path"""
    K = pd.read_csv(K_csv, header=None).to_numpy()
    # print(pd.read_csv(K_csv))
    C = pd.read_csv(C_csv, header=None).to_numpy()
    return K, C

def load_measurement(meas_csv):
    """Loads data from the specified path"""
    meas = pd.read_csv(meas_csv,header=None).to_numpy()
    return meas

def load_transformations(R_csv, T_csv):
    """Loads data from the specified path"""
    R = pd.read_csv(R_csv,header=None).to_numpy()
    T = pd.read_csv(T_csv,header=None).to_numpy()
    return R, T

class DataHandler:
    def __init__(self, data_dir, Q1_scale=10, Q2_scale=0, R1_scale = 1000, R2_scale=1000):
        # Camera 1

        """
        Initialize the data handler.

        Parameters
        ----------
        data_dir : str
            The directory of the data.
        Q1_scale : float, optional
            The scale of the covariance matrix of the first camera. Default is 10.
        Q2_scale : float, optional
            The scale of the covariance matrix of the second camera. Default is 0.
        R1_scale : float, optional
            The scale of the covariance matrix of the first measurement. Default is 1000.
        R2_scale : float, optional
            The scale of the covariance matrix of the second measurement. Default is 1000.
        """
        self.K1, self.C1 = load_data_camera(f'{data_dir}/Kf_1.csv', f'{data_dir}/C_1.csv')
        self.z1 = load_measurement(f'{data_dir}/z_1.csv')

        # Camera 2
        self.K2, self.C2 = load_data_camera(f'{data_dir}/Kf_2.csv', f'{data_dir}/C_2.csv')
        self.z2 = load_measurement(f'{data_dir}/z_2.csv')

        # load the transformations
        self.R, self.T = load_transformations(f'{data_dir}/R.csv', f'{data_dir}/T.csv')

        self.Q1 = np.eye(3) * Q1_scale
        self.Q2 = np.eye(3) * Q2_scale

        self.R1 = np.eye(2) * R1_scale
        self.R2 = np.eye(2) * R2_scale

    def h1_func(self, x):
        """
        Predict the measurement given the state.

        Parameters
        ----------
        x : numpy array of shape (3,)
            The state.

        Returns
        -------
        pred_z : numpy array of shape (2,)
            The predicted measurement.
        """
        pred_z = (1/x[2]) * self.K1 @ x[:2].reshape(-1,1) + self.C1
        return pred_z.reshape(-1,)

    def h2_func(self, x):
        """
        Predict the measurement given the state.

        Parameters
        ----------
        x : numpy array of shape (3,)
            The state.

        Returns
        -------
        pred_z : numpy array of shape (2,)
            The predicted measurement.
        """
        trans_P = (self.R.T@ x - self.R.T@self.T).reshape(-1,)
        # print(trans_P.shape)
        pred_z = (1/trans_P[2]) * self.K2 @ trans_P[:2].reshape(-1,1) + self.C2
        return pred_z.reshape(-1,)

    def process_func(self, x, Q):
        """
        Process function for the particle filter.

        Parameters
        ----------
        x : numpy array of shape (n,3)
            The states.
        Q : numpy array of shape (3,3)
            The process noise covariance.

        Returns
        -------
        x : numpy array of shape (n,3)
            The states after process update.
        """
        x = x.reshape(-1,3)
        # print(x.shape)
        return x + np.random.multivariate_normal(np.zeros(3), Q, size=x.shape[0])

    def h1_jacobian(self, x):
        """
        Compute the measurement Jacobian for the first camera.

        Parameters
        ----------
        x : numpy array of shape (3,)
            The state.

        Returns
        -------
        H1 : numpy array of shape (2,3)
            The measurement Jacobian for the first camera.
        """
        H1 = self.K1 @ np.array([[1/x[2], 0 , -x[0]/(x[2]**2)],[0, 1/x[2], -x[1]/(x[2]**2)]])
        return H1

    def h2_jacobian(self, x):
        """
        Compute the measurement Jacobian for the second camera.

        Parameters
        ----------
        x : numpy array of shape (3,)
            The state.

        Returns
        -------
        H2 : numpy array of shape (2,3)
            The measurement Jacobian for the second camera.
        """
        trans_P = (self.R.T@ x.reshape(-1,1) - self.R.T@self.T).reshape(-1,)
        H2 = self.K2 @ np.array([[1/trans_P[2], 0 , -trans_P[0]/(trans_P[2]**2)],[0, 1/trans_P[2], -trans_P[1]/(trans_P[2]**2)]]) @ self.R.T
        return H2