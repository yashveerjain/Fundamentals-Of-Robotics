import numpy as np
from scipy.linalg import block_diag
from copy import deepcopy, copy
import rospy

from system.RobotState import RobotState
from utils.Landmark import LandmarkList
from utils.utils import wrap2Pi

class EKF:

    def __init__(self, system, init):
        # EKF Construct an instance of this class
        # Inputs:
        #   system: system and noise models
        #   init:   initial state mean and covariance
        self.gfun = system.gfun  # motion model
        self.hfun = system.hfun  # measurement model
        self.Gfun = init.Gfun  # Jocabian of motion model
        self.Vfun = init.Vfun  # Jocabian of motion model
        self.Hfun = init.Hfun  # Jocabian of measurement model
        self.M = system.M # motion noise covariance
        self.Q = system.Q # measurement noise covariance

        self.state_ = RobotState()

        # init state
        self.state_.setState(init.mu)
        self.state_.setCovariance(init.Sigma)


    ## Do prediction and set state in RobotState()
    def prediction(self, u):

        # prior belief
        X = self.state_.getState()
        P = self.state_.getCovariance()

        ###############################################################################
        # TODO: Implement the prediction step for EKF                                 #
        # Hint: save your predicted state and cov as X_pred and P_pred                #
        ###############################################################################
        X_pred = self.gfun(X, u)
        P_pred = self.Gfun(X)@P@self.Gfun(X).T +  self.Vfun(X, u)@self.M(u)@self.Vfun(X, u).T

        ###############################################################################
        #                         END OF YOUR CODE                                    #
        ###############################################################################

        self.state_.setTime(rospy.Time.now())
        self.state_.setState(X_pred)
        self.state_.setCovariance(P_pred)


    def correction(self, z, landmarks):
        # EKF correction step
        #
        # Inputs:
        #   z:  measurement
        X_predict = self.state_.getState()
        P_predict = self.state_.getCovariance()
        
        landmark1 = landmarks.getLandmark(z[2].astype(int))
        landmark2 = landmarks.getLandmark(z[5].astype(int))

        ###############################################################################
        # TODO: Implement the correction step for EKF                                 #
        # Hint: save your corrected state and cov as X and P                          #
        # Hint: you can use landmark1.getPosition()[0] to get the x position of 1st   #
        #       landmark, and landmark1.getPosition()[1] to get its y position        #
        ###############################################################################

        H1 = self.Hfun(landmark1.getPosition()[0], landmark1.getPosition()[1], X_predict, z)
        error = z - self.hfun(landmark1.getPosition()[0], landmark1.getPosition()[1], X_predict)

        S = H1@P_predict@H1.T + self.Q

        K = P_predict@H1.T@np.linalg.inv(S)

        X = X_predict + K@error
        P = (np.eye(3) - K@H1)@P_predict 
        
        ###############################################################################
        #                         END OF YOUR CODE                                    #
        ###############################################################################

        self.state_.setTime(rospy.Time.now())
        self.state_.setState(X)
        self.state_.setCovariance(P)

    def getH(self, landmark, X):
        H = np.zeros((2,3))
        y_diff = landmark.getPosition()[1] - X[1]
        x_diff = landmark.getPosition()[0] - X[0] 
        sqrt_meas = np.sqrt(x_diff**2 + y_diff**2)

        H[0,0] = -1/np.square(sqrt_meas)
        H[0,1] = np.square(x_diff/np.square(sqrt_meas))*y_diff
        H[0,2] = -1

        H[1,0] = -x_diff/sqrt_meas
        H[1,1] = -y_diff/sqrt_meas
        H[1,2] = 0

        return H
    

    def getState(self):
        return deepcopy(self.state_)

    def setState(self, state):
        self.state_ = state