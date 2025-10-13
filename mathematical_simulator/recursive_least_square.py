import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# define the true parameters that we want to estimate
import math

class RecursiveLeastSquares:
    def __init__(self, s0, P0, R):
        """
        Initialize the Recursive Least Squares (RLS) algorithm.
        
        Parameters:
        x0 : np.ndarray
            Initial state estimate.
        P0 : np.ndarray
            Initial error covariance matrix.
        R : np.ndarray
            Measurement noise covariance matrix.
        """
        self.s0 = s0
        self.P0 = P0
        self.R = R
        #the list where the estimates of the slip will be stored
        self.estimates = []
        self.estimates.append(s0)
        # the list where the estimation error covariance matrices will be stored Pk
        self.estimationErrorCovarianceMatrices = []
        self.estimationErrorCovarianceMatrices.append(P0)
        #the list where the Kalman gain matrices will be stored Kk
        self.gainMatrices = []
        # the list where the estimation errors will be stored ek
        self.errors = []

        self.theta_diff = []
        self.angular_vel_z = []

        # the distance between the wheels
        self.L = 0.5

        # this variable is used to track the current time step k of the estimator 
        # after every time step arrives, this variables increases for one 
        # in this way, we can track the number of variblaes
        self.previousTimeStep=0

    def predict_sim(self, theta_noised, theta_previous_noised, vel_right, vel_left, delta_t):
            """
            First calculating the theta difference and the angular velocity 
            """
            theta_diff= np.array([theta_noised - theta_previous_noised])
            #Calculating the angular velocity in z axis
            angular_vel_z = (vel_right - vel_left) / self.L
            C = np.array([delta_t * angular_vel_z])
            #Calculating L matrix and its inverse
            L_matrix = self.R + np.matmul(C, np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], C.T))
            L_matrix_inverse = np.linalg.inv(L_matrix)            

            #Calculating the Kalman gain matrix
            gain_matrix = np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], np.matmul(C.T, L_matrix_inverse))

            #Calculating the estimation error(correction term (yk -Cxk))
            error = (C-theta_diff) - np.matmul(C, self.estimates[self.previousTimeStep])

            #Calculating the new estimate
            estimate = self.estimates[self.previousTimeStep] + np.matmul(gain_matrix, error)

            #Calculating the new estimation error covariance matrix
            ImKc = np.eye(np.size(self.s0), np.size(self.s0)) - np.matmul(gain_matrix, C)
            estimationErrorCovarianceMatrix = np.matmul(ImKc, self.estimationErrorCovarianceMatrices[self.previousTimeStep])

            #Storing the results
            self.estimates.append(estimate)
            self.estimationErrorCovarianceMatrices.append(estimationErrorCovarianceMatrix)
            self.gainMatrices.append(gain_matrix)
            self.errors.append(error)
            self.theta_diff.append(theta_diff)
            self.angular_vel_z.append(angular_vel_z)

            # increase the time step
            self.previousTimeStep = self.previousTimeStep + 1
        
    #writing method to estimate slip from experiment data
    def predict_exp(self, measurement_value, C_matrix):
         L_matrix = self.R + np.matmul(C_matrix, np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], C_matrix.T))
         L_matrix_inverse = np.linalg.inv(L_matrix)
         #Calculating the Kalman gain matrix
         gain_matrix = np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], np.matmul(C_matrix.T, L_matrix_inverse))
         #Calculating the estimation error(correction term (yk -Cxk))
         error = measurement_value - np.matmul(C_matrix, self.estimates[self.previousTimeStep])
         #Calculating the new estimate
         estimate = self.estimates[self.previousTimeStep] + np.matmul(gain_matrix, error)

         ImKc = np.eye(np.size(self.s0), np.size(self.s0)) - np.matmul(gain_matrix, C_matrix)
         estimationErrorCovarianceMatrix = np.matmul(ImKc, self.estimationErrorCovarianceMatrices[self.previousTimeStep])

            #Storing the results
         self.estimates.append(estimate)
         self.estimationErrorCovarianceMatrices.append(estimationErrorCovarianceMatrix)
         self.gainMatrices.append(gain_matrix)
         self.errors.append(error)


         self.previousTimeStep = self.previousTimeStep + 1