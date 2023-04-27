# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

#to tune parameters easier
# def matrix1(q1, q2, q3, q4):
#     return np.matrix([[q1, 0,  0,  q2, 0,  0 ],
#                       [0,  q1, 0,  0,  q2, 0 ],
#                       [0,  0,  q1, 0,  0,  q2],
#                       [q3, 0,  0,  q4, 0,  0 ],
#                       [0,  q3, 0,  0,  q4, 0 ],
#                       [0,  0,  q3, 0,  0,  q4]])


class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        # return matrix1(1, 0.05, 0, 1)
        dt = params.dt
        # system matrix
        #6*6 matrix main diagolnal 1 and 3 dt
        return np.matrix([[1,0,0,dt,0,0],
                          [0,1,0,0,dt,0],
                          [0,0,1,0,0,dt],
                          [0,0,0,1,0,0],
                          [0,0,0,0,1,0],
                          [0,0,0,0,0,1]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        # return matrix1(0.01, 0.001, 0.001, 0.01)
        q1 = (pow(params.dt,3) / 3) *params.q
        q2 = (pow(params.dt,2) / 2) *params.q
        q3 = params.dt *params.q
        # process noise covariance Q 6*6
        return np.matrix([[q1,0,0,q2,0,0],
                          [0,q1,0,0,q2,0],
                          [0,0,q1,0,0,q2],
                          [q2,0,0,q3,0,0],
                          [0,q2,0,0,q3,0],
                          [0,0,q2,0,0,q3]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        #track has x,P in it
        x = self.F() * track.x
        P = self.F() * track.P * self.F().transpose() + self.Q()
        
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        #measure matrix
        H = meas.sensor.get_H(track.x)
        #gamma = z - H*x # residual
        gamma = self.gamma(track, meas) #residual

        S = self.S(track, meas, H)
        K = track.P*H.transpose()*np.linalg.inv(S) #kalman gain

        x = track.x + K * gamma
        I = np.identity(params.dim_state)
        P = (I -K*H)*track.P

        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        #gamma = z - H*x # residual
        return meas.z - meas.sensor.get_hx(track.x)
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        #S=H*P*HT+R
        return H * track.P * H.transpose() + meas.R
        
        ############
        # END student code
        ############ 