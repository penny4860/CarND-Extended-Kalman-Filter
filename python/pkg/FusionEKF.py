
from pkg.tools import Tools
from pkg.kalman_filter import KalmanFilter

import numpy as np

class FusionEKF(object):
    
    def __init__(self):
        self.is_initialized_ = False
        self.previous_timestamp_ = 0

        # measurement covariance matrices
        self.R_laser_ = np.array([[0.0225, 0],
                                  [0, 0.0225]])
        self.R_radar_ = np.array([[0.09, 0, 0],
                                  [0, 0.0009, 0],
                                  [0, 0, 0.09]])

        # measurement  matrices
        self.H_laser_ = np.array([[1, 0, 0, 0,],
                                  [0, 1, 0, 0,]])
        self.H_jacobian = np.zeros((3, 4))

        # initialize the kalman filter variables
        self.ekf_ = KalmanFilter()
        self.ekf_.P_ = np.array([[1, 0, 0, 0,],
                                 [0, 1, 0, 0,],
                                 [0, 0, 1000, 0,],
                                 [0, 0, 0, 1000,]])

        self.ekf_.F_ = np.array([[1, 0, 1, 0,],
                                 [0, 1, 0, 1,],
                                 [0, 0, 1, 0,],
                                 [0, 0, 0, 1,]], dtype=float)

        # set measurement noises
        self.noise_ax = 9
        self.noise_ay = 9

    def ProcessMeasurement(self, measurement_pack):

        if self.is_initialized_ == False:
            
            if measurement_pack.sensor_type_ == "R":
                ro, phi, _ = measurement_pack.raw_measurements_[0:2]
                px = ro * np.cos(phi)
                py = ro * np.sin(phi)
            elif measurement_pack.sensor_type_ == "L":
                px, py = measurement_pack.raw_measurements_[0:2, 0]
            else:
                raise 
            self.ekf_.x_ = np.array([px, py, 0, 0]).reshape(-1,1)
            self.previous_timestamp_ = measurement_pack.timestamp_
            self.is_initialized_ = True
            return None
        
        
        # *****************************************************************************
        #  Prediction
        # *****************************************************************************
        # Time is measured in seconds.
        dt = (measurement_pack.timestamp_ - self.previous_timestamp_) / 1000000.0
        self.previous_timestamp_ = measurement_pack.timestamp_;

        dt_2 = dt**2
        dt_3 = dt**3
        dt_4 = dt**4

        # Update the state transition matrix F according to the new elapsed time.
        self.ekf_.F_[0, 2] = dt
        self.ekf_.F_[1, 3] = dt

        # set the process covariance matrix Q
        self.ekf_.Q_ = np.array([[dt_4/4*self.noise_ax,   0,                dt_3/2*self.noise_ax,  0],
                                 [0,                 dt_4/4*self.noise_ay,  0,                dt_3/2*self.noise_ay],
                                 [dt_3/2*self.noise_ax,   0,                dt_2*self.noise_ax,    0],
                                 [0,                 dt_3/2*self.noise_ay,  0,                dt_2*self.noise_ay]])

        self.ekf_.Predict()

        # *****************************************************************************
        #  Update
        # *****************************************************************************
#   /**
#      * Use the sensor type to perform the update step.
#      * Update the state and covariance matrices.
#    */
        if measurement_pack.sensor_type_ == "R":
            # Radar updates
            tools_ = Tools()
            
            H_jacobian = tools_.CalculateJacobian(self.ekf_.x_)
            self.ekf_.H_ = H_jacobian
            self.ekf_.R_ = self.R_radar_
            self.ekf_.UpdateEKF(measurement_pack.raw_measurements_)
            
        elif measurement_pack.sensor_type_ == "L":
            # Laser updates
            self.ekf_.H_ = self.H_laser_
            self.ekf_.R_ = self.R_laser_
            self.ekf_.Update(measurement_pack.raw_measurements_)
