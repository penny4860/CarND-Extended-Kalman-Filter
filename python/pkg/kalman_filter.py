
import numpy as np

class KalmanFilter(object):
    
    def __init__(self):
        pass

    def Init(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x_ = x_in
        self.P_ = P_in
        self.F_ = F_in
        self.H_ = H_in
        self.R_ = R_in
        self.Q_ = Q_in
    
    def Predict(self):
        # predict the state
        self.x_ = np.dot(self.F_, self.x_)
        Ft = self.F_.T
        self.P_ = np.dot(np.dot(self.F_, self.P_), Ft) + self.Q_
        
    def Update(self, z):
        # update the state by using Kalman Filter equations
        z_pred = np.dot(self.H_, self.x_)
        y = z - z_pred;
        Ht = self.H_.T
        PHt = np.dot(self.P_, Ht)
        S = np.dot(self.H_, PHt) + self.R_
        Si = np.linalg.inv(S)
        K = np.dot(PHt, Si)

        # new estimate
        self.x_ = self.x_ + np.dot(K, y)
        I = np.identity(len(self.x_))
        self.P_ = (I - np.dot(K, self.H_)) * self.P_

    
    def UpdateEKF(self, z):
        # update the state by using Extended Kalman Filter equations

        # convert radar measurements from cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot).
        z_pred = RadarCartesianToPolar(self.x_)
        y = z - z_pred

        # normalize the angle between -pi to pi
        while y[1,0] > np.pi:
            y[1,0] -= 2*np.pi
        
        while y[1,0] < -np.pi:
            y[1,0] += 2*np.pi

        # following is exact the same as in the function of KalmanFilter::Update()
        Ht = self.H_.T
        PHt = np.dot(self.P_, Ht)
        S = np.dot(self.H_, PHt) + self.R_
        Si = np.linalg.inv(S)
        K = np.dot(PHt, Si)
        
        # new estimate
        self.x_ = self.x_ + np.dot(K, y)
        I = np.identity(len(self.x_))
        self.P_ = (I - np.dot(K, self.H_)) * self.P_



def RadarCartesianToPolar(x_state):
    # cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot) coordinates
    px, py, vx, vy = x_state;

    rho = np.sqrt(px*px + py*py);
    phi = np.arctan2(py, px);  

    # if rho is very small, set it to 0.0001 to avoid division by 0 in computing rho_dot
    if rho < 0.000001:
        rho = 0.000001

    rho_dot = (px * vx + py * vy) / rho;

    z_pred = np.array([rho, phi, rho_dot]).reshape(-1,1)
    return z_pred;



