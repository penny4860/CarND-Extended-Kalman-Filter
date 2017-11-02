
import numpy as np

class _Sensor(object):
    
    def __init__(self, noise_scale_vector):
        self._r_scale = noise_scale_vector

    def state_init(self, z):
        pass
    
    def residual(self, x, z):
        pass
    
    def measurement_matrix(self):
        pass

    def noise_covariance_matrix(self):
        n_measurements = len(self._r_scale)
        R = np.zeros((n_measurements, n_measurements))
        for i, r in enumerate(self._r_scale):
            R[i,i] = r
        return R

class LaserSensor(_Sensor):
    
    def state_init(self, z):
        z = z.reshape(-1,1)
        x = np.zeros((4,1))
        x[:2, 0] = z[:, 0]
        return x
    
    def residual(self, x, z):
        H = self.measurement_matrix(x)
        z = z.reshape(-1,1)
        y = z - np.dot(H, x)
        return y
    
    def measurement_matrix(self, x):
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        return H

class RadarSensor(_Sensor):
    
    _EPSILON = 1e-4
    
    def state_init(self, z):
        rho, theta, _ = z.reshape(-1,)
        px = rho * np.cos(theta)
        py = rho * np.sin(theta)
        x = np.zeros((4,1))
        x[:2, 0] = (px, py)
        return x
    
    def residual(self, x, z):
        z = z.reshape(-1,1)
        hx = self._to_measurement(x)
        y = z - hx
        y[1, 0] = self._norm_angle(y[1, 0])
        return y
    
    def measurement_matrix(self, x):
        # recover state parameters
        px, py, vx, vy = x

        c1 = px*px + py*py
        c2 = np.sqrt(c1)
        c3 = c2**3

        # check division by zero
        if c1 < self._EPSILON:
            raise ValueError("CalculateJacobian () - Error - Division by Zero")

        H_jacobian = np.array([[(px/c2), (py/c2), 0, 0,],
                               [-(py/c1), (px/c1), 0, 0,],
                               [py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2]], dtype='float')
        return H_jacobian

    def _to_measurement(self, x):
        px, py, vx, vy = x.reshape(-1,)
        
        # Todo : divide by zero
        rho = np.sqrt(px**2 + py**2)
        theta = np.arctan2(py, px)
        
        velocity = (px*vx + py*vy) / max(rho, self._EPSILON)
        
        hx = np.array([rho, theta, velocity]).reshape(-1,1)
        return hx

    def _norm_angle(self, theta):
        # theta range to [-pi, +pi]
        while theta > np.pi or theta < -np.pi:
            if theta > np.pi:
                theta = theta - 2*np.pi
            elif theta < -np.pi:
                theta = theta + 2*np.pi
        return theta
