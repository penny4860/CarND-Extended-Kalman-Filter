
import numpy as np


class KalmanFilter(object):
    """2D position Kalman Filter
    
    """
    _n_states = 4

    def __init__(self, sensor, q_scale=9.0):

        self._sensor = sensor

        self._q_scale = q_scale
        
        self._x = np.zeros((self._n_states, 1))
        self._P = np.zeros((self._n_states, self._n_states))

        self._P[0,0] = 1
        self._P[1,1] = 1
        self._P[2,2] = 1000
        self._P[3,3] = 1000

    def initialize(self, z):
        self._x = self._sensor.state_init(z)

    def predict(self, dt=1):
        F = self._get_f(dt)
        Q = self._get_q()
        
        self._x = np.dot(F, self._x)
        self._P = np.dot(np.dot(F, self._P), F.T) + Q

    def update(self, z):
        R = self._sensor.noise_covariance_matrix()
        z = z.reshape(-1,1)
        
        y = self._sensor.residual(self._x, z)
        H = self._sensor.measurement_matrix(self._x)
        
        S = R + self._dot3(H, self._P, H.T)
        K = self._dot3(self._P, H.T, np.linalg.inv(S))
        self._x = self._x + np.dot(K, y)
        I = np.identity(self._n_states)
        self._P = np.dot(I - np.dot(K, H), self._P)

    def _dot3(self, A, B, C):
        return np.dot(np.dot(A, B), C)

    def _get_f(self, dt=1):
        F = np.array([[1,0,dt,0],
                      [0,1,0,dt],
                      [0,0,1,0],
                      [0,0,0,1]])
        return F

    def _get_q(self, dt=1):
        Q = np.zeros((self._n_states, self._n_states))
        
        # std(position) * std(position)
        Q[0,0] = 0.25 * dt**4 * self._q_scale
        Q[1,1] = 0.25 * dt**4 * self._q_scale

        # std(position) * std(velocity)
        Q[0,2] = 0.5 * dt**3 * self._q_scale
        Q[2,0] = 0.5 * dt**3 * self._q_scale
        Q[3,1] = 0.5 * dt**3 * self._q_scale
        Q[1,3] = 0.5 * dt**3 * self._q_scale

        # std(velocity) * std(velocity)
        Q[2,2] = dt**2 * self._q_scale
        Q[3,3] = dt**2 * self._q_scale
        
        return Q


class FusionKalmanFilter(KalmanFilter):
    def __init__(self, sensors, q_scale=9.0):

        self._sensors = sensors

        self._q_scale = q_scale
        
        self._x = np.zeros((self._n_states, 1))
        self._P = np.zeros((self._n_states, self._n_states))

        self._P[0,0] = 1
        self._P[1,1] = 1
        self._P[2,2] = 1000
        self._P[3,3] = 1000
        
    def initialize(self, z, sensor_type):
        sensor = self._get_sensor(sensor_type)
        self._x = sensor.state_init(z)

    def update(self, z, sensor_type):
        sensor = self._get_sensor(sensor_type)
        
        R = sensor.noise_covariance_matrix()
        z = z.reshape(-1,1)
        
        y = sensor.residual(self._x, z)
        H = sensor.measurement_matrix(self._x)
        
        S = R + self._dot3(H, self._P, H.T)
        K = self._dot3(self._P, H.T, np.linalg.inv(S))
        self._x = self._x + np.dot(K, y)
        I = np.identity(self._n_states)
        self._P = np.dot(I - np.dot(K, H), self._P)

    def _get_sensor(self, sensor_type):
        """
        # Args
            sensor_type : str
        
        # Returns
            _Sensor instance
        """
        return self._sensors[sensor_type]
