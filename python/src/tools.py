
import numpy as np
import matplotlib.pyplot as plt

def rmse(states1, states2, get_scalar=False):
    residuals = (np.array(states1) - np.array(states2)) ** 2
    rmse = np.sqrt(np.mean(residuals, axis=0))
    if get_scalar:
        np.mean(rmse)
    return rmse

def plot(zs, filtered=None, gts=None):
    zs = np.array(zs)
    plt.scatter(zs[:,0], zs[:,1], facecolors='none', edgecolors='gray')

    if filtered is not None:
        filtered = np.array(filtered)
        plt.plot(filtered[:,0], filtered[:,1], "r--")
    
    if gts is not None:
        gts = np.array(gts)
        plt.plot(gts[:,0], gts[:,1], "b--")
    plt.show()

def polar_to_cartesian(zs):
    def _to_cartesian(measurement):
        rho, theta, _ = measurement.reshape(-1,)
        
        px = rho * np.cos(theta)
        py = rho * np.sin(theta)
        
        xy = np.array([px, py]).reshape(-1, 1)
        return xy
    
    pos_buffers = []
    for z in zs:
        xy = _to_cartesian(z)
        pos_buffers.append(xy.reshape(-1,))
    pos_buffers = np.array(pos_buffers)
    return pos_buffers

