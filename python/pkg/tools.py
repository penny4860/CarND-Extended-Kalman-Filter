
import numpy as np


class Tools(object):
    def __init__(self):
        pass
    
    def CalculateRMSE(self, estimations, ground_truth):
        # Calculate the RMSE here.
        if np.array(estimations).shape != np.array(ground_truth).shape:
            raise ValueError("Invalid estimation or ground_truth data")
    
        # accumulate squared residuals
        ests = np.array(estimations)
        gts = np.array(ground_truth)

        residuals = (ests - gts) ** 2
        rmse = np.mean(residuals, axis=0)
        rmse = np.sqrt(rmse)

        return rmse;

    def CalculateJacobian(self, x_state):
        # Calculate a Jacobian here.
        Hj = np.zeros((3,4))

        # recover state parameters
        px, py, vx, vy = x_state
        
        # pre-compute a set of terms to avoid repeated calculation
        c1 = px*px+py*py;
        c2 = np.sqrt(c1);
        c3 = (c1*c2);

        # check division by zero
        if abs(c1) < 0.0001:
            raise ValueError("Function CalculateJacobian() has Error: Division by Zero")

        # compute the Jacobian matrix
        Hj[0,:] = [(px/c2), (py/c2), 0, 0]
        Hj[1,:] = [-(py/c1), (px/c1), 0, 0]
        Hj[2,:] = [py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3, 0, 0]
        return Hj;
            

