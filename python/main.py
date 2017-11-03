
from pkg.measurement_package import MeasurementPackage
from pkg.FusionEKF import FusionEKF
from pkg.tools import Tools

import numpy as np


def main():
    filename = "..//data//obj_pose-laser-radar-synthetic-input.txt"
    f = open(filename, 'r')
    lines = f.readlines()
    
    estimations = []
    ground_truth = []
    measurments = []
    
    # Create a Kalman Filter instance
    fusionEKF = FusionEKF()
    tools_ = Tools()
    count = 0
    
    for line in lines:
        line = str(line).split()
        sensor = line[0]
    
        meas_package = MeasurementPackage()
        if sensor == "R":
            ro, theta, ro_dot, timestamp = line[1:5]
            ro = float(ro)
            theta = float(theta)
            ro_dot = float(ro_dot)
            timestamp = float(timestamp)
    
            x_gt, y_gt, vx_gt, vy_gt = line[5:9]
            x_gt = float(x_gt)
            y_gt = float(y_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
            
            meas_package.sensor_type_ = "R";
            meas_package.raw_measurements_ = np.array([ro, theta, ro_dot]).reshape(-1,1)
            meas_package.timestamp_ = timestamp;
            
            px, py = _polar_to_xy(ro, theta)
            measurments.append((px, py))
    
        else:
            px, py, timestamp = line[1:4]
            px = float(px)
            py = float(py)
            timestamp = float(timestamp)
            
            x_gt, y_gt, vx_gt, vy_gt = line[4:8]
            x_gt = float(x_gt)
            y_gt = float(y_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
            
            meas_package.sensor_type_ = "L";
            meas_package.raw_measurements_ = np.array([px, py]).reshape(-1,1)
            meas_package.timestamp_ = timestamp;

            measurments.append((px, py))
        
        gt_values = np.array([x_gt, y_gt, vx_gt, vy_gt])
        ground_truth.append(gt_values)
        
        # Call ProcessMeasurment(meas_package) for Kalman filter
        fusionEKF.ProcessMeasurement(meas_package);

        # Push the current estimated x,y positon from the Kalman filter's state vector
        px, py, vx, vy = fusionEKF.ekf_.x_[0,0], fusionEKF.ekf_.x_[1,0], fusionEKF.ekf_.x_[2,0], fusionEKF.ekf_.x_[3,0]
        
        estimate = np.array([px, py, vx, vy])
        estimations.append(estimate)
        
        RMSE = tools_.CalculateRMSE(estimations, ground_truth)
        print("{} RMSE: {}, {}, {}, {}".format(count, RMSE[0], RMSE[1], RMSE[2], RMSE[3]))
        count += 1

    return measurments, estimations, ground_truth


def _polar_to_xy(rho, theta):
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    return x, y

if __name__ == "__main__":
    zs, xs, gts = main()
    zs = np.array(zs)
    xs = np.array(xs)
    gts = np.array(gts)

    import matplotlib.pyplot as plt
    plt.scatter(zs[:, 0], zs[:, 1], facecolors='none', edgecolors='gray', label='measurements')
    plt.scatter(gts[:, 0], gts[:, 1], facecolors='none', edgecolors='red', label='Ground Truth')
    plt.plot(xs[:, 0], xs[:, 1], "b--", label='Extended Kalman Filtered')
    plt.legend()
    plt.show()



    
    
    
    
    




