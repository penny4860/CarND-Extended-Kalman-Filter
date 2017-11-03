
from pkg.measurement_package import MeasurementPackage
from pkg.FusionEKF import FusionEKF
from pkg.tools import Tools

import numpy as np

filename = "..//data//obj_pose-laser-radar-synthetic-input.txt"
f = open(filename, 'r')
lines = f.readlines()

estimations = []
ground_truth = []

# Create a Kalman Filter instance
fusionEKF = FusionEKF()
tools_ = Tools()

for line in lines:
    line = str(line).split()
    sensor = line[0]

    meas_package = MeasurementPackage()
    if sensor == "R":
        ro, theta, ro_dot, timestamp = line[1:5]
        x_gt, y_gt, vx_gt, vy_gt = line[5:9]
        
        print(ro, theta, ro_dot)
        
        meas_package.sensor_type_ = "R";
        meas_package.raw_measurements_ = np.array([ro, theta, ro_dot]).reshape(-1,1)
        meas_package.timestamp_ = timestamp;

    else:
        px, py, timestamp = line[1:4]
        x_gt, y_gt, vx_gt, vy_gt = line[4:8]

        print(px, py)
        
        meas_package.sensor_type_ = "L";
        meas_package.raw_measurements_ = np.array([px, py]).reshape(-1,1)
        meas_package.timestamp_ = timestamp;
    
    gt_values = np.array([x_gt, y_gt, vx_gt, vy_gt])
    ground_truth.append(gt_values)
    
    
    # Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);
# 
    # Push the current estimated x,y positon from the Kalman filter's state vector
    print(fusionEKF.ekf_.x_)
    px, py, vx, vy = fusionEKF.ekf_.x_
    estimate = np.array([px, py, vx, vy]).reshape(-1,1)
    estimations.append(estimate)
    
    RMSE = tools.CalculateRMSE(estimations, ground_truth)
    print("RMSE = {}".format(RMSE))
    
#     cout << count << "    RMSE:    "<< RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << "\n";
#     # //        498    RMSE:    0.0973178, 0.0854597, 0.451267, 0.439935
#     # //        499    RMSE:    0.0972256, 0.0853761, 0.450855, 0.439588
#     count++;

    
    
# print(ground_truth)
    
