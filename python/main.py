
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

    if count == 1:
        print("sensor", meas_package.sensor_type_)
        print("measurement", meas_package.raw_measurements_)
        print("timestamp", meas_package.timestamp_)
    
    
    gt_values = np.array([x_gt, y_gt, vx_gt, vy_gt])
    ground_truth.append(gt_values)
    
    
    # Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);
# 
    # Push the current estimated x,y positon from the Kalman filter's state vector
    px, py, vx, vy = fusionEKF.ekf_.x_[0,0], fusionEKF.ekf_.x_[1,0], fusionEKF.ekf_.x_[2,0], fusionEKF.ekf_.x_[3,0]
    
    estimate = np.array([px, py, vx, vy])
    estimations.append(estimate)
    
    RMSE = tools_.CalculateRMSE(estimations, ground_truth)
    #print("{} RMSE: {}, {}, {}, {}".format(count, RMSE[0], RMSE[1], RMSE[2], RMSE[3]))
    if count == 1:
        print("{}-th:     px: {}, py: {}, vx: {}, vy: {}".format(count, px, py, vx, vy))
        print("=====================================================================================")
    
    count += 1
    
    if count == 2:
        break
    
    
#     cout << count << "    RMSE:    "<< RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << "\n";
#     # //        498    RMSE:    0.0973178, 0.0854597, 0.451267, 0.439935
#     # //        499    RMSE:    0.0972256, 0.0853761, 0.450855, 0.439588
#     count++;

    
    
# print(ground_truth)
    
