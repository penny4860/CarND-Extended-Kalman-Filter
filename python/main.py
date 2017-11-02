
from pkg.measurement_package import MeasurementPackage

import numpy as np

filename = "..//data//obj_pose-laser-radar-synthetic-input.txt"
f = open(filename, 'r')
lines = f.readlines()

estimations = []
ground_truth = []

# Create a Kalman Filter instance
# fusionEKF = FusionEKF()

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
    
    
#     # Call ProcessMeasurment(meas_package) for Kalman filter
#     fusionEKF.ProcessMeasurement(meas_package);
# 
#     # Push the current estimated x,y positon from the Kalman filter's state vector
#     VectorXd estimate(4);
# 
#     double p_x = fusionEKF.ekf_.x_(0);
#     double p_y = fusionEKF.ekf_.x_(1);
#     double v1  = fusionEKF.ekf_.x_(2);
#     double v2 = fusionEKF.ekf_.x_(3);
# 
#     estimate(0) = p_x;
#     estimate(1) = p_y;
#     estimate(2) = v1;
#     estimate(3) = v2;
# 
#     estimations.push_back(estimate);
# 
#     VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
# 
#     cout << count << "    RMSE:    "<< RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << "\n";
#     # //        498    RMSE:    0.0973178, 0.0854597, 0.451267, 0.439935
#     # //        499    RMSE:    0.0972256, 0.0853761, 0.450855, 0.439588
#     count++;

    
    
print(ground_truth)
    
