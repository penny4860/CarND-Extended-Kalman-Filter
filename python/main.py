
from pkg.measurement_package import MeasurementPackage

import numpy as np

filename = "..//data//obj_pose-laser-radar-synthetic-input.txt"
f = open(filename, 'r')
lines = f.readlines()

for line in lines:
    line = str(line).split()
    sensor = line[0]

    meas_package = MeasurementPackage()
    if sensor == "R":
        ro, theta, ro_dot, timestamp = line[1:5]
        print(ro, theta, ro_dot)
        
        meas_package.sensor_type_ = "R";
        meas_package.raw_measurements_ = np.array([ro, theta, ro_dot]).reshape(-1,1)
        meas_package.timestamp_ = timestamp;

    else:
        px, py, timestamp = line[1:4]
        print(px, py)
        
        meas_package.sensor_type_ = "L";
        meas_package.raw_measurements_ = np.array([px, py]).reshape(-1,1)
        meas_package.timestamp_ = timestamp;
        
    
