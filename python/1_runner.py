
from src.data_reader import DataReader
from src.kalman_filter import KalmanFilter, FusionKalmanFilter
from src.sensor import LaserSensor, RadarSensor
from src.tools import plot, rmse, polar_to_cartesian

import numpy as np

def kf_run(kalman_filter, zs, dts):
    filtered = []
    for i, (z, dt) in enumerate(zip(zs, dts)):
        if i == 0:
            kalman_filter.initialize(z)
        else:
            kalman_filter.predict(dt)
            kalman_filter.update(z)
        filtered.append(kalman_filter._x.reshape(-1,))
    return filtered


# 1. Init DataReader instance
data_reader = DataReader("input.txt", sensor_type="R")
zs = []
dts = []
gts = []
for (z, dt, truth) in zip(data_reader.generate_zs(), data_reader.generate_dt(), data_reader.generate_gt()):
    _, z = z
    _, dt = dt
    _, gt = truth
    
    zs.append(z)
    dts.append(dt)
    gts.append(gt)


# 3. Run radar kalman filter
radar_kf = KalmanFilter(RadarSensor(noise_scale_vector=[0.09, 0.0009, 0.09]),
                        q_scale=0.1)
filtered = kf_run(radar_kf, zs, dts)
zs = polar_to_cartesian(zs)
# plot(zs, filtered, gts)
# print(rmse(filtered, gts))


data_reader = DataReader("input.txt", sensor_type="L")
zs = []
dts = []
gts = []
for (z, dt, truth) in zip(data_reader.generate_zs(), data_reader.generate_dt(), data_reader.generate_gt()):
    _, z = z
    _, dt = dt
    _, gt = truth
    
    zs.append(z)
    dts.append(dt)
    gts.append(gt)

# 4. Run laser kalman filter
laser_kf = KalmanFilter(LaserSensor(noise_scale_vector=[0.0225, 0.0225]), 
                        q_scale=0.1)
filtered = kf_run(laser_kf, zs, dts)    
# plot(zs, filtered, gts)

# laser_kf = RadarKalmanFilter(q_scale=0.1)



data_reader = DataReader("input.txt")
zs = []
dts = []
gts = []
sensor_types = []
for (z, dt, truth) in zip(data_reader.generate_zs(), data_reader.generate_dt(), data_reader.generate_gt()):
    sensor_type, z = z
    _, dt = dt
    _, gt = truth
    
    zs.append(z)
    dts.append(dt)
    gts.append(gt)
    sensor_types.append(sensor_type)

print(gts)
print(sensor_types)

# 4. Run laser kalman filter
sensors = {"L" : LaserSensor(noise_scale_vector=[0.0225, 0.0225]),
           "R" : RadarSensor(noise_scale_vector=[0.09, 0.0009, 0.09])}
fusion_kf = FusionKalmanFilter(sensors, 0.1)

filtered = []
for i, (z, dt, sensor) in enumerate(zip(zs, dts, sensor_types)):
    print(dt, sensor)
    if i == 0:
        fusion_kf.initialize(z, sensor)
    else:
        fusion_kf.predict(dt)
        fusion_kf.update(z, sensor)
    filtered.append(fusion_kf._x.reshape(-1,))
print(rmse(filtered, gts))
# [.11, .11, 0.52, 0.52].

# filtered = fusion_kf_run(fusion_kf, zs, dts, sensor_types)
# # plot(zs, filtered, gts)
# 
# import matplotlib.pyplot as plt
# filtered = np.array(filtered)
# plt.plot(filtered[:,0], filtered[:,1], "r--")
# gts = np.array(gts)
# plt.plot(gts[:,0], gts[:,1], "b--")
# plt.show()

# [ 0.28271395  0.38665641  0.8200422   0.95842644]

