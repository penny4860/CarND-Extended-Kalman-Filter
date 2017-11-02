
import numpy as np


class SensorReader(object):
    def __init__(self):
        pass
    
    def get_z(self, single_time_data, sensor_type):
        if sensor_type == "R":
            z = np.array(single_time_data[1:4]).astype(float)
        elif sensor_type == "L":
            z = np.array(single_time_data[1:3]).astype(float)
        return z

    def get_time(self, single_time_data, sensor_type):
        if sensor_type == "R":
            t = int(single_time_data[4])
        elif sensor_type == "L":
            t = int(single_time_data[3])
        return t

    def get_gt(self, single_time_data, sensor_type):
        if sensor_type == "R":
            gt = np.array(single_time_data[5:9]).astype(float)
        elif sensor_type == "L":
            gt = np.array(single_time_data[4:8]).astype(float)
        return gt


class DataReader(object):
    
    _reader = SensorReader()
    
    def __init__(self, filename="input.txt", sensor_type="all"):
        """
        # Args
            filename : str
            sensor_type : str
                "all", "L", "R"
        """
        self._dataset = self._read(filename)
        if sensor_type == "all":
            self._sensor_types = ["L", "R"]
        else:
            self._sensor_types = [sensor_type]
    
    def _read(self, filename):
        f = open(filename, 'r')
        lines = f.readlines()
        
        dataset = []
        for single_data in lines:
            dataset.append(str(single_data).split())
        return dataset
    
    def _get_sensor_type(self, single_data):
        return single_data[0]
    
    def _is_valid_sensor(self, sensor):
        if sensor in self._sensor_types:
            return True
        else:
            return False

    def generate_zs(self):
        """
        Returns
            (sensor, z)
                Radar : ("R", (rho, pi, v_rho))
                Ladar : ("L", ((px, py))
        """
        for single_data in self._dataset:
            sensor = self._get_sensor_type(single_data)
            if self._is_valid_sensor(sensor):
                z = self._reader.get_z(single_data, sensor)
                yield (sensor, z)

    def generate_time(self):
        """Generate delta T in mili-second unit"""
        for single_data in self._dataset:
            sensor = self._get_sensor_type(single_data)
            if self._is_valid_sensor(sensor):
                t = self._reader.get_time(single_data, sensor)
                yield (sensor, t)

    def generate_dt(self):
        """Generate delta T in second unit"""
        
        # 1. Get time stamp
        time_stamps = np.array([t for _, t in self.generate_time()]).astype(float)
        dts = np.zeros_like(time_stamps)
        dts[1:] = (time_stamps[1:] - time_stamps[:-1]) * 10**-6

        for single_data, dt in zip(self._dataset, dts):
            sensor = self._get_sensor_type(single_data)
            yield (sensor, dt)
            
    def generate_gt(self):
        """
        Generate (px, py, vx, vy)
        """
        for single_data in self._dataset:
            sensor = self._get_sensor_type(single_data)
            if self._is_valid_sensor(sensor):
                gt = self._reader.get_gt(single_data, sensor)
                yield (sensor, gt)


if __name__ == "__main__":        
    dataset = DataReader(filename="..//input.txt", sensor_type="L")
    for sensor, z in dataset.generate_zs():
        print(z)



