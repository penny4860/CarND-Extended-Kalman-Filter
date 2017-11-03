// reading a text file
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

int main () {
  string line;
  ifstream myfile ("data/obj_pose-laser-radar-synthetic-input.txt");

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  int count = 0;

  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
		std::stringstream linestream(line);
		std::string         data;

		string sensor;
		std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).
		sensor = line[0];

		// Read the integers using the operator >>
        MeasurementPackage meas_package;
        long long timestamp;
        float x_gt, y_gt, vx_gt, vy_gt;

    	if (sensor.compare("R") == 0)
    	{
    		float ro;
    		float theta;
    		float ro_dot;

    		linestream >> ro >> theta >> ro_dot >> timestamp >> x_gt >> y_gt >> vx_gt >> vy_gt;
//        	cout << "sensor=" << sensor << " rho=" <<  ro <<
//        			" theta=" << theta << " vel=" << ro_dot <<
//					" time=" << timestamp <<
//					" x_gt=" << x_gt << " y_gt=" << y_gt << " vx_gt=" << vx_gt << " vy_gt=" << vy_gt << "\n";

            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            meas_package.timestamp_ = timestamp;
    	}
    	else
    	{
    		float px;
    		float py;

    		linestream >> px >> py >> timestamp >> x_gt >> y_gt >> vx_gt >> vy_gt;
//        	cout << "sensor=" << sensor << " px=" <<  px << " py=" << py << " time=" << timestamp <<
//					" x_gt=" << x_gt << " y_gt=" << y_gt << " vx_gt=" << vx_gt << " vy_gt=" << vy_gt << "\n";

            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            meas_package.raw_measurements_ << px, py;
            meas_package.timestamp_ = timestamp;
    	}
#if 0
        if (count == 1)
        {
    		cout << meas_package.sensor_type_ << "\n";
    		cout << meas_package.raw_measurements_ << "\n";
    		cout << meas_package.timestamp_ << "\n";
        }
#endif
        VectorXd gt_values(4);
        gt_values(0) = x_gt;
        gt_values(1) = y_gt;
        gt_values(2) = vx_gt;
        gt_values(3) = vy_gt;
        ground_truth.push_back(gt_values);

          //Call ProcessMeasurment(meas_package) for Kalman filter
        fusionEKF.ProcessMeasurement(meas_package);

        //Push the current estimated x,y positon from the Kalman filter's state vector

        VectorXd estimate(4);

        double p_x = fusionEKF.ekf_.x_(0);
        double p_y = fusionEKF.ekf_.x_(1);
        double v1  = fusionEKF.ekf_.x_(2);
        double v2 = fusionEKF.ekf_.x_(3);

        estimate(0) = p_x;
        estimate(1) = p_y;
        estimate(2) = v1;
        estimate(3) = v2;

        estimations.push_back(estimate);

        VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
#if 0
        if (count == 1)
        {
			cout << count << "-th:   " << "px: " << p_x << ", py: " << p_y << ", vx: " << v1 << ", vy: " << v2 << "\n";
			cout << "=====================================================================================" << "\n";
        }
#endif
//        498	RMSE:    0.0973178, 0.0854597, 0.451267, 0.439935
//        499	RMSE:    0.0972256, 0.0853761, 0.450855, 0.439588
        count++;


        if (count == 3)
            return 0;


    }
    myfile.close();
  }

  else cout << "Unable to open file";

  return 0;
}


