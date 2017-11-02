// reading a text file
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;

int main () {
  string line;
  ifstream myfile ("data/obj_pose-laser-radar-synthetic-input.txt");

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

    	if (sensor.compare("R") == 0)
    	{
    		float rho;
    		float theta;
    		float v_rho;

    		linestream >> rho >> theta >> v_rho;
        	cout << "sensor=" << sensor << " rho=" <<  rho << " theta=" << theta << " vel=" << v_rho << '\n';
    	}
    	else
    	{
    		float px;
    		float py;

    		linestream >> px >> py;
        	cout << "sensor=" << sensor << " px=" <<  px << " py=" << py << '\n';
    	}

    }
    myfile.close();
  }

  else cout << "Unable to open file";

  return 0;
}


