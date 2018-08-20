// 
#include "ObjectModeller3D.h"
#include <string>
#include <stdexcept>
// compile string: 
// cmake && make clean && make

using std::cout;
using std::cerr;
using std::endl;
using std::string;

int main(int argc, char *argv[])
{	
	InputType inputType;
	
	if (argc > 11)
	{
		string dir = argv[2];
		// need to create flags to see if the file needs to be read, or to create a new file for new data
		// using -c for camera
		// using -f for file
		if (strcmp(argv[1], "-c") == 0)
		{
			inputType = InputType::Camera;
			cout << "Data is captured from the camera, and saved in " << dir << "output/" << endl;
		}
		else if (strcmp(argv[1], "-f") == 0)
		{
			inputType = InputType::Directory;
			cout << "Data is read from the directory: " << dir << " , and saved in " << dir << "output/" << endl;
		}
		

		string angleArg = argv[3];
		int angle;
		try 
		{
			size_t pos;
			angle = std::stoi(angleArg, &pos);
			if (pos < angleArg.size()) cerr << "Trailing characters after angle: " << angleArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << angleArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Angle out of range: " << angleArg << endl;
		}

		string leafArg = argv[4];
		float leaf;
		try 
		{
			size_t pos;
			leaf = std::stof(leafArg, &pos);
			if (pos < leafArg.size()) cerr << "Trailing characters after leaf size: " << leafArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << leafArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Leaf size out of range: " << leafArg << endl;
		}

		string meanArg = argv[5];
		float mean;
		try 
		{
			size_t pos;
			mean = std::stof(meanArg, &pos);
			if (pos < meanArg.size()) cerr << "Trailing characters after Value: " << meanArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << meanArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << meanArg << endl;
		}

		string stdDevArg = argv[6];
		float stdDev;
		try 
		{
			size_t pos;
			stdDev = std::stof(stdDevArg, &pos);
			if (pos < stdDevArg.size()) cerr << "Trailing characters after Value: " << stdDevArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << stdDevArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << stdDevArg << endl;
		}

		string xMinArg = argv[7];
		float xMin;
		try 
		{
			size_t pos;
			xMin = std::stof(xMinArg, &pos);
			if (pos < xMinArg.size()) cerr << "Trailing characters after Value: " << xMinArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << xMinArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << xMinArg << endl;
		}

		string xMaxArg = argv[8];
		float xMax;
		try 
		{
			size_t pos;
			xMax = std::stof(xMaxArg, &pos);
			if (pos < xMaxArg.size()) cerr << "Trailing characters after Value: " << xMaxArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << xMaxArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << xMaxArg << endl;
		}

		string yMinArg = argv[9];
		float yMin;
		try 
		{
			size_t pos;
			yMin = std::stof(yMinArg, &pos);
			if (pos < yMinArg.size()) cerr << "Trailing characters after Value: " << yMinArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << yMinArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << yMinArg << endl;
		}

		string yMaxArg = argv[10];
		float yMax;
		try 
		{
			size_t pos;
			yMax = std::stof(yMaxArg, &pos);
			if (pos < yMaxArg.size()) cerr << "Trailing characters after Value: " << yMaxArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << yMaxArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << yMaxArg << endl;
		}

		string zMinArg = argv[11];
		float zMin;
		try 
		{
			size_t pos;
			zMin = std::stof(zMinArg, &pos);
			if (pos < zMinArg.size()) cerr << "Trailing characters after Value: " << zMinArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << zMinArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << zMinArg << endl;
		}

		string zMaxArg = argv[12];
		float zMax;
		try 
		{
			size_t pos;
			zMax = std::stof(zMaxArg, &pos);
			if (pos < zMaxArg.size()) cerr << "Trailing characters after Value: " << zMaxArg << endl;
		} 
		catch (std::invalid_argument const &ex) 
		{
			cerr << "Invalid Number: " << zMaxArg << endl;
		}
		catch (std::out_of_range const &ex)
		{
			cerr << "Value out of range: " << zMaxArg << endl;
		}

		ObjectModeller3D modeller(inputType, angle, dir, leaf, mean, stdDev, xMin, xMax, yMin, yMax, zMin, zMax);

		cout << "Each input cloud is rotated by " << angle << " degrees from the previous." << endl;
		cout << "This means there will be " << 360/angle << " total point clouds." << endl;

		modeller.generate_model();
		return 0;
	}
	else
	{
		cout << "Insufficient Arguments." << endl;
		cout << "\t Usage: ./quick_turtle [data-option -c/-f] /path/to/directory/ <angle> <leaf-size> <mean> <std-dev> <xmin> <xmax> <ymin> <ymax> <zmin> <zmax>" << endl;
		return -1;
	}
}    

