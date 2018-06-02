#ifndef  _LOADBWF_H_  
#define  _LOADBWF_H_  

#include <regex> 
#include <sstream>
#include "LoadObj.h"

using namespace std;

template <class Type>
Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}

struct bezier
{
	float3 points[4];
};

struct Car2DCurveNet
{
	vector<bezier> lines;
};

Car2DCurveNet curvenet;

void loadBwf(const std::string filename, Car2DCurveNet &curvenet);
void loadBwf(const std::string bwffilename, Car2DCurveNet &curvenet)
{
	std::ifstream fin(bwffilename);
	std::string s;
	int line_idx = 0;

	bezier curve;

	while (getline(fin, s))
	{
		line_idx++;
		//cout << "Read from file: " << s << endl;
		std::string pattern{ "-?\\d+\\.\\d+" }; // url  
		std::regex re(pattern);
		std::smatch results;

		int point_idx = 0;
		float p_x = 0.0;
		float p_y = 0.0;
		float p_z = 0.0;
		while (std::regex_search(s, results, re))
		{
			point_idx++;
			for (auto x : results)
			{
				if (point_idx == 1)
					p_x = stringToNum<float>(x.str());
				if (point_idx == 2)
					p_y = stringToNum<float>(x.str());
				if (point_idx == 3)
					p_z = stringToNum<float>(x.str());
			}
			//std::cout << std::endl;
			s = results.suffix().str();
		}

		float3 tmp = float3(p_x, p_y, p_z);

		if (line_idx % 4 == 1)
			curve.points[0] = tmp;
		if (line_idx % 4 == 2)
			curve.points[1] = tmp;
		if (line_idx % 4 == 3)
			curve.points[2] = tmp;
		if (line_idx % 4 == 0)
		{
			curve.points[3] = tmp;
			curvenet.lines.push_back(curve);
		}
	}

	cout << "----------bwf file loaded-------------" << endl;
	cout << "Number of bezier curves: " << curvenet.lines.size() << endl;
}

#endif  