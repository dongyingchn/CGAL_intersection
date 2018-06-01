#include "LoadObj.h"
#include "LoadBwf.h"
#include <fstream>
#include <regex>  

void main()
{
	std::string objfilename = "D:\\visual studio 2013\\Projects\\CGAL_intersection\\CGAL_intersection\\Volkswagen_Phaeton_GP3_scale.obj";
	std::string bwffilename = "D:\\visual studio 2013\\Projects\\CGAL_intersection\\CGAL_intersection\\Volkswagen_Phaeton_GP3.BWF";

	TriangleMesh mesh;

	loadObj(objfilename, mesh);

	Car2DCurveNet curvenet;

	loadBwf(bwffilename, curvenet);
	
}