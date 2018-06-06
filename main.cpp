#include <iostream>
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include "LoadObj.h"
#include "LoadBwf.h"

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

float get_sym_plane(Tree &tree, TriangleMesh &mesh);
void get_intersection(Tree &tree, Car2DCurveNet &curvenet_trans, Point *intersections);
void output_curvenet_tmplt(Car2DCurveNet &curvenet_output, Point *intersections, float scale_factor, float *trans_factors);
void output_curvenet_bwf(Car2DCurveNet &curvenet_output);

std::string filepath = "D:\\visual studio 2013\\Projects\\CGAL_intersection\\CGAL_intersection\\";
std::string filename = "Benz_S_class_render";

std::string objfilename = filepath + filename + ".obj";
std::string bwffilename = filepath + filename + ".TMPLT";
std::string output_tmplt = filepath + filename + "_output1.TMPLT";
std::string output_bwf = filepath + filename + "_output1.bwf";

int main(int argc, char *argv[])
{

	TriangleMesh mesh;

	loadObj(objfilename, mesh);

	//Rotate 90 degrees around the X axis
	rotate3(mesh, 1.0, 0.0, 0.0, 90);

	//Rotate 180 degrees around the Z axis
	rotate3(mesh, 0.0, 0.0, 1.0, 180);

	std::list<Triangle> triangles;
	float tmp_x = 0.0;
	float tmp_y = 0.0;
	float tmp_z = 0.0;
	int v_idx = 0;
	Point v1, v2, v3;
	for (int i = 0; i < mesh.faces.size(); i++)
	{
		v_idx = mesh.faces[i].v[0] - 1;
		tmp_x = mesh.verts[v_idx].x;
		tmp_y = mesh.verts[v_idx].y;
		tmp_z = mesh.verts[v_idx].z;
		v1 = Point(tmp_x,tmp_y, tmp_z);

		v_idx = mesh.faces[i].v[1] - 1;
		tmp_x = mesh.verts[v_idx].x;
		tmp_y = mesh.verts[v_idx].y;
		tmp_z = mesh.verts[v_idx].z;
		v2 = Point(tmp_x, tmp_y, tmp_z);

		v_idx = mesh.faces[i].v[2] - 1;
		tmp_x = mesh.verts[v_idx].x;
		tmp_y = mesh.verts[v_idx].y;
		tmp_z = mesh.verts[v_idx].z;
		v3 = Point(tmp_x, tmp_y, tmp_z);

		triangles.push_back(Triangle(v1, v2, v3));
	}
	std::cout << "---------- CGAL Triangles finished -------------" << std::endl;
	
	// constructs AABB tree
	Tree tree(triangles.begin(), triangles.end());
	std::cout << "---------- AABB tree constructed -------------" << std::endl;

	float center_y = get_sym_plane(tree, mesh);
	move_coord(mesh, center_y);

	std::list<Triangle> triangles1;
	for (int i = 0; i < mesh.faces.size(); i++)
	{
		v_idx = mesh.faces[i].v[0] - 1;
		tmp_x = mesh.verts[v_idx].x;
		tmp_y = mesh.verts[v_idx].y;
		tmp_z = mesh.verts[v_idx].z;
		v1 = Point(tmp_x, tmp_y, tmp_z);

		v_idx = mesh.faces[i].v[1] - 1;
		tmp_x = mesh.verts[v_idx].x;
		tmp_y = mesh.verts[v_idx].y;
		tmp_z = mesh.verts[v_idx].z;
		v2 = Point(tmp_x, tmp_y, tmp_z);

		v_idx = mesh.faces[i].v[2] - 1;
		tmp_x = mesh.verts[v_idx].x;
		tmp_y = mesh.verts[v_idx].y;
		tmp_z = mesh.verts[v_idx].z;
		v3 = Point(tmp_x, tmp_y, tmp_z);

		triangles1.push_back(Triangle(v1, v2, v3));
	}
	Tree tree1(triangles1.begin(), triangles1.end());
	std::cout << "---------- AABB tree updated -------------" << std::endl;

	Car2DCurveNet curvenet;
	loadBwf(bwffilename, curvenet);

	Car2DCurveNet curvenet_trans = curvenet;
	float trans_factors[2];
	coord_trans_Bwf(curvenet_trans, trans_factors);

	float obj_length = mesh.bounding_box[1].x - mesh.bounding_box[0].x;
	float obj_height = mesh.bounding_box[1].z - mesh.bounding_box[0].z;

	float tmplt_length = curvenet_trans.bounding_box[1].x - curvenet_trans.bounding_box[0].x;
	float tmplt_height = curvenet_trans.bounding_box[1].y - curvenet_trans.bounding_box[0].y;

	float scale_factor = obj_length / tmplt_length;
	scaleBwf(curvenet_trans, scale_factor);


	std::cout << "------------Calculating intersactions-----------" << std::endl;
	Point intersections[12];
	get_intersection(tree1, curvenet_trans, intersections);

	for (int i = 0; i < 12; i++)
	{
		std::cout << intersections[i] << std::endl;
	}

	Car2DCurveNet curvenet_output = curvenet_trans;
	output_curvenet_tmplt(curvenet_output, intersections, scale_factor, trans_factors);

	output_curvenet_bwf(curvenet_output);

	
	return EXIT_SUCCESS;
}

float get_sym_plane(Tree &tree,TriangleMesh &mesh)
{
	float model_lenght = mesh.bounding_box[1].x - mesh.bounding_box[0].x;
	float model_width = mesh.bounding_box[1].y - mesh.bounding_box[0].y;
	float model_height = mesh.bounding_box[1].z - mesh.bounding_box[0].z;

	//--------------get first pair of intersections
	// define the ray
	Point left_1pt_start(mesh.bbox_centroid.x, mesh.bounding_box[1].y + 10.0, mesh.bbox_centroid.z);
	Point left_1pt_end(mesh.bbox_centroid.x, mesh.bbox_centroid.y, mesh.bbox_centroid.z);
	Ray left_ray_query(left_1pt_start, left_1pt_end);

	std::cout << tree.number_of_intersected_primitives(left_ray_query)
		<< " intersections(s) with ray query" << std::endl;

	Ray_intersection left_intersection = tree.first_intersection(left_ray_query);
	Point* p1_left = boost::get<Point>(&(left_intersection->first));
	std::cout << *p1_left << std::endl;

	Point right_1pt_start(mesh.bbox_centroid.x, mesh.bounding_box[0].y - 10.0, mesh.bbox_centroid.z);
	Point right_1pt_end(mesh.bbox_centroid.x, mesh.bbox_centroid.y, mesh.bbox_centroid.z);
	Ray right_ray_query(right_1pt_start, right_1pt_end);
	std::cout << tree.number_of_intersected_primitives(right_ray_query)
		<< " intersections(s) with ray query" << std::endl;

	Ray_intersection intersection_right = tree.first_intersection(right_ray_query);
	Point* p1_right = boost::get<Point>(&(intersection_right->first));
	std::cout << *p1_right << std::endl;

	//------------------get second pair of intersections
	Point left_2pt_start(mesh.bbox_centroid.x + model_lenght * 0.25, mesh.bounding_box[1].y + 10.0, mesh.bbox_centroid.z);
	Point left_2pt_end(mesh.bbox_centroid.x + model_lenght * 0.25, mesh.bbox_centroid.y, mesh.bbox_centroid.z);
	left_ray_query = Ray(left_2pt_start, left_2pt_end);
	std::cout << tree.number_of_intersected_primitives(left_ray_query)
		<< " intersections(s) with ray query" << std::endl;

	left_intersection = tree.first_intersection(left_ray_query);
	
	Point* p2_left = boost::get<Point>(&(left_intersection->first));
	std::cout << *p2_left << std::endl;

	Point right_2pt_start(mesh.bbox_centroid.x + model_lenght * 0.25, mesh.bounding_box[0].y - 10.0, mesh.bbox_centroid.z);
	Point right_2pt_end(mesh.bbox_centroid.x + model_lenght * 0.25, mesh.bbox_centroid.y, mesh.bbox_centroid.z);
	right_ray_query = Ray(right_2pt_start, right_2pt_end);
	std::cout << tree.number_of_intersected_primitives(right_ray_query)
		<< " intersections(s) with ray query" << std::endl;

	intersection_right = tree.first_intersection(right_ray_query);
	Point* p2_right = boost::get<Point>(&(intersection_right->first));
	std::cout << *p2_right << std::endl;

	//------------------get third pair of intersections
	Point left_3pt_start(mesh.bbox_centroid.x - model_lenght * 0.25, mesh.bounding_box[1].y + 10.0, mesh.bbox_centroid.z);
	Point left_3pt_end(mesh.bbox_centroid.x - model_lenght * 0.25, mesh.bbox_centroid.y, mesh.bbox_centroid.z);
	left_ray_query = Ray(left_3pt_start, left_3pt_end);
	std::cout << tree.number_of_intersected_primitives(left_ray_query)
		<< " intersections(s) with ray query" << std::endl;

	left_intersection = tree.first_intersection(left_ray_query);

	Point* p3_left = boost::get<Point>(&(left_intersection->first));
	std::cout << *p3_left << std::endl;

	Point right_3pt_start(mesh.bbox_centroid.x - model_lenght * 0.25, mesh.bounding_box[0].y - 10.0, mesh.bbox_centroid.z);
	Point right_3pt_end(mesh.bbox_centroid.x - model_lenght * 0.25, mesh.bbox_centroid.y, mesh.bbox_centroid.z);
	right_ray_query = Ray(right_3pt_start, right_3pt_end);
	std::cout << tree.number_of_intersected_primitives(right_ray_query)
		<< " intersections(s) with ray query" << std::endl;

	intersection_right = tree.first_intersection(right_ray_query);
	Point* p3_right = boost::get<Point>(&(intersection_right->first));
	std::cout << *p3_right << std::endl;

	float center1_y = ((*p1_left)[1] + (*p1_right)[1]) * 0.5;
	float center2_y = ((*p2_left)[1] + (*p2_right)[1]) * 0.5;
	float center3_y = ((*p3_left)[1] + (*p3_right)[1]) * 0.5;

	float center_y = (center1_y + center2_y + center3_y) / 3;

	return 0.0;
}

void get_intersection(Tree &tree, Car2DCurveNet &curvenet_trans, Point *intersections)
{
	Point ray_start[12];
	Point ray_end[12];
	ray_start[0] = Point(curvenet_trans.lines[42].points[0].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[42].points[0].y);
	ray_start[1] = Point(curvenet_trans.lines[42].points[3].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[42].points[3].y);

	ray_start[2] = Point(curvenet_trans.lines[44].points[0].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[44].points[0].y);
	ray_start[3] = Point(curvenet_trans.lines[44].points[3].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[44].points[3].y);

	ray_start[4] = Point(curvenet_trans.lines[50].points[0].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[50].points[0].y);
	ray_start[5] = Point(curvenet_trans.lines[50].points[3].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[50].points[3].y);

	ray_start[6] = Point(curvenet_trans.lines[94].points[0].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[94].points[0].y);
	ray_start[7] = Point(curvenet_trans.lines[94].points[3].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[94].points[3].y);

	ray_start[8] = Point(curvenet_trans.lines[95].points[0].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[95].points[0].y);
	ray_start[9] = Point(curvenet_trans.lines[95].points[3].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[95].points[3].y);

	ray_start[10] = Point(curvenet_trans.lines[96].points[0].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[96].points[0].y);
	ray_start[11] = Point(curvenet_trans.lines[96].points[3].x, mesh.bounding_box[1].y + 10.0, curvenet_trans.lines[96].points[3].y);

	Ray ray_query[12];

	for (int i = 0; i < 12; i++)
	{
		ray_end[i] = Point(ray_start[i][0], 0, ray_start[i][2]);
		ray_query[i] = Ray(ray_start[i], ray_end[i]);
		std::cout << tree.number_of_intersected_primitives(ray_query[i])
			<< " intersections(s) with ray query" << std::endl;

		Ray_intersection intersection = tree.first_intersection(ray_query[i]);
		Point* p = boost::get<Point>(&(intersection->first));
		std::cout << *p << std::endl;
		intersections[i] = *p;
	}
}

void output_curvenet_tmplt(Car2DCurveNet &curvenet_output, Point *intersections, float scale_factor, float *trans_factors)
{
	curvenet_output.lines[42].points[0].z = intersections[0][1];
	curvenet_output.lines[42].points[3].z = intersections[1][1];

	curvenet_output.lines[44].points[0].z = intersections[2][1];
	curvenet_output.lines[44].points[3].z = intersections[3][1];

	curvenet_output.lines[50].points[0].z = intersections[4][1];
	curvenet_output.lines[50].points[3].z = intersections[5][1];

	curvenet_output.lines[94].points[0].z = intersections[6][1];
	curvenet_output.lines[94].points[3].z = intersections[7][1];

	curvenet_output.lines[95].points[0].z = intersections[8][1];
	curvenet_output.lines[95].points[3].z = intersections[9][1];

	curvenet_output.lines[96].points[0].z = intersections[10][1];
	curvenet_output.lines[96].points[3].z = intersections[11][1];

	curvenet_output.lines[43].points[0] = curvenet_output.lines[42].points[3];
	curvenet_output.lines[43].points[3] = curvenet_output.lines[44].points[0];
	curvenet_output.lines[45].points[0] = curvenet_output.lines[44].points[3];
	curvenet_output.lines[45].points[3] = curvenet_output.lines[42].points[0];
	curvenet_output.lines[46].points[0] = curvenet_output.lines[44].points[3];
	curvenet_output.lines[46].points[3] = curvenet_output.lines[95].points[0];

	curvenet_output.lines[47].points[0] = curvenet_output.lines[95].points[3];
	curvenet_output.lines[47].points[3] = curvenet_output.lines[96].points[0];
	curvenet_output.lines[48].points[0] = curvenet_output.lines[96].points[3];
	curvenet_output.lines[48].points[3] = curvenet_output.lines[42].points[0];

	curvenet_output.lines[49].points[0] = curvenet_output.lines[44].points[0];
	curvenet_output.lines[49].points[3] = curvenet_output.lines[50].points[0];
	curvenet_output.lines[51].points[0] = curvenet_output.lines[50].points[3];
	curvenet_output.lines[51].points[3] = curvenet_output.lines[44].points[3];
	curvenet_output.lines[52].points[0] = curvenet_output.lines[50].points[3];
	curvenet_output.lines[52].points[3] = curvenet_output.lines[94].points[0];
	curvenet_output.lines[53].points[0] = curvenet_output.lines[94].points[3];
	curvenet_output.lines[53].points[3] = curvenet_output.lines[95].points[3];

	curvenet_output.lines[54].points[0] = curvenet_output.lines[50].points[0];
	curvenet_output.lines[56].points[3] = curvenet_output.lines[50].points[3];
	curvenet_output.lines[57].points[3] = curvenet_output.lines[96].points[3];

	curvenet_output.lines[58].points[0] = curvenet_output.lines[96].points[3];
	curvenet_output.lines[58].points[3] = curvenet_output.lines[95].points[0];
	curvenet_output.lines[59].points[0] = curvenet_output.lines[95].points[0];
	curvenet_output.lines[59].points[3] = curvenet_output.lines[94].points[0];

	curvenet_output.lines[60].points[0] = curvenet_output.lines[94].points[0];

	float interpolation = 0.0;
	for (int i = 42; i < 61; i++)
	{
		interpolation = (curvenet_output.lines[i].points[3].z - curvenet_output.lines[i].points[0].z) / 3;
		curvenet_output.lines[i].points[1].z = curvenet_output.lines[i].points[0].z + interpolation;
		curvenet_output.lines[i].points[2].z = curvenet_output.lines[i].points[1].z + interpolation;
	}

	for (int i = 94; i < 97; i++)
	{
		interpolation = (curvenet_output.lines[i].points[3].z - curvenet_output.lines[i].points[0].z) / 3;
		curvenet_output.lines[i].points[1].z = curvenet_output.lines[i].points[0].z + interpolation;
		curvenet_output.lines[i].points[2].z = curvenet_output.lines[i].points[1].z + interpolation;
	}

	scaleBwf(curvenet_output, 1.0 / scale_factor);

	for (int i = 0; i < curvenet_output.lines.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			curvenet_output.lines[i].points[j].x += trans_factors[0];
			curvenet_output.lines[i].points[j].y = trans_factors[1] - curvenet_output.lines[i].points[j].y;
		}
	}
	bbox_section_curve(curvenet_output);

	std::ofstream fout(output_tmplt);
	for (int i = 0; i < curvenet_output.lines.size(); i++)
	{
		fout << "BEZIER  *" << setiosflags(ios::right) << setw(8)<< i + 1 << "           0" << setprecision(6) << std::fixed << setiosflags(ios::right) << setw(16) << curvenet_output.lines[i].points[0].x
			<< setw(16) << curvenet_output.lines[i].points[0].y << setw(16) << curvenet_output.lines[i].points[0].z << " T" << i+1 << endl;

		fout << "*T" << left << setw(2) << i + 1;
		fout << setprecision(6) << right << setw(20) << curvenet_output.lines[i].points[1].x
			<< setw(16) << curvenet_output.lines[i].points[1].y << setw(16) << curvenet_output.lines[i].points[1].z << endl;

		fout << "*T" << left << setw(2) << i + 1;
		fout << setprecision(6) << right << setw(20) << curvenet_output.lines[i].points[2].x
			<< setw(16) << curvenet_output.lines[i].points[2].y << setw(16) << curvenet_output.lines[i].points[2].z << endl;
		fout << "*T" << left << setw(2) << i + 1;
		fout << setprecision(6) << right << setw(20) << curvenet_output.lines[i].points[3].x
			<< setw(16) << curvenet_output.lines[i].points[3].y << setw(16) << curvenet_output.lines[i].points[3].z << endl;
		//fout << "*T" << i+1 << "            " << setprecision(6) << curvenet_output.lines[i].points[2].x << "      " << curvenet_output.lines[i].points[2].y << "        " << curvenet_output.lines[i].points[2].z << endl;
		//fout << "*T" << i+1 << "            " << setprecision(6) << curvenet_output.lines[i].points[3].x << "      " << curvenet_output.lines[i].points[3].y << "        " << curvenet_output.lines[i].points[3].z << endl;
	}
	
}

void output_curvenet_bwf(Car2DCurveNet &curvenet_output)
{
	std::ofstream fout(output_bwf);

	float center_front_wheel[2];
	center_front_wheel[0] = (curvenet_output.lines[29].points[0].x + curvenet_output.lines[30].points[0].x + curvenet_output.lines[31].points[0].x + curvenet_output.lines[32].points[0].x) / 4.0;
	center_front_wheel[1] = (curvenet_output.lines[29].points[0].y + curvenet_output.lines[30].points[0].y + curvenet_output.lines[31].points[0].y + curvenet_output.lines[32].points[0].y) / 4.0;

	float bwf_x[4], bwf_y[4], bwf_z[4];

	for (int i = 0; i < curvenet_output.lines.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			bwf_x[j] = curvenet_output.lines[i].points[j].x;
			bwf_y[j] = curvenet_output.lines[i].points[j].z;
			bwf_z[j] = curvenet_output.lines[i].points[j].y;
		}
		
		fout << "BEZIER  *" << setiosflags(ios::right) << setw(8) << i + 1 << "           0" << setprecision(6) << std::fixed << setiosflags(ios::right) << setw(16) << bwf_x[0] - center_front_wheel[0]
			<< setw(16) << bwf_y[0] << setw(16) << center_front_wheel[1] - bwf_z[0] << " T" << i + 1 << endl;

		fout << "*T" << left << setw(2) << i + 1;
		fout << setprecision(6) << right << setw(20) << bwf_x[1] - center_front_wheel[0]
			<< setw(16) << bwf_y[1] << setw(16) << center_front_wheel[1] - bwf_z[1] << endl;

		fout << "*T" << left << setw(2) << i + 1;
		fout << setprecision(6) << right << setw(20) << bwf_x[2] - center_front_wheel[0]
			<< setw(16) << bwf_y[2] << setw(16) << center_front_wheel[1] - bwf_z[2] << endl;
		fout << "*T" << left << setw(2) << i + 1;
		fout << setprecision(6) << right << setw(20) << bwf_x[3] - center_front_wheel[0]
			<< setw(16) << bwf_y[3] << setw(16) << center_front_wheel[1] - bwf_z[3] << endl;
		//fout << "*T" << i+1 << "            " << setprecision(6) << curvenet_output.lines[i].points[2].x << "      " << curvenet_output.lines[i].points[2].y << "        " << curvenet_output.lines[i].points[2].z << endl;
		//fout << "*T" << i+1 << "            " << setprecision(6) << curvenet_output.lines[i].points[3].x << "      " << curvenet_output.lines[i].points[3].y << "        " << curvenet_output.lines[i].points[3].z << endl;
	}

}