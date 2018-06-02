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

int main(int argc, char *argv[])
{
	std::string objfilename = "D:\\visual studio 2013\\Projects\\CGAL_intersection\\CGAL_intersection\\Benz_model.obj";
	std::string bwffilename = "D:\\visual studio 2013\\Projects\\CGAL_intersection\\CGAL_intersection\\Volkswagen_Phaeton_GP3.BWF";

	TriangleMesh mesh;

	loadObj(objfilename, mesh);

	//Rotate 90 degrees around the X axis
	rotate3(mesh, 1.0, 0.0, 0.0, 90);

	//Rotate 180 degrees around the Z axis
	rotate3(mesh, 0.0, 0.0, 1.0, 180);

	Car2DCurveNet curvenet;

	loadBwf(bwffilename, curvenet);

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