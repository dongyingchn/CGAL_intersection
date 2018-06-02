#ifndef  _LOADOBJ_H_  
#define  _LOADOBJ_H_  

#include <math.h>  
#include <iostream>  
#include <fstream>  
#include <string>  
#include <vector> 
#include <stdlib.h>

using namespace std;

#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z*(v).z)    
#define norm2(v)   dot(v,v)        // norm2 = squared length of vector    
#define norm(v)    sqrt(norm2(v))  // norm = length of vector    
#define dis(u,v)     norm(u-v)       // distance = norm of difference   
#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))

// Obj loader  
struct TriangleFace
{
	int v[3]; // vertex indices  
};

class float3
{
public:
	float x;
	float y;
	float z;
public:
	float3(){ x = 0; y = 0; z = 0; }
	float3(float mx, float my, float mz){ x = mx; y = my; z = mz; }
	~float3(){}

	float3 operator+(float3);
	float3 operator-(float3);
	float3 operator/(float);

	friend float3 operator*(float m, float3 f3)
	{
		return float3(f3.x*m, f3.y*m, f3.z*m);
	}
	friend float3 operator*(float3 f3, float m)
	{
		return float3(f3.x*m, f3.y*m, f3.z*m);
	}

	float3 operator=(float3);

	float3& operator += (float3);

};

float3 float3::operator +(float3 m)
{
	float3 result;
	result.x = x + m.x;
	result.y = y + m.y;
	result.z = z + m.z;
	return result;
}

float3 float3::operator - (float3 m)
{
	float3 result;
	result.x = x - m.x;
	result.y = y - m.y;
	result.z = z - m.z;
	return result;
}

float3 float3::operator /(float m)
{
	if (m == 0){
		printf("error /");
		return float3(x, y, z);
	}
	else
		return float3(x / m, y / m, z / m);
}

float3 float3::operator =(float3 f3)
{
	x = f3.x;
	y = f3.y;
	z = f3.z;
	return float3(x, y, z);
}

struct TriangleMesh
{
	vector<float3> verts;
	vector<TriangleFace> faces;
	vector<TriangleFace> textures;
	vector<TriangleFace> normals;
	//模型的包围盒  
	float3 bounding_box[2];
	float3 bbox_centroid;
	//模型的包围球参数  
	float3 bounding_sphere_c;
	float bounding_sphere_r;
};

TriangleMesh mesh;

int total_number_of_triangles = 0;

// Scene bounding box   
float3 scene_aabbox_min;
float3 scene_aabbox_max;

void calculate_bbox(TriangleMesh &mesh);
void loadObj(const std::string filename, TriangleMesh &mesh);

void loadObj(const std::string filename, TriangleMesh &mesh)
{
	std::ifstream in(filename.c_str());

	if (!in.good())
	{
		cout << "ERROR: loading obj:(" << filename << ") file is not good" << "\n";
		exit(0);
	}

	char buffer[256], str[255];
	float f1, f2, f3;

	while (!in.getline(buffer, 255).eof())
	{
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		// reading a vertex  
		if (buffer[0] == 'v' && (buffer[1] == ' ' || buffer[1] == 32))
		{
			if (sscanf(buffer, "v %f %f %f", &f1, &f2, &f3) == 3)
			{
				mesh.verts.push_back(float3(f1, f2, f3));
			}
			else
			{
				cout << "ERROR: vertex not in wanted format in OBJLoader" << "\n";
				exit(-1);
			}
		}
		// reading FaceMtls   
		else if (buffer[0] == 'f' && (buffer[1] == ' ' || buffer[1] == 32))
		{
			TriangleFace f;
			TriangleFace t;
			TriangleFace n;
			int nt = sscanf(buffer, "f %d/%d/%d %d/%d/%d %d/%d/%d", &f.v[0], &t.v[0], &n.v[0], &f.v[1], &t.v[1], &n.v[1], &f.v[2], &t.v[2], &n.v[2]);
			if (nt != 9)
			{
				cout << "ERROR: I don't know the format of that FaceMtl" << "\n";
				exit(-1);
			}

			mesh.faces.push_back(f);
			mesh.textures.push_back(t);
			mesh.normals.push_back(n);
		}
	}

	calculate_bbox(mesh);
}

void calculate_bbox(TriangleMesh &mesh)
{
	float xmin, ymin, zmin, xmax, ymax, zmax;
	int Pxmin, Pxmax, Pymin, Pymax, Pzmin, Pzmax;

	//calculate the bounding sphere  
	xmin = xmax = mesh.verts[0].x;
	ymin = ymax = mesh.verts[0].y;
	zmin = zmax = mesh.verts[0].z;
	Pxmin = Pxmax = Pymin = Pymax = Pzmin = Pzmax = 0;

	//calculate the bounding box  
	mesh.bounding_box[0] = float3(mesh.verts[0].x, mesh.verts[0].y, mesh.verts[0].z);
	mesh.bounding_box[1] = float3(mesh.verts[0].x, mesh.verts[0].y, mesh.verts[0].z);

	for (unsigned int i = 1; i < mesh.verts.size(); i++)
	{
		//update min value  
		mesh.bounding_box[0].x = min(mesh.verts[i].x, mesh.bounding_box[0].x);
		mesh.bounding_box[0].y = min(mesh.verts[i].y, mesh.bounding_box[0].y);
		mesh.bounding_box[0].z = min(mesh.verts[i].z, mesh.bounding_box[0].z);

		//update max value  
		mesh.bounding_box[1].x = max(mesh.verts[i].x, mesh.bounding_box[1].x);
		mesh.bounding_box[1].y = max(mesh.verts[i].y, mesh.bounding_box[1].y);
		mesh.bounding_box[1].z = max(mesh.verts[i].z, mesh.bounding_box[1].z);

		//update the  x min and max  
		if (mesh.verts[i].x < xmin){
			xmin = mesh.verts[i].x;
			Pxmin = i;
		}
		else if (mesh.verts[i].x > xmax){
			xmax = mesh.verts[i].x;
			Pxmax = i;
		}
		//update the y min and max  
		if (mesh.verts[i].y < ymin){
			ymin = mesh.verts[i].y;
			Pymin = i;
		}
		else if (mesh.verts[i].y > ymax){
			ymax = mesh.verts[i].y;
			Pymax = i;
		}
		//update the z min and max  
		if (mesh.verts[i].z < zmin){
			zmin = mesh.verts[i].z;
			Pzmin = i;
		}
		else if (mesh.verts[i].z > zmax){
			zmax = mesh.verts[i].z;
			Pzmax = i;
		}
	}

	mesh.bbox_centroid.x = (mesh.bounding_box[0].x + mesh.bounding_box[1].x) / 2.0;
	mesh.bbox_centroid.y = (mesh.bounding_box[0].y + mesh.bounding_box[1].y) / 2.0;
	mesh.bbox_centroid.z = (mesh.bounding_box[0].z + mesh.bounding_box[1].z) / 2.0;

	//calculate the bounding sphere  
	float3 dVx = mesh.verts[Pxmax] - mesh.verts[Pxmin];
	float3 dVy = mesh.verts[Pymax] - mesh.verts[Pymin];
	float3 dVz = mesh.verts[Pzmax] - mesh.verts[Pzmin];
	float dx2 = norm2(dVx);
	float dy2 = norm2(dVy);
	float dz2 = norm2(dVz);

	float3 center;
	float  radius2;
	float  radius;

	if (dx2 >= dy2 && dx2 >= dz2) {                 // x direction is largest extent    
		center = mesh.verts[Pxmin] + (dVx / 2.0);   // Center = midpoint of extremes    
		radius2 = norm2(mesh.verts[Pxmax] - center);// radius squared    
	}
	else if (dy2 >= dx2  && dy2 >= dz2){                // y direction is largest extent    
		center = mesh.verts[Pymin] + (dVy / 2.0);   // Center = midpoint of extremes    
		radius2 = norm2(mesh.verts[Pymax] - center);// radius squared    
	}
	else{
		center = mesh.verts[Pzmin] + (dVz / 2.0);   // Center = midpoint of extremes    
		radius2 = norm2(mesh.verts[Pzmax] - center);// radius squared     
	}

	radius = sqrt(radius2);

	// now check that all points V[i] are in the ball    
	// and if not, expand the ball just enough to include them    
	float3 dV;
	float dist2, dist;
	for (unsigned int i = 0; i<mesh.verts.size(); i++)
	{
		dV = mesh.verts[i] - center;
		dist2 = norm2(dV);
		if (dist2 <= radius2) // V[i] is inside the ball already    
			continue;

		// V[i] not in ball, so expand ball to include it    
		dist = sqrt(dist2);
		radius = (radius + dist) / 2.0;         // enlarge radius just enough    
		radius2 = radius * radius;
		center = center + ((dist - radius) / dist) * dV;   // shift Center toward V[i]    

	}

	mesh.bounding_sphere_c = center;
	mesh.bounding_sphere_r = radius;

	cout << "----------obj file loaded or updated-------------" << endl;
	cout << "number of faces:" << mesh.faces.size() << " number of vertices:" << mesh.verts.size() << endl;
	cout << "obj bounding box: min:("
		<< mesh.bounding_box[0].x << "," << mesh.bounding_box[0].y << "," << mesh.bounding_box[0].z << ") max:("
		<< mesh.bounding_box[1].x << "," << mesh.bounding_box[1].y << "," << mesh.bounding_box[1].z << ")" << endl
		<< "obj bounding sphere center:" << mesh.bounding_sphere_c.x << "," << mesh.bounding_sphere_c.y << "," << mesh.bounding_sphere_c.z << endl
		<< "obj bounding sphere radius:" << mesh.bounding_sphere_r << endl;
}

void rotate3(TriangleMesh &mesh, double x0, double y0, double z0, double ang);

void rotate3(TriangleMesh &mesh, double x0, double y0, double z0, double ang)
{

	double x2 = x0*x0;
	double y2 = y0*y0;
	double z2 = z0*z0;

	double d2 = x2 + y2 + z2;
	double d = sqrt(d2);

	double sina = sin(ang);
	double cosa = cos(ang);

	float3 q, ans;

	for (int i = 0; i < mesh.verts.size(); i++)
	{
		q.x = mesh.verts[i].x;
		q.y = mesh.verts[i].y;
		q.z = mesh.verts[i].z;

		if (x0 == 1 && y0 == 0 && z0 == 0 && ang == 90)
		{
			ans.x = q.x;
			ans.y = -q.z;
			ans.z = q.y;
		}
		else if (x0 == 0 && y0 == 0 && z0 == 1 && ang == 180)
		{
			ans.x = -q.x;
			ans.y = -q.y;
			ans.z = q.z;
		}
		else
		{
			ans.x = (x2 + (y2 + z2)*cosa) / d2*q.x
				+ (x0*y0*(1 - cosa) / d2 - z0*sina / d)* q.y
				+ (x0*z0*(1 - cosa) / d2 + y0*sina / d)*q.z;

			ans.y = (y0*x0*(1 - cosa) / d2 + z0*sina / d)*q.x
				+ (y2 + (x2 + z2)*cosa) / d2* q.y
				+ (y0*z0*(1 - cosa) / d2 - x0*sina / d)*q.z;

			ans.z = (z0*x0*(1 - cosa) / d2 - y0*sina / d)*q.x
				+ (z0*y0*(1 - cosa) / d2 + x0*sina / d)*q.y
				+ (z2 + (x2 + y2)*cosa) / d2*q.z;
		}

		mesh.verts[i].x = ans.x;
		mesh.verts[i].y = ans.y;
		mesh.verts[i].z = ans.z;
	}

	calculate_bbox(mesh);
}

void move_coord(TriangleMesh &mesh, float y);
void move_coord(TriangleMesh &mesh, float y)
{
	for (int i = 0; i < mesh.verts.size(); i++)
	{
		mesh.verts[i].x += mesh.bounding_box[1].x;
		mesh.verts[i].z += mesh.bounding_box[1].z;
		mesh.verts[i].y += y;
	}

	calculate_bbox(mesh);
}

#endif  