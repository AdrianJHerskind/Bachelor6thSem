#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <math.h>
#include <complex>
#include <vector>
 
#include <Eigen/Dense>
//using namespace Eigen;



int main (int argc, char** argv)
{
  ros::init(argc, argv, "pcl_read");
  ros::NodeHandle nh;

std::vector<double> p1;
std::vector<double> p2;
std::vector<double> p3;


double p1x;
std::cout << "input p1x" << "\n";
std::cin >> p1x;
double p1y;
std::cout << "input p1y" << "\n";
std::cin >> p1y;
double p1z;
std::cout << "input p1z" << "\n";
std::cin >> p1z;
p1.push_back(p1x);
p1.push_back(p1y);
p1.push_back(p1z);

double p2x;
std::cout << "input p2x" << "\n";
std::cin >> p2x;
double p2y;
std::cout << "input p2y" << "\n";
std::cin >> p2y;
double p2z;
std::cout << "input p2z" << "\n";
std::cin >> p2z;
p2.push_back(p2x);
p2.push_back(p2y);
p2.push_back(p2z);

double p3x;
std::cout << "input p3x" << "\n";
std::cin >> p3x;
double p3y;
std::cout << "input p3y" << "\n";
std::cin >> p3y;
double p3z;
std::cout << "input p3z" << "\n";
std::cin >> p3z;
p3.push_back(p3x);
p3.push_back(p3y);
p3.push_back(p3z);

/*Vector3d p1(0,0,0);
Vector3d p2(2,0,0);
Vector3d p3(0,2,0);*/

//triangle "edges"
std::vector<double> t;
std::vector<double> u;
std::vector<double> v;

t.push_back(p2.at(0)-p1.at(0));
t.push_back(p2.at(1)-p1.at(1));
t.push_back(p2.at(2)-p1.at(2));

u.push_back(p3.at(0)-p1.at(0));
u.push_back(p3.at(1)-p1.at(1));
u.push_back(p3.at(2)-p1.at(2));

v.push_back(p3.at(0)-p2.at(0));
v.push_back(p3.at(1)-p2.at(1));
v.push_back(p3.at(2)-p2.at(2));

/*// triangle "edges"
const Vector3d t = p2-p1;
const Vector3d u = p3-p1;
const Vector3d v = p3-p2;*/


//triangle normal  tcrossu
std::vector<double> w;
w.push_back(t.at(1)*u.at(2) - t.at(2)*u.at(1));
w.push_back(t.at(2)*u.at(0) - t.at(0)*u.at(2));
w.push_back(t.at(0)*u.at(1) - t.at(1)*u.at(0));

double wsl = (w.at(0)*w.at(0) + w.at(1)*w.at(1) + w.at(2)*w.at(2));
std::cout << "wsl " << wsl << "\n";
if (wsl<10e-14) return false; // area of the triangle is too small (you may additionally check the podoubles for colinearity if you are paranoid)

// triangle normal
//const Vector3d w = t.cross(u);
//const double wsl = w*w;
//if (wsl<10e-14) return false; // area of the triangle is too small (you may additionally check the podoubles for colinearity if you are paranoid)

//helpers
double iwsl2 = (1.0 / ((2.0)*wsl));
std::cout << "iwsl2 " << iwsl2 << "\n";
double tt = (t.at(0)*t.at(0) + t.at(1)*t.at(1) + t.at(2)*t.at(2));
double uu = (u.at(0)*u.at(0) + u.at(1)*u.at(1) + u.at(2)*u.at(2));

// helpers
//const double iwsl2 = 1.0 / (2.0*wsl);
//const double tt = t*t;
//const double uu = u*u;

//result circle
double circX;
double circY;
double circZ;
double uv = (u.at(0)*v.at(0) + u.at(1)*v.at(1) + u.at(2)*v.at(2));
double tv = (t.at(0)*v.at(0) + t.at(1)*v.at(1) + t.at(2)*v.at(2));
double vv = (v.at(0)*v.at(0) + v.at(1)*v.at(1) + v.at(2)*v.at(2));
std::cout << "p1.at(0) " << p1.at(0) << " u.at(0) " << u.at(0) << " tt " << tt << " uv " << uv << " t.at(0) " << t.at(0) << " uu " << uu << " tv " << tv << " iwsl2 " << iwsl2 << "\n";
circX = p1.at(0) + (u.at(0)*tt*(uv) - t.at(0)*uu*(tv)) * iwsl2;
circY = p1.at(1) + (u.at(1)*tt*(uv) - t.at(1)*uu*(tv)) * iwsl2;
circZ = p1.at(2) + (u.at(2)*tt*(uv) - t.at(2)*uu*(tv)) * iwsl2;
std::vector<double> circCenter;
circCenter.push_back(circX);
circCenter.push_back(circY);
circCenter.push_back(circZ);

double circRadius = sqrt(tt * uu * (vv) * iwsl2*0.5);
std::vector<double> circAxis;
circAxis.push_back(w.at(0) / sqrt(wsl));
circAxis.push_back(w.at(1) / sqrt(wsl));
circAxis.push_back(w.at(2) / sqrt(wsl));
// result circle
//Vector3d circCenter = p1 + (u*tt*(u*v) - t*uu*(t*v)) * iwsl2;
//double   circRadius = sqrt(tt * uu * (v*v) * iwsl2*0.5);
//Vector3d circAxis   = w / sqrt(wsl);
 
std::cout << "CircCenter.x " << circCenter.at(0) << " circCenter.y " << circCenter.at(1) << " circCenter.z " << circCenter.at(2) << "\n";
std::cout << "circRadius " << circRadius << "\n";
std::cout << "circAxis0 " << circAxis.at(0) << " circAxis1 " << circAxis.at(1) << " circAxis2 " << circAxis.at(2) << "\n";
char input;
std::cin >> input;

while(ros::ok())
{
 	
	ros::spinOnce();
}
  return (0);
}
