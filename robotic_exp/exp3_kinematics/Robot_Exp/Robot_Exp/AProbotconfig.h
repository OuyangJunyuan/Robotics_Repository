
#ifndef AP_ROBOTCONFIG_H_
#define AP_ROBOTCONFIG_H_
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
#define PI 3.1415926535898
#define R 0.200 //m
#define r 0.045   //m
#define L 0.350 //m
#define l 0.800 //m
namespace APRobot
{ 
		void robotBackward(const double* TransVector, double* theta);
		void robotForward(const double* q, const double* Tool, double* TransVector);
}

#endif
