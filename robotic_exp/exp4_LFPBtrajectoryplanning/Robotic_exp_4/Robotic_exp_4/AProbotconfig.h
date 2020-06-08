
#ifndef AP_ROBOTCONFIG_H_
#define AP_ROBOTCONFIG_H_
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
#define PI 3.1415926535898
namespace APRobot
{ 
	
	void robotBackward(const double* TransVector, double* theta);
	void robotForward(const double* q, const double* Tool, double* TransVector);
	void GetJointAngles(double *trans,double &angle1, double& angle2, double& angle3, double& angle4);
}

#endif
