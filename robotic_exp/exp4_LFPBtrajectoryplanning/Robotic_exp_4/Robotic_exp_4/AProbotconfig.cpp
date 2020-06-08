
#include"AProbotconfig.h"
#include"stdio.h"
#include<iostream>


#define PI 3.1415926535898
static double limit_max=(100.0 / 180 * PI);
static double limit_min=(-39.0 / 180 * PI);
static double R = 0.200; //m
static double r = 0.045; //m
static double L = 0.350;//m
static double l = 0.800;//m

namespace APRobot
{
	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		TransVector[16]:	位姿矩阵，其中长度距离为米
				
	OUTPUTS:    theta[4]：4个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, double* theta)
	{
		double g[16];
		for (int i = 0; i < 16; i++)
			g[i] = TransVector[i];

		Vector3d P;
		P << TransVector[3], TransVector[7], TransVector[11];

		/*zyz-0,180,yaw格式。此处第四轴的角度和yaw角相对于y对称，故第四轴的cos值取cosyaw的负号，sin值相同。*/
		/* 按zyz-0，180，yaw格式转换成变换矩阵，其旋转矩阵左上角第一行为 -cosyaw，sinyaw，0，x */
		theta[3] = atan2(TransVector[1], TransVector[0]); 
		
		double ai[3] = { 0,4 * PI / 3,2 * PI / 3 };
		Vector3d ci[3], temp;
		temp << R - r, 0, 0;
		for (int i = 0; i < 3; i++)
		{
			Matrix3d rot = AngleAxisd(ai[i], Vector3d(0, 0, 1)).toRotationMatrix();
			ci[i] = rot * P - temp;
			double Altitude = 2 * L * sqrt(ci[i](0) * ci[i](0) + ci[i](2) * ci[i](2));

			double theta1 = asin((ci[i].norm() * ci[i].norm() + L * L - l * l) / Altitude) - PI;
			theta1 += atan2(ci[i](0), ci[i](2));
			while (theta1 < limit_min || theta1> limit_max)
			{
				if (theta1 < limit_min)
					theta1 += 2 * PI;
				else if (theta1 > limit_max)
					theta1 -= 2 * PI;
			}
			theta[i] = theta1;
		}
	}
	/********************************************************************
	ABSTRACT:	机器人正运动学

	INPUTS:		q[4] ：4个关节角, 单位为弧度。前3个是并联主轴，第四个是中间轴旋转调整姿态，默认姿态为zyz：0，180，q[4]。
				Tool:默认朝上为z正。

	OUTPUTS:    TransVector[16]:	位姿矩阵，其中长度距离为米

	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, const double* Tool, double* TransVector)
	{
		double ai[3] = { 0,2 * PI / 3,4 * PI / 3 };
		Vector3d ci[3];
		for (int i = 0; i < 3; i++)
		{
			ci[i] << (R - r + L * cos(q[i])) * cos(ai[i]), (R - r + L * cos(q[i]))* sin(ai[i]), -L * sin(q[i]);
		}
		double ECi, p;
		Vector3d c1c2 = ci[1] - ci[0], c2c3 = ci[2] - ci[1], c3c1 = ci[0] - ci[2];
		p = (c1c2.norm() + c2c3.norm() + c3c1.norm()) / 2;
		ECi = (c1c2.norm() * c2c3.norm() * c3c1.norm()) / (4 * sqrt(p * (p - c1c2.norm()) * (p - c2c3.norm()) * (p - c3c1.norm())));
		Vector3d F = (ci[1] + ci[0]) / 2, temp = c2c3.cross(c3c1).cross(c1c2), nFE = temp / temp.norm();
		double FE = sqrt(ECi * ECi - pow(c1c2.norm(), 2) / 4), EP = sqrt(l * l - ECi * ECi);
		Vector3d E = FE * nFE + F, nEP = (-c1c2.cross(c2c3));
		nEP = nEP / nEP.norm();
		Vector3d P = EP * nEP + E;
		Matrix3d yaw = AngleAxisd(-q[3], Vector3d(0, 0, 1)).toRotationMatrix();
		P = yaw * P;
		Isometry3d T = Isometry3d::Identity();

		T.rotate(yaw);
		T.pretranslate(P);
		
		Matrix<double, 4, 4, RowMajor> trans = T.matrix();
		for (int i = 0; i < 16; i++)
			TransVector[i] = trans.data()[i];
	}


	void GetJointAngles(double* trans, double& angle1, double& angle2, double& angle3, double& angle4)//弧度
	{
		double theta[4];//度
		APRobot::robotBackward(trans,theta);
		angle1 = theta[0] * 180 / PI;
		angle2 = theta[1] * 180 / PI;
		angle3 = theta[2] * 180 / PI;
		angle4 = theta[3] * 180 / PI;
	}
}

