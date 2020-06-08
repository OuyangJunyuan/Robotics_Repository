#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "AProbotconfig.h"
#include <algorithm>
#include <Windows.h>

#include "math.h"

using namespace std;
using namespace APRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

static void zyz2rot(PosClass startPos, double* mStartMatrixData)
{
	double startAngle[3];
	startAngle[0] = startPos.yaw * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll * PI / 180;

	mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;
}
void CHLMotionPlan::LFPB(double bPos, double ePos, double vel, double acc, double dec, double now, double& output, double& t)
{
	double a1 = 0, a2 = 0, v = 0, x = 0, dir = 1;

	dir = (bPos - ePos) < 0 ? 1 : -1;

	x = dir * (ePos - bPos);
	v = vel;
	a1 = dir == 1 ? acc: dec;
	a2 = dir == 1 ? dec : acc;

	double dt_phase1 = v / a1, dt_phase3 = v / a2, dt_phase2 = 0;
	double dx_phase1 = 0.5 * a1 * dt_phase1 * dt_phase1, dx_phase3 = 0.5 * a2 * dt_phase3 * dt_phase3, dx_phase2 = 0;
	if (dx_phase1 + dx_phase3 <= x)//有匀速段。
	{
		dx_phase2 = x - dx_phase1 - dx_phase3;
		dt_phase2 = dx_phase2 / v;

		if (now <= dt_phase1)
		{
			output = bPos + dir * (0.5 * a1 * now * now);
		}
		else if (now <= dt_phase1 + dt_phase2)
		{
			output = bPos + dir * (0.5 * a1 * dt_phase1 * dt_phase1 + v * (now - dt_phase1));
		}
		else if (now <= dt_phase1 + dt_phase2 + dt_phase3)
		{
			output = ePos - dir * (0.5 * a2 * pow(dt_phase1 + dt_phase2 + dt_phase3 - now, 2));
		}
		else
		{
			output = ePos;
		}
	}
	else
	{
		dt_phase1 = sqrt(2 * x / (a1 + a1 * a1 / a2));
		dt_phase3 = a1 * dt_phase1 / a2;
		if (now <= dt_phase1)
		{
			output = bPos + dir * (0.5 * a1 * now * now);
		}
		else if (now <= dt_phase1 + dt_phase3)
		{
			output = ePos - dir * (0.5 * a2 * pow(dt_phase1 + dt_phase3 - now, 2));
		}
		else
		{
			output = ePos;
		}
	}
	t = dt_phase1 + dt_phase2 + dt_phase3;
}
void CHLMotionPlan::Delta_3D_LFPB(double now, Vector4d& output, double& t)
{
	Vector3d out_3d;
	double a1 = mAcc, a2 = mDec, v = mVel,x = this->Norm;
	double dt_phase1 = v / a1, dt_phase3 = v / a2, dt_phase2 = 0;
	double dx_phase1 = 0.5 * a1 * dt_phase1 * dt_phase1, dx_phase3 = 0.5 * a2 * dt_phase3 * dt_phase3, dx_phase2 = 0;
	if (dx_phase1 + dx_phase3 <= x)//有匀速段。
	{
		dx_phase2 = x - dx_phase1 - dx_phase3;
		dt_phase2 = dx_phase2 / v;

		if (now <= dt_phase1)
		{
			out_3d = mStartPointData + Direaction * (0.5 * a1 * now * now);
		}
		else if (now <= dt_phase1 + dt_phase2)
		{
			out_3d = mStartPointData + Direaction * (0.5 * a1 * dt_phase1 * dt_phase1 + v * (now - dt_phase1));
		}
		else if (now <= dt_phase1 + dt_phase2 + dt_phase3)
		{
			out_3d = mEndPointData - Direaction * (0.5 * a2 * pow(dt_phase1 + dt_phase2 + dt_phase3 - now, 2));
		}
		else
		{
			out_3d = mEndPointData;
		}
	}
	else
	{
		dt_phase1 = sqrt(2 * x / (a1 + a1 * a1 / a2));
		dt_phase3 = a1 * dt_phase1 / a2;
		if (now <= dt_phase1)
		{
			out_3d = mStartPointData + Direaction * (0.5 * a1 * now * now);
		}
		else if (now <= dt_phase1 + dt_phase3)
		{
			out_3d = mEndPointData - Direaction * (0.5 * a2 * pow(dt_phase1 + dt_phase3 - now, 2));
		}
		else
		{
			out_3d = mEndPointData;
		}
	}
	t = dt_phase1 + dt_phase2 + dt_phase3;
	
	double out_yaw;
	LFPB(mJointAngleBegin[3], mJointAngleEnd[3], mAVel, mAAcc, mADec, now/t*yawtimes, out_yaw, this->yawtimes);
	
	output(0) = out_3d(0);
	output(1) = out_3d(1);
	output(2) = out_3d(2);
	output(3) = out_yaw;
}
void CHLMotionPlan::Delta_AS_LFPB(double now, double* output, double& t_max)
{
	double _t_max = 0, t;
	/* 空间位置用线速度规划 */
	for (int i = 0; i < 4; i++)
	{
		LFPB(mJointAngleBegin[i], mJointAngleEnd[i], mAVel, mAAcc, mADec, now, output[i], t);
		_t_max = t > _t_max ? t : _t_max;
	}

	t_max = _t_max;
}


PosClass::PosClass(){;}
PosClass::~PosClass(){;}
PosClass::PosClass(double x, double y, double z, double yaw, double pitch, double roll)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->yaw = yaw;
	this->pitch = pitch;
	this->roll = roll;
}


CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 4; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}
	for (int i = 0; i < 3; i++)
	{
		mConfig[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

CHLMotionPlan::~CHLMotionPlan()
{

}

void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/* m  degree */
void CHLMotionPlan::SetProfile(double vel, double acc, double dec, double avel, double aacc, double adec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;

	mAVel = avel;
	mADec = adec;
	mAAcc = aacc;
}

void CHLMotionPlan::SetPlanPoints(PosClass startPos, PosClass endPos)
{

	/* 设置起始终止变换矩阵 */
	zyz2rot(startPos, mStartMatrixData);
	zyz2rot(endPos, mEndMatrixData);

	/* 逆解设置起始终止关节角 */
	APRobot::GetJointAngles(mStartMatrixData, mJointAngleBegin[0], mJointAngleBegin[1], mJointAngleBegin[2], mJointAngleBegin[3]);
	APRobot::GetJointAngles(mEndMatrixData, mJointAngleEnd[0], mJointAngleEnd[1], mJointAngleEnd[2], mJointAngleEnd[3]);

	/* 设置笛卡尔始末位姿 */
	mStartPointData(0) = startPos.x / 1000;
	mStartPointData(1) = startPos.y / 1000;
	mStartPointData(2) = startPos.z / 1000;
	mEndPointData(0) = endPos.x / 1000;
	mEndPointData(1) = endPos.y / 1000;
	mEndPointData(2) = endPos.z / 1000;

	this->Direaction = (mEndPointData - mStartPointData);
	this->Norm = this->Direaction.norm();
	this->Direaction /= this->Norm;
	double dump;
	LFPB(mJointAngleBegin[3], mJointAngleEnd[3], mAVel, mAAcc, mADec,0 , dump,this->yawtimes); /* calculate yaw angle cost */
}


/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints(bool isCartesian)
{
	ofstream outfile;               			//创建文件
	outfile.open("data.txt");
	outfile << mJointAngleBegin[0] << "  "
		<< mJointAngleBegin[1] << "  "
		<< mJointAngleBegin[2] << "  "
		<< -mJointAngleBegin[3] << "  " << endl;//保存初始的时间、四个关节角度

	double dt = this->mSampleTime;
	double now = 0, t_end = dt,trans[16], A_output[4];;
	if (isCartesian)
	{
		Vector4d C_output;
		for (now; now < t_end; now += dt)
		{
			Delta_3D_LFPB(now, C_output, t_end); //采样计算，output中为三位空间插值
			trans[0] = cos(C_output(3) * PI / 180);
			trans[1] = sin(C_output(3) * PI / 180);
	
			trans[3] = C_output(0);
			trans[7] = C_output(1);
			trans[11] = C_output(2);

			robotBackward(trans,A_output);
			outfile << A_output[0] *180/PI<< "  "
				<< A_output[1] * 180 / PI << "  "
				<< A_output[2] * 180 / PI << "  "
				<<- A_output[3] * 180 / PI << "  " << endl;//保存初始的时间、六个关节角度
		}
	}
	else
	{
		for (now; now < t_end; now += dt)
		{
			Delta_AS_LFPB(now, A_output, t_end); //采样计算，output中为三位空间插值
			outfile << A_output[0]  << "  "
				<< A_output[1]<< "  "
				<< A_output[2]<< "  "
				<< -A_output[3]<< "  " << endl;//保存初始的时间、4个关节角度
		}
	}


	outfile << mJointAngleEnd[0] << "  "
		<< mJointAngleEnd[1] << "  "
		<< mJointAngleEnd[2] << "  "
		<< -mJointAngleEnd[3] << "  " << endl;//保存初始的时间、4个关节角度
	outfile.close();
}

