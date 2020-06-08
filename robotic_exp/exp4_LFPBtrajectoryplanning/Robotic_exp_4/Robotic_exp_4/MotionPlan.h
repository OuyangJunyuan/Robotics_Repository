#pragma once
#include <vector>
#include "Eigen/Dense"
using namespace Eigen;
using namespace std;

class PosClass //单位°
{
public:
	PosClass();
	PosClass(double x, double y, double z, double yaw, double pitch, double roll);
	~PosClass();
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
	bool config[3]{1,1,1};	// config, 表示机器人姿态

};

class CHLMotionPlan
{
private:


	double mJointAngleBegin[4];					//起始点位的关节角度,	单位度
	double mJointAngleEnd[4];					//结束点位的关节角度，	单位度

	double mStartMatrixData[16];				//起始点位的转换矩阵数组  单位m
	double mEndMatrixData[16];					//结束点位的转换矩阵数组  单位m

	Vector3d mStartPointData;
	Vector3d mEndPointData;
	Vector3d Direaction;
	double Norm,yawtimes;

	double mSampleTime;							//采样点位，单位S

	double mVel;								//速度，	单位m/s
	double mAcc;								//加速度，	单位m/s/s
	double mDec;								//减速度，	单位m/s/s

	double mAVel;								//速度,		单位d/s
	double mAAcc;								//加速度，	单位d/s/s
	double mADec;								//减速度，	单位d/s/s

	bool mConfig[3];							//机器人姿态

	
	void LFPB(double bPos, double ePos, double vel, double acc, double dec, double now, double& output,double &t);
	void Delta_AS_LFPB(double now, double* output, double& t_max);
	void Delta_3D_LFPB(double now, Vector4d& output, double& t);
	
public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();
	void SetSampleTime(double sampleTime);		//设置采样时间
	void SetPlanPoints(PosClass startPos, PosClass endPos);		//输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double vel, double acc, double dec,double avel,double aacc,double adec);			//设置运动参数，速度、加速度和减速度
	void GetPlanPoints(bool isCartesian);			//获取轨迹规划后离散点位

};

