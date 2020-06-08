#include <iostream>
#include "MotionPlan.h"

using namespace std;

/****
 * 实验四: 轨迹规划
 * 要 求：使用C/C++完成梯型速度规划，生成data.txt文件
 * 规划类型：关节空间、笛卡尔空间（直线）
 * 点位数量：起始点和终止点
 * 给定条件：Vel，Acc，Dec
 *
 */

int main(int argc, char* argv[])
{   
    /*-- x,y,z,yaw,pitch,roll --*/
    /*--    m,  degree --*/
    PosClass Start(251, -77.52, -605.6, 0, 180, 35);
    PosClass End(-1.138, -19.51, -645.5, 0, 180, 65);

    /*-- LFPB --*/
    CHLMotionPlan trajectory1;
    trajectory1.SetPlanPoints(Start, End);
    trajectory1.SetProfile(0.03, 0.01, 0.01, 5, 30, 30); /* m，degree */
    trajectory1.SetSampleTime(0.001);      /* s */
    
    char flag = 0;
    cout << "输入'1':笛卡尔空间规划" << endl;
    cout << "输入'2':关节空间规划" << endl;
    cin >> flag;
    if(flag == '1')
        trajectory1.GetPlanPoints(true); /* true/false */
    else
        trajectory1.GetPlanPoints(false); /* true/false */
}