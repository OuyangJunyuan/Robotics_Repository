// Robot_Exp.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include"AProbotconfig.h"

int main()
{
    double qi[4] = { 0,0,0,0 },qo[4], output[16];

    for (int i = -39; i < 99; i++)
    {
        for (int j = 0; j < 4; j++)
            qi[j] = i * PI / 180;
        APRobot::robotForward(qi, NULL, output);
        APRobot::robotBackward(output, qo);
        cout << "_____________" << endl;
        for (int i = 0; i < 4; i++)
        {
            cout << "qi :" << qi[i] << endl;
            cout << "qo: " << qo[i] << endl;
        }
    }
    while (1)
    {
        ;
    }
}

