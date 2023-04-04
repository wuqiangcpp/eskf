//
// Created by meng on 2021/2/24.
//
#include "eskf_flow.h"

int main(int argc, char** argv){




    ESKFFlow eskf_flow(".");

    ////使用该函数时相当于只使用IMU位姿解算
    ////eskf_flow.TestRun();//only predict

    eskf_flow.Run();

    getchar();
    return 0;
}
