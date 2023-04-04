//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_IMU_FLOW_H
#define GPS_IMU_FUSION_IMU_FLOW_H

#include "imu_data.h"

#include <vector>
#include <deque>
#include <cstdint>


#include <fstream>
#include<iostream>

class IMUFlow{
public:
    IMUFlow() = default;

    void ReadIMUData(const std::string& file,
                            std::deque<IMUData>& imu_data_buff,
                            double start_time, double end_time
                           );
};


struct imu_g370 {
    double time;
    int32_t gyro[3];
    int32_t a[3];
};
void readIMU_G370(const std::string& file, std::deque<IMUData>& imu_data_buff,double start_time, double end_time);

#endif //GPS_IMU_FUSION_IMU_FLOW_H
