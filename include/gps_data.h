//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_DATA_H
#define GPS_IMU_FUSION_GPS_DATA_H

#include "LocalCartesian.hpp"
#include <eigen3/Eigen/Core>
#include<iostream>

class GPSData{
public:
    GPSData() = default;

    Eigen::Vector3d position_lla = Eigen::Vector3d::Zero();//LLA, unit: degree
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();//ENU
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero();

    double time = 0.0;

    void print() {
        std::cout << time << "\t" << position_lla.x() << "\t" << position_lla.y() << "\t" << position_lla.z() << std::endl;
    }
};

#endif //GPS_IMU_FUSION_GPS_DATA_H
