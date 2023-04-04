//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_IMU_DATA_H
#define GPS_IMU_FUSION_IMU_DATA_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include<iostream>



class IMUData{
public:
    IMUData() = default;

    double time = 0.0;
    Eigen::Vector3d linear_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d angle_velocity = Eigen::Vector3d::Zero();//unit: rad/s

    void print() {
        std::cout << time << "\t" << linear_accel.transpose() << "\t" << angle_velocity.transpose() << std::endl;
    }
};
#endif //GPS_IMU_FUSION_IMU_DATA_H
