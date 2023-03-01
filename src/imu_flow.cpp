//
// Created by meng on 2021/2/19.
//
#include "tool.h"
#include "imu_flow.h"
#include <fstream>
#include <iostream>

//binary file format:sequential 7 double including time, angle_velocity (rad/s), linear_accel(m/s^2).
void IMUFlow::ReadIMUData(const std::string &file, std::deque<IMUData> &imu_data_buff, double start_time, double end_time) {
    //readIMU_G370(file, imu_data_buff, start_time, end_time);
    std::ifstream fin(file.c_str(), std::ios::binary);
    if (!fin) {
        std::cout << "cannot open file " << file << std::endl;
        return;
    }
    const size_t N = 7;
    fin.seekg(0, fin.end);
    int length = fin.tellg();
    fin.seekg(0, fin.beg);
    std::cout << "reading " << length << " bytes of " << file << std::endl;
    std::vector<double> buffer(length / sizeof(double));
    fin.read((char*)buffer.data(), length);

    std::cout << "imu data start at " << buffer[0] << std::endl;
    std::cout << "imu data end at " << buffer[buffer.size() - N] << std::endl;

    for (int i = 0; i < buffer.size(); i+=N) {
        IMUData imu_data;
        imu_data.time = buffer[i];
        if (imu_data.time < start_time) continue;
        if (end_time > 0 && (imu_data.time > end_time)) break;
        imu_data.angle_velocity = Eigen::Vector3d(&buffer[i+1]);//rad / s
        imu_data.linear_accel = Eigen::Vector3d(&buffer[i+4]);//m / s ^ 2
        imu_data_buff.push_back(imu_data);
    }
}




void readIMU_G370(const std::string& file, std::deque<IMUData>& imu_data_buff, double start_time, double end_time) {
    std::ifstream fin(file.c_str(), std::ios::binary);
    if (!fin) {
        std::cout << "cannot open file " << file << std::endl;
        return;
    }
    fin.seekg(0, fin.end);
    int length = fin.tellg();
    fin.seekg(0, fin.beg);
    std::cout << "reading " << length << " bytes of " << file << std::endl;
    std::vector<imu_g370> buffer(length / sizeof(imu_g370));
    fin.read((char*)buffer.data(), length);
    const double G = 9.80665;// m / s ^ 2
    const double D2R = 0.017453292519943295;
    double dGyroScaleFactor = 1.0 / 66 / 65536 * D2R;
    double dAccelScaleFactor = 1.0 / 2.5 / 65536 * G / 1000;

    std::cout << "imu data start at " << buffer[0].time << std::endl;
    std::cout << "imu data end at " << buffer[buffer.size() - 1].time << std::endl;

    for (int i = 0; i < buffer.size(); i++) {
        IMUData imu_data;
        imu_data.time = buffer[i].time;
        if (imu_data.time < start_time) continue;
        if (end_time > 0 && (imu_data.time > end_time)) break;
        imu_data.angle_velocity = Eigen::Vector3i(buffer[i].gyro).cast<double>()* dGyroScaleFactor;//rad / s
        imu_data.linear_accel = Eigen::Vector3i(buffer[i].a).cast<double>()* dAccelScaleFactor;//m / s ^ 2
        imu_data_buff.push_back(imu_data);
    }
}