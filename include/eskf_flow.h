//
// Created by meng on 2021/2/24.
//
#ifndef GPS_IMU_FUSION_ESKF_FLOW_H
#define GPS_IMU_FUSION_ESKF_FLOW_H

#include "eskf.h"
#include "imu_flow.h"
#include "gps_flow.h"

#include <memory>
#include <deque>
#include <iostream>

class ESKFFlow{
public:
    ESKFFlow() = default;
    ESKFFlow(const std::string& work_space_path);

    /*!
     * 从本地文件中读取IMU和GPS的数据
     * @return
     */
    void ReadData();

    /*!
     * 对IMU和GPS数据进行时间戳对齐，该函数只在ESKF初始化时使用
     * @return
     */
    bool ValidGPSAndIMUData();

    void Run();

    void TestRun();

    /*!
     * 保存位姿
     * @param ofs
     * @param pose
     */
    void SavePose(std::ofstream &ofs, double time,const ESKF::Pose &pose);

private:
    std::shared_ptr<ESKF> eskf_ptr_;
    std::shared_ptr<IMUFlow> imu_flow_ptr_;
    std::shared_ptr<GPSFlow> gps_flow_ptr_;


    std::deque<IMUData> imu_data_buff_;
    std::deque<GPSData> gps_data_buff_;

    IMUData curr_imu_data_;
    GPSData curr_gps_data_;

    const std::string work_space_path_;
    
    double imu_freq_;
    double start_time_,end_time_;

    std::string imu_file_;
    std::string gps_file_;
    std::string output_file_;
};

#endif //GPS_IMU_FUSION_ESKF_FLOW_H
