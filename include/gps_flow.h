//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_FLOW_H
#define GPS_IMU_FUSION_GPS_FLOW_H

#include "gps_data.h"
#include "LocalCartesian.hpp"

#include <deque>
#include <vector>
#include <eigen3/Eigen/Dense>

class GPSFlow{
public:
    GPSFlow() :geo_converter_{ 32.0, 120.0, 0.0 }, ref_set_(false), lla_ref_(32.0, 120.0, 0.0 ){};

    void LLA2ENU(GPSData& gps_data);

    void ReadGPSData(const std::string& file, std::deque<GPSData>& gps_data_vec, double start_time, double end_time);

    void setRef(const Eigen::Vector3d&position_lla0);
    Eigen::Vector3d getRef();
private:
    GeographicLib::LocalCartesian geo_converter_;
    bool ref_set_;
    Eigen::Vector3d lla_ref_;
};

#endif //GPS_IMU_FUSION_GPS_FLOW_H
