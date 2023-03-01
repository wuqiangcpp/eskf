//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_FLOW_H
#define GPS_IMU_FUSION_GPS_FLOW_H

#include "gps_data.h"
#include "LocalCartesian.hpp"

#include <deque>
#include <vector>

class GPSFlow{
public:
    GPSFlow() = default;

    static void LLA2ENU(GPSData& gps_data);

    static void ReadGPSData(const std::string& file, std::deque<GPSData>& gps_data_vec, double start_time, double end_time);

    static void setRef(const Eigen::Vector3d&position_lla0);

private:
    static GeographicLib::LocalCartesian geo_converter_;
};

#endif //GPS_IMU_FUSION_GPS_FLOW_H
