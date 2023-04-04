//
// Created by meng on 2021/2/19.
//
#include "gps_flow.h"

#include <iostream>
#include <fstream>


void GPSFlow::setRef(const Eigen::Vector3d& lla0) {
    geo_converter_.Reset(lla0.x(), lla0.y(), lla0.z());
    lla_ref_ = lla0;
    ref_set_ = true;
}

Eigen::Vector3d  GPSFlow::getRef() {
    return lla_ref_;
}

void GPSFlow::LLA2ENU(GPSData &gps_data) {
    //lla -> ENU frame
    geo_converter_.Forward(gps_data.position_lla.x(),
                           gps_data.position_lla.y(),
                           gps_data.position_lla.z(),
                           gps_data.position_enu.x(),
                           gps_data.position_enu.y(),
                           gps_data.position_enu.z());
}

//binary file format:sequential 4 double including time, gps latidtude (degree), longtitude (degree), altitude.
void GPSFlow::ReadGPSData(const std::string &file, std::deque<GPSData>& gps_data_vec,double start_time,double end_time) {
    std::ifstream fin(file.c_str(), std::ios::binary);
    if (!fin) {
        std::cout << "cannot open file " << file << std::endl;
        return;
    }
    fin.seekg(0, fin.end);
    int length = fin.tellg();
    fin.seekg(0, fin.beg);
    std::cout << "reading " << length << " bytes of " << file << std::endl;
    std::vector<double> buffer(length / sizeof(double));
    fin.read((char*)buffer.data(), length);

    std::cout << "gps data start at " << buffer[0] << std::endl;
    std::cout << "gps data end at " << buffer[buffer.size()-4] << std::endl;
    for (int i = 0; i < buffer.size(); i+=4) {
        GPSData gps_data;
        gps_data.time = buffer[i];
        if (gps_data.time < start_time) continue;
        if (end_time > 0  && (gps_data.time > end_time) ) break;
        gps_data.position_lla = Eigen::Vector3d(&(buffer[i+1])) ;//degree
        if (!ref_set_) {
            setRef(gps_data.position_lla);
        }
        LLA2ENU(gps_data);
        //std::cout << gps_data.position_enu.transpose() << std::endl;
        gps_data_vec.push_back(gps_data);
    }
}