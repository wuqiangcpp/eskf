//
// Created by meng on 2021/2/24.
//
#include "eskf_flow.h"


#include <fstream>
#include <yaml-cpp/yaml.h>


ESKFFlow::ESKFFlow(const std::string &work_space_path)
        : work_space_path_(work_space_path){

    std::string config_file_path = work_space_path_ + "/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    eskf_ptr_ = std::make_shared<ESKF>(config_node);
    Eigen::Vector3d lla0;
    lla0[0] = config_node["ref_lla"]["lat"].as<double>();
    lla0[1] = config_node["ref_lla"]["lon"].as<double>();
    lla0[2] = config_node["ref_lla"]["alti"].as<double>();
    imu_freq_= config_node["imu_freq"].as<double>();
    imu_file_= config_node["input_file"]["imu"].as<std::string>();
    gps_file_ = config_node["input_file"]["gps"].as<std::string>();
    output_file_ = config_node["output_file"].as<std::string>();
    gps_flow_ptr_->setRef(lla0);

    start_time_= config_node["start_time"].as<double>();
    end_time_=config_node["end_time"].as<double>();
}

void ESKFFlow::ReadData() {
    const std::string data_path = work_space_path_ + "/data/";

    imu_flow_ptr_->ReadIMUData(data_path + imu_file_, imu_data_buff_,start_time_,end_time_);
    gps_flow_ptr_->ReadGPSData(data_path + gps_file_, gps_data_buff_,start_time_,end_time_);
}

bool ESKFFlow::ValidGPSAndIMUData() {
    curr_imu_data_ = imu_data_buff_.front();
    curr_gps_data_ = gps_data_buff_.front();

    double delta_time = curr_imu_data_.time - curr_gps_data_.time;

    double delta_imu_t = 1. / imu_freq_;

    if (curr_gps_data_.time < start_time_) {
        gps_data_buff_.pop_front();
        return false;
    }

    if (delta_time > delta_imu_t){
        gps_data_buff_.pop_front();
        return false;
    }

    if (delta_time < -delta_imu_t){
        imu_data_buff_.pop_front();
        return false;
    }

    imu_data_buff_.pop_front();
    gps_data_buff_.pop_front();

    return true;
}


void ESKFFlow::Run() {
    ReadData();

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        if (!ValidGPSAndIMUData()){
            continue;
        } else{
            eskf_ptr_->Init(curr_gps_data_, curr_imu_data_);
            std::cout << "start time: " << curr_gps_data_.time << std::endl;
            break;
        }
    }

    std::ofstream fused_file(work_space_path_+"/data/"+ output_file_, std::ios::trunc | std::ios::binary);
    std::cout << "start processing ..." << std::endl;
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
        if (curr_imu_data_.time < curr_gps_data_.time){
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();
        } else if(std::abs(curr_imu_data_.time - curr_gps_data_.time)< 1. / imu_freq_){
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();

            eskf_ptr_->Correct(curr_gps_data_);

            SavePose(fused_file, curr_gps_data_.time,eskf_ptr_->GetPose());

            gps_data_buff_.pop_front();
        }

    }
    std::cout << "end time: " << curr_gps_data_.time << std::endl;
    std::cout << "end processing." << std::endl;

}

void ESKFFlow::TestRun() {
    ReadData();

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        if (!ValidGPSAndIMUData()) {
            continue;
        } else {
            eskf_ptr_->Init(curr_gps_data_, curr_imu_data_);
            std::cout << "start time: " << curr_gps_data_.time << std::endl;
            break;
        }
    }
    std::ofstream fused_file(work_space_path_ + "/data/"+output_file_, std::ios::trunc | std::ios::binary);
    std::cout << "start processing ..." << std::endl;
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();
            SavePose(fused_file, curr_imu_data_.time, eskf_ptr_->GetPose());
    }
    std::cout << "end time: " << curr_gps_data_.time << std::endl;
    std::cout << "end processing." << std::endl;
}

void ESKFFlow::SavePose(std::ofstream &ofs, double time, const ESKF::Pose &pose) {
    ofs.write((char*)&time,sizeof(double));
    ofs.write((char*)pose.data(), pose.size() * sizeof(ESKF::Pose::Scalar));
}

