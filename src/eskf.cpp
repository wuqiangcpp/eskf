//
// Created by meng on 2021/2/19.
//
#include "eskf.h"

#include<iostream>
constexpr double M_PI = 3.141592653589793;
constexpr double kDegree2Radian = M_PI / 180.0;

Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vec[2], vec[1],
        vec[2], 0.0, -vec[0],
        -vec[1], vec[0], 0.0;

    return matrix;
}

ESKF::ESKF(const YAML::Node& node) {
    double gravity = node["earth"]["gravity"].as<double>();
    double earth_rotation_speed = node["earth"]["rotation_speed"].as<double>();
    double cov_prior_posi = node["covariance"]["prior"]["posi"].as<double>();
    double cov_prior_vel = node["covariance"]["prior"]["vel"].as<double>();
    double cov_prior_ori = node["covariance"]["prior"]["ori"].as<double>();
    double cov_prior_epsilon = node["covariance"]["prior"]["epsilon"].as<double>();
    double cov_prior_delta = node["covariance"]["prior"]["delta"].as<double>();
    double cov_prior_gravity=node["covariance"]["prior"]["gravity"].as<double>();
    double cov_measurement_posi = node["covariance"]["measurement"]["posi"].as<double>();
    double cov_process_vel= node["covariance"]["process"]["vel"].as<double>();
    double cov_process_ori = node["covariance"]["process"]["ori"].as<double>();
    double cov_process_gyro = node["covariance"]["process"]["gyro"].as<double>();
    double cov_process_accel = node["covariance"]["process"]["accel"].as<double>();
    double lx = node["L_gps"]["x"].as<double>();//gps coordinate in body frame of imu
    double ly= node["L_gps"]["y"].as<double>();
    double lz= node["L_gps"]["z"].as<double>();
    L_pos_ = Eigen::Vector3d(lx,ly,lz);
    L_ = node["earth"]["latitude"].as<double>();
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(L_ * kDegree2Radian),
        earth_rotation_speed * sin(L_ * kDegree2Radian));
    w_.setZero();

    SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
        cov_prior_epsilon, cov_prior_delta,cov_prior_gravity);
    SetCovarianceR(cov_measurement_posi);
    SetCovarianceQ(cov_process_vel, cov_process_ori,cov_process_gyro, cov_process_accel);

    X_.setZero();
    InitJacobian();
    double v_x= node["initial_state"]["velocity"]["x"].as<double>();
    double v_y = node["initial_state"]["velocity"]["y"].as<double>();
    double v_z = node["initial_state"]["velocity"]["z"].as<double>();
    double ori_z = node["initial_state"]["orientation"]["z"].as<double>();
    double ori_y = node["initial_state"]["orientation"]["y"].as<double>();
    double ori_x = node["initial_state"]["orientation"]["x"].as<double>();
    init_velocity_= Eigen::Vector3d(v_x, v_y, v_z);
    Eigen::Quaterniond Q = Eigen::AngleAxisd(ori_z * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ori_y * kDegree2Radian, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ori_x * kDegree2Radian, Eigen::Vector3d::UnitX());
    init_q_ = Q;
}

void ESKF::SetCovarianceQ(double vel_noise,double ori_noise,double gyro_noise, double accel_noise) {
    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * vel_noise;
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * ori_noise;
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyro_noise;
    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * accel_noise;
}

void ESKF::SetCovarianceR(double posi_noise) {
    R_.setZero();
    R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
}

void ESKF::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
    double gyro_noise, double accel_noise,double gravity_uncertainty) {
    P_.setZero();
    P_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
    P_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
    P_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise;
    P_.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
    P_.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
    //P_.block<3, 3>(INDEX_STATE_GRA, INDEX_STATE_GRA) = Eigen::Matrix3d::Identity() * gravity_uncertainty;
    P_(INDEX_STATE_GRA+2,INDEX_STATE_GRA+2) = gravity_uncertainty;
}

void ESKF::Init(const GPSData& curr_gps_data, const IMUData& curr_imu_data) {
    //init_velocity_ = Eigen::Vector3d::Zero();
    velocity_ = init_velocity_;

    //Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
    //    Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
    //    Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    //init_q_ = Q;

    q_ = init_q_;

    init_pos_ = curr_gps_data.position_enu;
    pos_ = init_pos_;

    imu_data_buff_.clear();
    imu_data_buff_.push_back(curr_imu_data);

    curr_gps_data_ = curr_gps_data;
}

void ESKF::InitJacobian() {
    Xx_.setZero();
    constexpr int dim1 = INDEX_NOMINAL_STATE_ORI - INDEX_NOMINAL_STATE_POSI;
    Xx_.block<dim1, dim1>(INDEX_NOMINAL_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix<double, dim1, dim1>::Identity();
    constexpr int dim2 = DIM_NOMINAL_STATE - INDEX_NOMINAL_STATE_GYRO_BIAS;
    Xx_.block<dim2, dim2>(INDEX_NOMINAL_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix<double, dim2, dim2>::Identity();

    Hx_.setZero();
    Hx_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_NOMINAL_STATE_POSI) = Eigen::Matrix<double, 3, 3>::Identity();

    Fi_.setZero();
    Fi_.block<DIM_STATE_NOISE, DIM_STATE_NOISE>(INDEX_STATE_VEL, 0) = TypeMatrixQ::Identity();//eq. (311)

    Fx_.setZero();
    Fx_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
    Fx_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_GRA) = Eigen::Matrix3d::Identity();
}

void ESKF::ComputeJacobianH() {
    Eigen::Matrix<double, 4, 3> Q_theta;
    Q_theta << -q_.x(), -q_.y(), -q_.z(),
        q_.w(), q_.z(), -q_.y(),
        -q_.z(), q_.w(), q_.x(),
        q_.y(), -q_.x(), q_.w();//eq. (321)
    Q_theta *= 0.5;
    Xx_.block<4, 3>(INDEX_NOMINAL_STATE_ORI, INDEX_STATE_ORI) = Q_theta;

    Eigen::Matrix<double,3, 4> J_pos_q;
    J_pos_q.block<3, 1>(0, 0) =q_.w()* L_pos_+ q_.vec().cross(L_pos_);
    J_pos_q.block<3, 3>(0, 1) = q_.vec().dot(L_pos_)*Eigen::Matrix3d::Identity()
        + q_.vec()* L_pos_.transpose()-L_pos_* q_.vec().transpose()- q_.w()* BuildSkewMatrix(L_pos_);
    Hx_.block<3, 4>(INDEX_MEASUREMENT_POSI, INDEX_NOMINAL_STATE_ORI) = 2.0*J_pos_q;//eq. (174)

    H_ = Hx_ * Xx_;
}

void ESKF::Correct(const GPSData& curr_gps_data) {
    curr_gps_data_ = curr_gps_data;

    Y_ = curr_gps_data.position_enu-(pos_+q_.toRotationMatrix()*L_pos_);

    ComputeJacobianH();

    K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse(); //eq. (273)

    P_ = (TypeMatrixP::Identity() - K_ * H_) * P_;//eq. (275)
    //X_ = X_ + K_ * (Y_ - H_ * X_);
    X_ = K_ * Y_; //as error state X_ is zero, eq. (274) degenerate to this form

    EliminateError();

    ResetState();
}

void ESKF::Predict(const IMUData& curr_imu_data) {
    imu_data_buff_.push_back(curr_imu_data);
    delta_t_= curr_imu_data.time - imu_data_buff_.front().time;

    UpdateEstimation();
    UpdateErrorState();

    imu_data_buff_.pop_front();
}


void ESKF::ComputerJacobianF() {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    Eigen::Vector3d curr_accel = q_.toRotationMatrix()*(curr_imu_data.linear_accel - accel_bias_);

    Fx_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = -BuildSkewMatrix(curr_accel);
    Fx_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = -q_.toRotationMatrix();
    Fx_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -q_.toRotationMatrix();
}

void ESKF::UpdateErrorState() {
    ComputerJacobianF();
    auto Fx = TypeMatrixFx::Identity() + Fx_ * delta_t_;//eq. (310)
    TypeMatrixQ Q = Q_ * delta_t_;
    Q.block<6, 6>(0, 0) *= delta_t_;
    //X_ = Fx * X_;
    P_ = Fx * P_ * Fx.transpose() + Fi_ * Q * Fi_.transpose();//eq. (268)
}


void ESKF::UpdateEstimation() {
    Eigen::Vector3d angular_delta;
    ComputeAngularDelta(angular_delta);

    Eigen::Matrix3d curr_R, last_R;
    ComputeOrientation(angular_delta,curr_R, last_R);

    Eigen::Vector3d curr_vel, last_vel;
    ComputeVelocity(curr_vel, last_vel, curr_R, last_R);

    ComputePosition(curr_vel, last_vel);
}

void ESKF::ComputeAngularDelta(Eigen::Vector3d& angular_delta) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);

    double delta_t = curr_imu_data.time - last_imu_data.time;

    if (delta_t <= 0) {
        return;
    }

    Eigen::Vector3d curr_angular_vel = curr_imu_data.angle_velocity;

    Eigen::Vector3d last_angular_vel = last_imu_data.angle_velocity;

    Eigen::Vector3d curr_unbias_angular_vel = curr_angular_vel- gyro_bias_;
    Eigen::Vector3d last_unbias_angular_vel = last_angular_vel- gyro_bias_;


    Eigen::Matrix3d last_R = q_.toRotationMatrix();
    Eigen::Vector3d earth_angular_vel=last_R.transpose()* w_;
    angular_delta = (0.5 * (curr_unbias_angular_vel + last_unbias_angular_vel)- earth_angular_vel) * delta_t;
}


void ESKF::ComputeOrientation(const Eigen::Vector3d& angular_delta,
    Eigen::Matrix3d& curr_R,
    Eigen::Matrix3d& last_R) {
    Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized());
    last_R = q_.toRotationMatrix();

    q_ = q_ * angle_axisd;
    curr_R = q_.toRotationMatrix();
}

void ESKF::ComputeVelocity(Eigen::Vector3d& curr_vel, Eigen::Vector3d& last_vel,
    const Eigen::Matrix3d& curr_R,
    const Eigen::Matrix3d last_R) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);
    double delta_t = curr_imu_data.time - last_imu_data.time;
    if (delta_t <= 0) {
        return;
    }

    Eigen::Vector3d curr_accel = curr_imu_data.linear_accel;
    Eigen::Vector3d curr_unbias_accel = curr_R * (curr_accel- accel_bias_ )+g_;

    Eigen::Vector3d last_accel = last_imu_data.linear_accel;
    Eigen::Vector3d last_unbias_accel = last_R * (last_accel- accel_bias_ )+g_;

    last_vel = velocity_;

    velocity_ += delta_t * 0.5 * (curr_unbias_accel + last_unbias_accel);
    curr_vel = velocity_;
}


void ESKF::ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel) {
    double delta_t = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;

    pos_ += 0.5 * delta_t * (curr_vel + last_vel);
}

void ESKF::ResetState() {
    //eq. (314) (315)
    TypeMatrixP G = TypeMatrixP::Identity();
    G.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) += 0.5* BuildSkewMatrix(X_.block<3, 1>(INDEX_STATE_ORI, 0));
    P_ = G * P_ * G.transpose();
    X_.setZero();
}



void ESKF::EliminateError() {
    //eq. (313)
    pos_ = pos_ + X_.block<3, 1>(INDEX_STATE_POSI, 0);

    velocity_ = velocity_ + X_.block<3, 1>(INDEX_STATE_VEL, 0);
    Eigen::Vector3d angular_error = X_.block<3, 1>(INDEX_STATE_ORI, 0);
    q_ = Eigen::AngleAxisd(angular_error.norm(), angular_error.normalized()) *q_;

    gyro_bias_ = gyro_bias_ + X_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
    accel_bias_ = accel_bias_ + X_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);

    g_ = g_ + X_.block<3, 1>(INDEX_STATE_GRA, 0);
    //std::cout << g_.transpose() <<" "<<g_.norm()<< std::endl;
}

ESKF::Pose ESKF::GetPose() const {
    ESKF::Pose pose;
    pose.block<3,1>(0,0) = pos_;
    pose.block<4,1>(3,0) = q_.coeffs();
    return pose;
}