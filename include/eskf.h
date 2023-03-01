//
// Created by meng on 2021/2/19.
// modified by qiang on 2023/2/28.
// reference: Joan Solà. Quaternion kinematics for the error-state Kalman filter. 2015. ffhal-01122406v5f.
//we are implementing ESKF using global angular errors, not local angular errors, see the reference starting from page 64.

#ifndef GPS_IMU_FUSION_ESKF_H
#define GPS_IMU_FUSION_ESKF_H

#include "imu_data.h"
#include "gps_data.h"

#include <deque>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

class ESKF {
public:


    ESKF(const YAML::Node &node);

    /*!
     * 用于ESKF滤波器的初始化，设置初始位姿，初始速度
     * @param curr_gps_data 与imu时间同步的gps数据
     * @param curr_imu_data 与gps时间同步的imu数据
     * @return
     */
    void Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data);

    /*!
     * 滤波器的预测，
     * @param curr_imu_data
     * @return
     */
    void Predict(const IMUData &curr_imu_data);

    /*!
     * 滤波器的矫正，
     * @param curr_gps_data
     * @return
     */
    void Correct(const GPSData &curr_gps_data);


    typedef typename Eigen::Vector<double, 7> Pose;
    Pose GetPose() const;//position and quaternian 

    Eigen::Vector3d GetVelocity(){
        return velocity_;
    }

private:
    void SetCovarianceQ(double vel_noise, double ori_noise, double gyro_noise, double accel_noise);
    void SetCovarianceR(double posi_noise_cov);

    void SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                        double gyro_noise, double accel_noise,double gravity_uncertainty);

    /*!
     * 通过IMU计算位姿和速度
     * @return
     */
    void UpdateEstimation();

    void UpdateErrorState();

    void ComputeAngularDelta(Eigen::Vector3d &angular_delta);

    void InitJacobian();//initialize jacobian where value do not change
    void ComputeJacobianH();
    void ComputerJacobianF();

    /*!
     * 通过IMU计算当前姿态
     * @param angular_delta
     * @param R_nm_nm_1
     * @param curr_R
     * @param last_R
     * @return
     */
    void ESKF::ComputeOrientation(const Eigen::Vector3d& angular_delta,
        Eigen::Matrix3d& curr_R,
        Eigen::Matrix3d& last_R);

    void ComputeVelocity(Eigen::Vector3d &curr_vel,
                         Eigen::Vector3d &last_vel,
                         const Eigen::Matrix3d &curr_R,
                         const Eigen::Matrix3d last_R);



    /*!
     * 通过imu计算当前位移
     * @param curr_vel
     * @param last_vel
     * @return
     */
    void ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel);

    /*!
     * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
     */
    void EliminateError();

    /*!
     * 每次矫正之后，需要重置状态变量X
     */
    void ResetState();

private:
    static const unsigned int DIM_STATE = 18;//error state, including error of position, velocity, oritation (delta_theta), gyro_bias, acc_bias, gravity
    static const unsigned int DIM_STATE_NOISE = 12;//error state noise, noise of velocity, oritation, gyro_bias, acc_bias


    static const unsigned int INDEX_STATE_POSI = 0;
    static const unsigned int INDEX_STATE_VEL = 3;
    static const unsigned int INDEX_STATE_ORI = 6;
    static const unsigned int INDEX_STATE_GYRO_BIAS = 9;
    static const unsigned int INDEX_STATE_ACC_BIAS = 12;
    static const unsigned int INDEX_STATE_GRA = 15;

    static const unsigned int DIM_NOMINAL_STATE = 19;//nominal state, including position, velocity, oritation (quaternion), gyro_bias, acc_bias, gravity
    static const unsigned int INDEX_NOMINAL_STATE_POSI = 0;
    static const unsigned int INDEX_NOMINAL_STATE_VEL = 3;
    static const unsigned int INDEX_NOMINAL_STATE_ORI = 6;
    static const unsigned int INDEX_NOMINAL_STATE_GYRO_BIAS = 10;
    static const unsigned int INDEX_NOMINAL_STATE_ACC_BIAS = 13;
    static const unsigned int INDEX_NOMINAL_STATE_GRA = 16;

    static const unsigned int DIM_MEASUREMENT = 3;
    static const unsigned int INDEX_MEASUREMENT_POSI = 0;

    typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixFx;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixFi;
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixH;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_NOMINAL_STATE> TypeMatrixHx;
    typedef typename Eigen::Matrix<double, DIM_NOMINAL_STATE, DIM_STATE> TypeMatrixXx;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

    TypeVectorX X_;// error state
    TypeVectorY Y_;// measurement 
    TypeMatrixFx Fx_;//error state transition matrix
    TypeMatrixFi Fi_;//transition matrix for error state noise
    TypeMatrixQ Q_;//error state noise covariance
    TypeMatrixP P_;//error state covariance
    TypeMatrixK K_;//kalman gain
    TypeMatrixH H_;//measurement matrix, H=Hx*Xx
    TypeMatrixHx Hx_;//jacobian of measurement with respect to true state
    TypeMatrixXx Xx_;//jacobian of true state with respect to error state
    TypeMatrixR R_;//measurement covariance


    Eigen::Vector3d init_velocity_ = Eigen::Vector3d::Zero();
    //Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d init_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond init_q_= Eigen::Quaterniond::Identity();

    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    //Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();


    double delta_t_;
    Eigen::Vector3d g_;//重力加速度
    Eigen::Vector3d w_;//地球自转角速度
    Eigen::Vector3d L_pos_;//gps antenna position in body fixed frame of imu


    GPSData curr_gps_data_;
    double L_ = 0.0;//纬度

    std::deque<IMUData> imu_data_buff_;


};


#endif //GPS_IMU_FUSION_ESKF_H