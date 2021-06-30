//
// Created by zc on 2021/6/29.
//
#include <iostream>
#include <Eigen/Dense>
#include "ekf_base.hpp"
#include "ekf.h"
#include <fstream>
#include <vector>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Path.h>
#include <mutex>

using namespace std;
using namespace Eigen;

// TODO: change to ROS

const auto ANGLE_TO_RADIAN = float {M_PI / 180.0};

EKF ekf;
ofstream odata;
bool init_ = false;
Eigen::Vector3d refllh;
ros::Publisher result_pub;
ros::Publisher gps_pub;
nav_msgs::Path path;
nav_msgs::Path path_gps;
mutex callback_mutex;

void publish_path(const vector<double>& result) {
    if (!odata.is_open()) {
        return;
    }

    if (result.empty()) {
        return;
    }

    path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = result[1];
    pose.pose.position.y = result[2];
    pose.pose.position.z = result[3];

    path.poses.push_back(pose);
    result_pub.publish(path);

    for(const auto& it : result) {
        odata << it << "    " ;
        //std::cout << *it << "    " ;
    }

    odata << endl;
    return;
}

void gps_callback (const sensor_msgs::NavSatFix::ConstPtr& msg) {
    std::unique_lock<std::mutex> lock(callback_mutex);

    Eigen::Vector3d llh;
    llh <<  msg->latitude * ANGLE_TO_RADIAN, msg->longitude * ANGLE_TO_RADIAN, msg->altitude;

    //ROS_INFO_STREAM("llh: " << llh);

    ImuData imu;
    imu.gyro   = Vector3d(0, 0,	0);
    imu.acc  = Vector3d(0, 0, -10);
    imu.dt_ms = 0.01;

    if (!init_) {
        refllh = llh;
        init_ = ekf.Init(imu, 0, refllh);

        if (!init_) {
            ROS_ERROR_STREAM("EKF init failed: " << refllh);
        }

        return;
    }

    Eigen::Vector3d ned;

    if (!CoordUtils::LLH2NED(llh, refllh, &ned)) {
        ROS_ERROR("LLH2NED failed");
        return;
    }

    GpsData gps;
    gps.pos_ned = ned;
    gps.pos_sd = Vector3d(0.2, 0.2, 0.2);
    gps.yaw = 0;
    gps.yaw_sd = 0;
    gps.vel_ned = Vector3d(0, 0, 0);
    gps.vel_sd = Vector3d(0, 0, 0);
    gps.sat_cnt = 13;
    gps.valid = 1;

    ekf.GpsMeas(gps);

    std::vector<double> result;

    ekf.RunOnce(result);

    publish_path(result);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = ned.x();
    pose.pose.position.y = ned.y();
    pose.pose.position.z = ned.z();

    path_gps.poses.push_back(pose);
    gps_pub.publish(path_gps);
};

void imu_callback (const geometry_msgs::AccelStamped::ConstPtr& msg) {
    std::unique_lock<std::mutex> lock(callback_mutex);

    if (!init_) {
        return;
    }

    ImuData imu;

    Eigen::Vector3d gyro(msg->accel.angular.x, msg->accel.angular.y, msg->accel.angular.z);
    Eigen::Vector3d acc(msg->accel.linear.x, msg->accel.linear.y, -msg->accel.linear.z);

    ROS_INFO_STREAM("GYRO: " << gyro);
    ROS_INFO_STREAM("ACC: " << acc);

    imu.gyro = gyro;
    imu.acc = acc;
    imu.dt_ms = 0.01;

    ekf.ImuMeas(imu);

    std::vector<double> result;
    ekf.RunOnce(result);

    publish_path(result);
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ekf_ros_test");

    ros::NodeHandle nh;
    result_pub = nh.advertise<nav_msgs::Path>("/path",5);
    gps_pub = nh.advertise<nav_msgs::Path>("/gps_path",5);

    odata.open("result.txt");

    path.header.frame_id = "base_link";
    path_gps.header.frame_id = "base_link";

    ros::Subscriber gps_sub = nh.subscribe("/sensor/gnss/gps_fix", 1, &gps_callback);
    ros::Subscriber imu_sub = nh.subscribe("/sensor/gnss/imu", 1, &imu_callback);

    ROS_INFO("start");

    ros::spin();

    odata.close();
    return 0;
}
