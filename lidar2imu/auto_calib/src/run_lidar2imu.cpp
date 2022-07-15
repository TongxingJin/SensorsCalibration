/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "calibration.hpp"
#include "extrinsic_param.hpp"
#include "registration.hpp"
#include <iostream>

#include "logging.hpp"
#include "transform_util.hpp"
using namespace std;

int main(int argc, char **argv) {
  // if (argc != 4) {
  //   cout << "Usage: ./run_lidar2imu <lidar_pcds_dir> <poses_path> "
  //           "<extrinsic_json> "
  //           "\nexample:\n\t"
  //           "./bin/run_lidar2imu data/top_center_lidar/ "
  //           "data/NovAtel-pose-lidar-time.txt "
  //           "data/gnss-to-top_center_lidar-extrinsic.json "
  //        << endl;
  //   return 0;
  // }
  std::string base_path = argv[1];
  string lidar_pcds_dir = base_path + "/pcd/";
  string poses_path = base_path + "/ins/ins_interpolation.txt";
  string extrinsic_json = base_path + "/params/velodyne16_back_novatel_extrinsics.yaml";
  std::cout << base_path + "/params/velodyne16_back_novatel_extrinsics.yaml" << std::endl;
  string stitching_path = "stitching.pcd";
  // load extrinsic
  // Eigen::Matrix4d json_param;
  // LoadExtrinsic(extrinsic_json, json_param);
  // LOGI("Load extrinsic!");
  // // convert to lidar 2 imu
  // Eigen::Matrix4d lidar2imu_extrinsic = json_param.inverse().eval();
  // std::cout << json_param << std::endl;
  // // Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  // // Registrator registrator;
  // // registrator.LoadOdometerData(poses_path, lidar2imu);
  // // registrator.LoadLidarPCDs(lidar_pcds_dir);
  // // registrator.RegistrationByGroundPlane(transform);
  // // registrator.RegistrationByVoxelOccupancy(transform);
  // // registrator.SaveStitching(stitching_path);
  // // std::cout << "the calibration result is " << std::endl;
  // // std::cout << transform << std::endl;

  Eigen::Matrix4d lidar2imu_extrinsic;
  LoadExtrinsic(extrinsic_json, lidar2imu_extrinsic);
  // lidar2imu_extrinsic = lidar2imu_extrinsic.inverse().eval();
  // lidar2imu_extrinsic.setIdentity();
  // Eigen::AngleAxisd delta(5.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY());
  // lidar2imu_extrinsic *= Eigen::Affine3d(Eigen::Translation3d::Identity() * Eigen::Quaterniond(delta)).matrix();
  // lidar2imu_extrinsic  = (lidar2imu_extrinsic * Eigen::Affine3d(Eigen::AngleAxisd(5.0 / 180.0 * M_PI, Eigen::Vector3d::UnitX())).matrix()).eval();
  Calibrator calibrator(base_path);
  calibrator.Calibration(lidar_pcds_dir, poses_path, lidar2imu_extrinsic);

  return 0;
}