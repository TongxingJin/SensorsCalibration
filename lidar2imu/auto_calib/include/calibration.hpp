/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "common/Lidar_parser_base.h"

class Calibrator {
public:
  Calibrator(std::string base_path);
  ~Calibrator();

  // load data
  void LoadTimeAndPoes(const std::string &filename, const Eigen::Matrix4d &Tl2i,
                       std::vector<std::string> &lidarTimes,
                       std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &lidarPoses);

  void LoadInsTimeAndPose();

  Eigen::Matrix4d GetDeltaTrans(double R[3], double t[3]);

  void Calibration(const std::string lidar_path, const std::string odom_path,
                   const Eigen::Matrix4d init_Tl2i);
  void SaveStitching(const Eigen::Matrix4d transform,
                     const std::string pcd_name);
  void SaveStitchingUndistortion(const Eigen::Matrix4d transform,
                               const std::string pcd_name);

  Eigen::Matrix4d inter(Eigen::Matrix4d pose1, Eigen::Matrix4d pose2, double ratio){
    Eigen::Affine3d p1(pose1);
    Eigen::Affine3d p2(pose2);
    Eigen::Quaterniond q = Eigen::Quaterniond(p1.linear()).slerp(ratio, Eigen::Quaterniond(p2.linear()));
    Eigen::Translation3d p(p1.translation().matrix() * (1 - ratio) + p2.translation().matrix() * ratio);
    return Eigen::Affine3d(p * q).matrix();
  }

private:
  int turn_ = 35;
  int window_ = 10;
  std::vector<std::string> lidar_files_;
  std::vector<double> lidar_timestamps_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> lidar_poses_;
  // std::vector<pcl::PointCloud<LidarPointXYZIRT>> pcd_seq_;
  double degree_2_radian = 0.017453293;
  std::string lidar_path_;

  std::string base_path_;
  bool initialized_ = false;
  Eigen::Affine3d initial_pose_;
  Eigen::Affine3d initial_extrinsic_;
  std::vector<double> ins_timestamps_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> high_freq_lidar_poses_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;