/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "calibration.hpp"
#include "BALM.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "gen_BALM_feature.hpp"
#include "logging.hpp"

struct PointXYZIT {
  float x;
  float y;
  float z;
  unsigned char intensity;
  float timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(float, timestamp, timestamp))

Calibrator::Calibrator(){

};

Calibrator::~Calibrator(){

};

void Calibrator::LoadTimeAndPoes(const std::string &filename,
                                 const Eigen::Matrix4d &Tl2i,
                                 std::vector<std::string> &lidarTimes,
                                 std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &lidarPoses) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cout << "ERROR--->>> cannot open: " << filename << std::endl;
    exit(1);
  }
  // load pose and lidar timestamp(filename)
  std::string line;
  // double max_x, max_y, max_z, min_x, min_y, min_z;
  // max_x = max_y = max_z = -INT_MAX;
  // min_x = min_y = min_z = INT_MAX;
  bool initialized = false;
  Eigen::Affine3d initial_pose;
  while (getline(file, line)) {
    std::stringstream ss(line);
    std::string timeStr;
    ss >> timeStr;
    if(timeStr == "#") continue;
    // lidarTimes.emplace_back(timeStr);
    lidar_files_.emplace_back(timeStr);
    // Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
    // ss >> Ti(0, 0) >> Ti(0, 1) >> Ti(0, 2) >> Ti(0, 3) >> Ti(1, 0) >>
    //     Ti(1, 1) >> Ti(1, 2) >> Ti(1, 3) >> Ti(2, 0) >> Ti(2, 1) >> Ti(2, 2) >>
    //     Ti(2, 3);
    double tmp;
    ss >> tmp;
    Eigen::Vector3d position;
    ss >> position.x() >> position.y() >> position.z();
    Eigen::Quaterniond q;
    ss >> q.x() >> q.y() >> q.z() >> q.w();
    std::cout << "Pose " << timeStr << ": " << std::endl;
    // std::cout << position << std::endl;
    Eigen::Affine3d current_pose = Eigen::Affine3d(Eigen::Translation3d(position) * q);
    if(!initialized){
      initial_pose.setIdentity();
      initial_pose.translation() = current_pose.translation();
      initialized = true;
    }
    
    // Eigen::Matrix4d Ti = (initial_pose.inverse() * current_pose).matrix();
    // Ti *= Tl2i;
    Eigen::Affine3d tmpPose = (initial_pose.inverse() * current_pose) * Eigen::Affine3d(Tl2i); 

    // max_x = std::max(max_x, Ti(0, 3));
    // max_y = std::max(max_y, Ti(1, 3));
    // max_z = std::max(max_z, Ti(2, 3));
    // min_x = std::min(min_x, Ti(0, 3));
    // min_y = std::min(min_y, Ti(1, 3));
    // min_z = std::min(min_z, Ti(2, 3));

    // lidarPoses.emplace_back(Ti);
    lidarPoses.emplace_back(tmpPose.matrix());    

    // if(std::atoi(timeStr.c_str()) % 5 == 0){
      // std::cout << current_pose.matrix() << std::endl;
      // std::cout << Ti.matrix() << std::endl;
    // }
  }
  std::cout << "Loaded " << lidarPoses.size() << " poses." << std::endl;
  file.close();
}

Eigen::Matrix4d Calibrator::GetDeltaTrans(double R[3], double t[3]) {
  Eigen::Matrix3d deltaR;
  double mat[9];
  // ceres::EulerAnglesToRotationMatrix(R, mat);
  ceres::AngleAxisToRotationMatrix(R, mat);
  deltaR << mat[0], mat[3], mat[6], mat[1], mat[4], mat[7], mat[2], mat[5],
      mat[8];
  // auto deltaR = Eigen::Matrix3d(
  //     Eigen::AngleAxisd(R[2], Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(R[1], Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(R[0], Eigen::Vector3d::UnitX()));
  Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
  deltaT.block<3, 3>(0, 0) = deltaR;
  deltaT(0, 3) = t[0];
  deltaT(1, 3) = t[1];
  deltaT(2, 3) = t[2];
  return deltaT;
}

void Calibrator::Calibration(const std::string lidar_path,
                             const std::string odom_path,
                             const Eigen::Matrix4d init_Tl2i) {
  lidar_path_ = lidar_path;
  auto time_begin = std::chrono::steady_clock::now();
  int turn = 35;
  int window = 10;
  //   Eigen::Matrix4d init_Tl2i = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 1> last_deltaT;
  LoadTimeAndPoes(odom_path, init_Tl2i, lidar_files_, lidar_poses_);

  Eigen::Matrix4d deltaTrans = Eigen::Matrix4d::Identity();
  SaveStitching(deltaTrans,"before.pcd");

  std::vector<int> frm_start_box;
  std::vector<int> frm_step_box;
  std::vector<int> frm_num_box;
  int upper_bound = std::min(int(lidar_files_.size()), 1000);
  int start_step = (upper_bound / 2) / turn_ - 1;
  for (int i = 0; i < turn_; i++) {
    int a = upper_bound / 2 - i * start_step - 1;
    frm_start_box.push_back(a);
    frm_step_box.push_back((upper_bound - a) / window_ - 1);
    frm_num_box.push_back(window_);
  }
  double deltaRPY[3] = {0, 0, 0};
  double deltaT[3] = {0, 0, 0};
  for (int i = 0; i < frm_start_box.size(); i++) {
    std::cout << "\n==>ROUND " << i << std::endl;
    int step = frm_step_box[i];
    int start = frm_start_box[i];
    int frmnum = frm_num_box[i];
    // The hash table of voxel map
    std::unordered_map<VOXEL_LOC, OCTO_TREE *> surf_map, corn_map;
    // build voxel_map
    OCTO_TREE::imu_transmat.clear();
    Eigen::Matrix4d deltaTrans = GetDeltaTrans(deltaRPY, deltaT);
    OCTO_TREE::voxel_windowsize = frmnum;
    int window_size = frmnum;

    for (size_t frmIdx = 0; frmIdx < frmnum; frmIdx++) {
      int real_frmIdx = start + frmIdx * step;
      std::string lidar_file_name =
          lidar_path + lidar_files_[real_frmIdx] + ".pcd";

      Eigen::Matrix4d imu_T = lidar_poses_[real_frmIdx];
      Eigen::Matrix4d refined_T = imu_T * deltaTrans;
      Eigen::Matrix4d imu_T_last = lidar_poses_[real_frmIdx - 1];// todo 有效性！
      Eigen::Matrix4d refined_T_last = imu_T_last * deltaTrans;
      Eigen::Matrix4d delta_pose = refined_T.inverse() * refined_T_last;
      // std::cout << "delta_pose: " << delta_pose << std::endl;
      Eigen::Quaterniond delta_rotation(Eigen::Affine3d(delta_pose).linear());
      Eigen::Vector3d delta_pose_trans = Eigen::Affine3d(delta_pose).translation().matrix();

      pcl::PointCloud<LidarPointXYZIRT>::Ptr cloud(
          new pcl::PointCloud<LidarPointXYZIRT>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile(lidar_file_name, *tmp_cloud) < 0) {
        std::cout << "cannot open pcd_file: " << lidar_file_name << "\n";
        exit(1);
      }else{
        // std::vector<int> line_count(33, 0);
        // todo distortion, make sure pcd continuity
        
        cloud->resize(tmp_cloud->size());
        for(int index = 0; index < tmp_cloud->size(); ++index){
          const auto& point = tmp_cloud->points[index];
          LidarPointXYZIRT p;
          // p.timestamp = -point.timestamp;
          double ratio = (p.intensity - int(p.intensity)) / 0.1;
          if(ratio > 1){
            // std::cout << "Large point timestamp: " << p.timestamp << std::endl;
            ratio = 1.0;
          }
          Eigen::Quaterniond inter_qua = Eigen::Quaterniond::Identity().slerp(ratio, delta_rotation);
          Eigen::Vector3d position = inter_qua.matrix() * Eigen::Vector3d(point.x, point.y, point.z) + delta_pose_trans * ratio;
          p.x = position.x();
          p.y = position.y();
          p.z = position.z();
          // p.x = point.x;
          // p.y = point.y;
          // p.z = point.z;
          p.intensity = float(point.intensity);
          float dis = sqrt(p.x * p.x + p.y * p.y);
          float angle = atan2(p.z, dis) / M_PI * 180.0 + 16.5;
          p.ring = int(angle);
          cloud->points[index] = p;
          // if(p.ring >= 0 & p.ring < 32){
            // line_count[p.ring]++;
          // }else{
            // line_count[32]++;
          // }
        }
        cloud->width = tmp_cloud->width;
        cloud->height = tmp_cloud->height;
        // for(int index = 0; index < line_count.size(); ++index){
        //   std::cout << line_count[index] << ", ";
        // }
        // std::cout << std::endl;
        // pcl::io::savePCDFileASCII<LidarPointXYZIRT>("/home/jin/Documents/data/calibration/round2/1/pcd_tmp/" + lidar_files_[real_frmIdx] + "_tmp.pcd", *cloud);
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_corn(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf_sharp(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf_combine(
          new pcl::PointCloud<pcl::PointXYZI>);
      // generate feature points
      // GenFeature feature;
      genPcdFeature(cloud, pl_surf, pl_surf_sharp, pl_corn);
      LOG(INFO) << "Feature size: " << pl_surf->size() << ", " << pl_surf_sharp->size();
      // Eigen::Matrix4d imu_T = lidar_poses_[real_frmIdx];
      // Eigen::Matrix4d refined_T = imu_T * deltaTrans;
      OCTO_TREE::imu_transmat.push_back(imu_T);
      if (i < turn / 2) {
        cut_voxel(surf_map, pl_surf_sharp, refined_T, 0, frmIdx,
                  window_size + 5);
      } else {
        cut_voxel(surf_map, pl_surf, refined_T, 0, frmIdx, window_size + 5);
      }
      // *pl_surf_combine += *pl_surf_sharp;
      // *pl_surf_combine += *pl_surf;
      // cut_voxel(surf_map, pl_surf_combine, refined_T, 0, frmIdx, window_size + 5);

      // if (i > turn / 2)
      //     cut_voxel(corn_map, pl_corn, refined_T, 1, frmIdx, window_size +
      //     5);

      // Points in new frame have been distributed in corresponding root node
      // voxel
      // Then continue to cut the root voxel until right size
      for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
        if (iter->second->is2opt) // Sliding window of root voxel should
                                  // have points
        {
          iter->second->root_centors.clear();
          iter->second->recut(0, frmIdx, iter->second->root_centors);
        }
      }

      for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
        if (iter->second->is2opt) {
          iter->second->root_centors.clear();
          iter->second->recut(0, frmIdx, iter->second->root_centors);
        }
      }
    }
    // display
    displayVoxelMap(surf_map, i);
    // optimize delta R, t1, t2
    if (i < turn / 2) {
      optimizeDeltaTrans(surf_map, corn_map, 4, deltaRPY, deltaT);
    } else {
      optimizeDeltaTrans(surf_map, corn_map, 2, deltaRPY, deltaT);
    }
    // optimizeDeltaTrans(surf_map, corn_map, 2, deltaRPY, deltaT);
    
    std::cout << "delta rpy: " << deltaRPY[0] / degree_2_radian << " "
              << deltaRPY[1] / degree_2_radian << " "
              << deltaRPY[2] / degree_2_radian << std::endl;
    std::cout << "delta T: " << deltaT[0] << " " << deltaT[1] << " "
              << deltaT[2] << std::endl;

    //  clear tree
    for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
      clear_tree(iter->second);
    }
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      clear_tree(iter->second);
    }
    std::cout << "Round Finish!\n";
  }
  double bestVal[6];
  bestVal[0] = deltaRPY[0];
  bestVal[1] = deltaRPY[1];
  bestVal[2] = deltaRPY[2];
  bestVal[3] = deltaT[0];
  bestVal[4] = deltaT[1];
  bestVal[5] = deltaT[2];
  auto time_end = std::chrono::steady_clock::now();
  std::cout << "calib cost "
            << std::chrono::duration<double>(time_end - time_begin).count()
            << "s" << std::endl;
  // save refined calib
  std::string refine_calib_file = "./refined_calib_imu_to_lidar.txt";
  // Eigen::Matrix4d deltaTrans = Eigen::Matrix4d::Identity();
  // SaveStitching(deltaTrans,"before.pcd");
  deltaTrans = GetDeltaTrans(deltaRPY, deltaT);
  // SaveStitching(deltaTrans,"after.pcd");
  SaveStitchingUndistortion(deltaTrans,"after.pcd");
  std::cout << "delta T is:" << std::endl;
  std::cout << deltaTrans << std::endl;
  auto refined_Tl2i = init_Tl2i * deltaTrans;
  std::cout << "refined T(lidar 2 imu): " << std::endl;
  std::cout << refined_Tl2i << std::endl
  // auto refined_Ti2l = refined_Tl2i.inverse().eval();
  // std::cout << "refined T(imu 2 lidar): " << std::endl;
  // std::cout << refined_Ti2l << std::endl;
;
  // std::ofstream fCalib(refine_calib_file);
  // if (!fCalib.is_open()) {
  //   std::cerr << "open file " << refine_calib_file << "failed." << std::endl;
  //   // return 1;
  // }

  // fCalib << "refined calib:" << std::endl;
  // fCalib << "R: " << refined_Ti2l(0, 0) << " " << refined_Ti2l(0, 1) << " "
  //        << refined_Ti2l(0, 2) << " " << refined_Ti2l(1, 0) << " "
  //        << refined_Ti2l(1, 1) << " " << refined_Ti2l(1, 2) << " "
  //        << refined_Ti2l(2, 0) << " " << refined_Ti2l(2, 1) << " "
  //        << refined_Ti2l(2, 2) << std::endl;
  // fCalib << "t: " << refined_Ti2l(0, 3) << " " << refined_Ti2l(1, 3) << " "
  //        << refined_Ti2l(2, 3) << std::endl;
  // fCalib << "deltaTrans:" << std::endl;
  // fCalib << deltaTrans << std::endl;
  // fCalib << "delta roll, pitch, yaw, tx, ty, tz:" << std::endl;
  // fCalib << bestVal[0] << " " << bestVal[1] << " " << bestVal[2] << " "
  //        << bestVal[3] << " " << bestVal[4] << " " << bestVal[5] << std::endl;
  // fCalib << "delta roll, pitch, yaw, tx, ty, tz from begin:" << std::endl;
  // fCalib << bestVal[0] + last_deltaT[0] << " " << bestVal[1] + last_deltaT[1]
  //        << " " << bestVal[2] + last_deltaT[2] << " "
  //        << bestVal[3] + last_deltaT[3] << " " << bestVal[4] + last_deltaT[4]
  //        << " " << bestVal[5] + last_deltaT[5] << std::endl;
  // std::cout << "save refined calib to " << refine_calib_file << std::endl;
}

void Calibrator::SaveStitching(const Eigen::Matrix4d transform,
                               const std::string pcd_name) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr all_octree(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.3));
  // pcl::PointCloud<PointXYZIT>::Ptr tmp_cloud(
  //     new pcl::PointCloud<PointXYZIT>());

  all_octree->setInputCloud(all_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < lidar_files_.size(); i++) {
    std::string lidar_file_name;
    lidar_file_name = lidar_path_ + lidar_files_[i] + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_file_name, *cloud) < 0) {
      LOGW("can not open %s", lidar_file_name);
      return;
    }
    
    // cloud->resize(tmp_cloud->size());
    // for(int index = 0; index < tmp_cloud->size(); ++index){
    //   const auto& point = tmp_cloud->points[index];
    //   auto& p= cloud->points[index];
    //   p.x = point.x;
    //   p.y = point.y;
    //   p.z = point.z;
    //   p.intensity = point.intensity;
    // }
    // cloud->width = tmp_cloud->width;
    // cloud->height = tmp_cloud->height;

    Eigen::Matrix4d T = lidar_poses_[i] * transform;
    for (const auto &src_pt : cloud->points) {
      if (!pcl_isfinite(src_pt.x) || !pcl_isfinite(src_pt.y) ||
          !pcl_isfinite(src_pt.z))
        continue;
      Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
      Eigen::Vector3d p_res;
      p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
      pcl::PointXYZI dst_pt;
      dst_pt.x = p_res(0);
      dst_pt.y = p_res(1);
      dst_pt.z = p_res(2);
      dst_pt.intensity = src_pt.intensity;
      if (!all_octree->isVoxelOccupiedAtPoint(dst_pt)) {
        all_octree->addPointToCloud(dst_pt, all_cloud);
      }
    }
  }
  pcl::io::savePCDFileASCII(pcd_name, *all_cloud);
  all_cloud->clear();
  all_octree->deleteTree();
}

void Calibrator::SaveStitchingUndistortion(const Eigen::Matrix4d transform,
                               const std::string pcd_name) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr all_octree(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.3));
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  all_octree->setInputCloud(all_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 1; i < lidar_files_.size(); i++) {
    std::string lidar_file_name = lidar_path_ + lidar_files_[i] + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_file_name, *tmp_cloud) < 0) {
      LOGW("can not open %s", lidar_file_name);
      return;
    }
    
    Eigen::Matrix4d imu_T = lidar_poses_[i];
    Eigen::Matrix4d refined_T = imu_T * transform;
    Eigen::Matrix4d imu_T_last = lidar_poses_[i - 1];// todo 有效性！
    Eigen::Matrix4d refined_T_last = imu_T_last * transform;
    Eigen::Matrix4d delta_pose = refined_T.inverse() * refined_T_last;
    Eigen::Quaterniond delta_rotation(Eigen::Affine3d(delta_pose).linear());
    Eigen::Vector3d delta_pose_trans = Eigen::Affine3d(delta_pose).translation().matrix();

    cloud->resize(tmp_cloud->size());
    for(int index = 0; index < tmp_cloud->size(); ++index){
      const auto& point = tmp_cloud->points[index];
      auto& p= cloud->points[index];
      // p.x = point.x;
      // p.y = point.y;
      // p.z = point.z;

      double ratio = (point.intensity - int(point.intensity)) / 0.1;
      if(ratio > 1){
        ratio = 1.0;
      }
      Eigen::Quaterniond inter_qua = Eigen::Quaterniond::Identity().slerp(ratio, delta_rotation);
      Eigen::Vector3d position = inter_qua.matrix() * Eigen::Vector3d(point.x, point.y, point.z) + delta_pose_trans * ratio;
      // p.x = point.x;
      // p.y = point.y;
      // p.z = point.z;
      p.x = position.x();
      p.y = position.y();
      p.z = position.z();

      p.intensity = point.intensity;
    }
    cloud->width = tmp_cloud->width;
    cloud->height = tmp_cloud->height;

    Eigen::Matrix4d T = lidar_poses_[i] * transform;
    for (const auto &src_pt : cloud->points) {
      if (!pcl_isfinite(src_pt.x) || !pcl_isfinite(src_pt.y) ||
          !pcl_isfinite(src_pt.z))
        continue;
      Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
      Eigen::Vector3d p_res;
      p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
      pcl::PointXYZI dst_pt;
      dst_pt.x = p_res(0);
      dst_pt.y = p_res(1);
      dst_pt.z = p_res(2);
      dst_pt.intensity = src_pt.intensity;
      if (!all_octree->isVoxelOccupiedAtPoint(dst_pt)) {
        all_octree->addPointToCloud(dst_pt, all_cloud);
      }
    }
  }
  pcl::io::savePCDFileASCII(pcd_name, *all_cloud);
  all_cloud->clear();
  all_octree->deleteTree();
}