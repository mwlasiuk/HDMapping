#pragma once

// TODO(mwlasiuk) : do this at the end (print -> spdlog) as it is .h and not cpp + hpp and that breaks some subprojects ...

#include <Core/session.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

bool exportLaz(
    const std::string& filename,
    const std::vector<Eigen::Vector3d>& pointcloud,
    const std::vector<unsigned short>& intensity,
    const std::vector<double>& timestamps,
    double offset_x = 0.0,
    double offset_y = 0.0,
    double offset_alt = 0.0);

void save_processed_pc(
    const fs::path& file_path_in,
    const fs::path& file_path_put,
    const Eigen::Affine3d& m_pose,
    const Eigen::Vector3d& offset,
    bool override_compressed = false);

void points_to_vector(
    const std::vector<Point3Di> points,
    std::vector<Eigen::Affine3d>& trajectory,
    double threshold_output_filter,
    std::vector<int>* index_poses,
    std::vector<Eigen::Vector3d>& pointcloud,
    std::vector<unsigned short>& intensity,
    std::vector<double>& timestamps,
    bool use_first_pose,
    bool save_index_pose);

void save_all_to_las(const Session& session, std::string output_las_name, bool as_local, bool skip_ts_0);