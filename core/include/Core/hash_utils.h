#pragma once

#include <Eigen/Eigen>

#include <set>

uint64_t get_index_2d(const int16_t x, const int16_t y);
uint64_t get_rgd_index_2d(const Eigen::Vector3d& p, const Eigen::Vector2d& b);

uint64_t get_index_3d(const int16_t x, const int16_t y, const int16_t z);
uint64_t get_rgd_index_3d(const Eigen::Vector3d& p, const Eigen::Vector3d& b);

int32_t get_index_in_set(const std::set<int32_t>& set, const int32_t query);