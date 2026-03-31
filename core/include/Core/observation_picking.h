#pragma once

#include <Core/structures.h>
#include <Core/transformations.h>

#include <Eigen/Eigen>
#include <map>
#include <nlohmann/json.hpp>

struct PointInsideIntersection
{
    double x_local;
    double y_local;
    double z_local;
    int index_scan;
    std::string source_filename;
    Eigen::Vector3d color;
};

class Intersection
{
public:
    Intersection() = default;
    ~Intersection() = default;

    float color[3];
    float translation[3];
    float rotation[3];
    float width_length_height[3];
    std::vector<PointInsideIntersection> points;

    void render();
};

class ObservationPicking
{
public:
    ObservationPicking() = default;
    ~ObservationPicking() = default;

    bool is_observation_picking_mode = false;
    float picking_plane_height = 0.0f;
    float picking_plane_threshold = 0.1f;
    float max_xy = 200.0f;
    int point_size = 1;
    // bool high_density_grid = false;
    bool grid10x10m = false;
    bool grid1x1m = false;
    bool grid01x01m = false;
    bool grid001x001m = false;
    void render();
    void add_picked_to_current_observation(int index_picked, Eigen::Vector3d p);
    void accept_current_observation(const std::vector<Eigen::Affine3d>& m_poses);
    void import_observations(const std::string& filename);
    void export_observation(const std::string& filename);
    void add_intersection(const Eigen::Vector3d& translation);

    std::map<int, Eigen::Vector3d> current_observation;
    std::vector<std::map<int, Eigen::Vector3d>> observations;
    std::vector<Intersection> intersections;
    float label_dist = 100.0f;
};
