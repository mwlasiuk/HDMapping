#include <pch/pch.h>

#include <Core/observation_picking.h>

#include <GL/freeglut.h>
#include <imgui.h>

#include <nlohmann/json.hpp>

void Intersection::render()
{
    TaitBryanPose pose;
    pose.px = translation[0];
    pose.py = translation[1];
    pose.pz = translation[2];
    pose.om = rotation[0];
    pose.fi = rotation[1];
    pose.ka = rotation[2];
    Eigen::Affine3d m_pose = affine_matrix_from_pose_tait_bryan(pose);

    float& x_length = width_length_height[0];
    float& y_width = width_length_height[1];
    float& z_height = width_length_height[2];

    glColor3f(color[0], color[1], color[2]);

    Eigen::Vector3d v000(-x_length * 0.5, -y_width * 0.5, -z_height * 0.5);
    Eigen::Vector3d v100(x_length * 0.5, -y_width * 0.5, -z_height * 0.5);
    Eigen::Vector3d v110(x_length * 0.5, y_width * 0.5, -z_height * 0.5);
    Eigen::Vector3d v010(-x_length * 0.5, y_width * 0.5, -z_height * 0.5);

    Eigen::Vector3d v001(-x_length * 0.5, -y_width * 0.5, z_height * 0.5);
    Eigen::Vector3d v101(x_length * 0.5, -y_width * 0.5, z_height * 0.5);
    Eigen::Vector3d v111(x_length * 0.5, y_width * 0.5, z_height * 0.5);
    Eigen::Vector3d v011(-x_length * 0.5, y_width * 0.5, z_height * 0.5);

    Eigen::Vector3d v000t = m_pose * v000;
    Eigen::Vector3d v100t = m_pose * v100;
    Eigen::Vector3d v110t = m_pose * v110;
    Eigen::Vector3d v010t = m_pose * v010;

    Eigen::Vector3d v001t = m_pose * v001;
    Eigen::Vector3d v101t = m_pose * v101;
    Eigen::Vector3d v111t = m_pose * v111;
    Eigen::Vector3d v011t = m_pose * v011;

    glBegin(GL_LINES);

    glVertex3d(v000t.x(), v000t.y(), v000t.z());
    glVertex3d(v100t.x(), v100t.y(), v100t.z());

    glVertex3d(v100t.x(), v100t.y(), v100t.z());
    glVertex3d(v110t.x(), v110t.y(), v110t.z());

    glVertex3d(v110t.x(), v110t.y(), v110t.z());
    glVertex3d(v010t.x(), v010t.y(), v010t.z());

    glVertex3d(v010t.x(), v010t.y(), v010t.z());
    glVertex3d(v000t.x(), v000t.y(), v000t.z());

    glVertex3d(v001t.x(), v001t.y(), v001t.z());
    glVertex3d(v101t.x(), v101t.y(), v101t.z());

    glVertex3d(v101t.x(), v101t.y(), v101t.z());
    glVertex3d(v111t.x(), v111t.y(), v111t.z());

    glVertex3d(v111t.x(), v111t.y(), v111t.z());
    glVertex3d(v011t.x(), v011t.y(), v011t.z());

    glVertex3d(v011t.x(), v011t.y(), v011t.z());
    glVertex3d(v001t.x(), v001t.y(), v001t.z());

    glVertex3d(v000t.x(), v000t.y(), v000t.z());
    glVertex3d(v001t.x(), v001t.y(), v001t.z());

    glVertex3d(v100t.x(), v100t.y(), v100t.z());
    glVertex3d(v101t.x(), v101t.y(), v101t.z());

    glVertex3d(v110t.x(), v110t.y(), v110t.z());
    glVertex3d(v111t.x(), v111t.y(), v111t.z());

    glVertex3d(v010t.x(), v010t.y(), v010t.z());
    glVertex3d(v011t.x(), v011t.y(), v011t.z());

    //
    glVertex3d(v000t.x(), v000t.y(), v000t.z());
    glVertex3d(v110t.x(), v110t.y(), v110t.z());

    glVertex3d(v010t.x(), v010t.y(), v010t.z());
    glVertex3d(v100t.x(), v100t.y(), v100t.z());

    glVertex3d(v001t.x(), v001t.y(), v001t.z());
    glVertex3d(v111t.x(), v111t.y(), v111t.z());

    glVertex3d(v011t.x(), v011t.y(), v011t.z());
    glVertex3d(v101t.x(), v101t.y(), v101t.z());

    glEnd();
}

void ObservationPicking::render()
{
    if (is_observation_picking_mode)
    {
        if (grid10x10m)
        {
            glColor3f(0.7, 0.7, 0.7);
            glBegin(GL_LINES);
            for (float x = -max_xy; x <= max_xy; x += 10.0)
            {
                glVertex3f(x, -max_xy, picking_plane_height);
                glVertex3f(x, max_xy, picking_plane_height);
            }
            for (float y = -max_xy; y <= max_xy; y += 10.0)
            {
                glVertex3f(-max_xy, y, picking_plane_height);
                glVertex3f(max_xy, y, picking_plane_height);
            }
            glEnd();
        }

        if (grid1x1m)
        {
            glColor3f(0.3, 0.3, 0.3);
            glBegin(GL_LINES);
            for (float x = -max_xy; x <= max_xy; x += 1)
            {
                glVertex3f(x, -max_xy, picking_plane_height);
                glVertex3f(x, max_xy, picking_plane_height);
            }
            for (float y = -max_xy; y <= max_xy; y += 1)
            {
                glVertex3f(-max_xy, y, picking_plane_height);
                glVertex3f(max_xy, y, picking_plane_height);
            }
            glEnd();
        }

        if (grid01x01m)
        {
            glColor3f(0.1, 0.1, 0.1);
            glBegin(GL_LINES);
            for (float x = -max_xy; x <= max_xy; x += 0.1)
            {
                glVertex3f(x, -max_xy, picking_plane_height);
                glVertex3f(x, max_xy, picking_plane_height);
            }
            for (float y = -max_xy; y <= max_xy; y += 0.1)
            {
                glVertex3f(-max_xy, y, picking_plane_height);
                glVertex3f(max_xy, y, picking_plane_height);
            }
            glEnd();
        }

        if (grid001x001m)
        {
            glColor3f(0.8, 0.8, 0.8);
            glBegin(GL_LINES);
            for (float x = -max_xy; x <= max_xy; x += 0.01)
            {
                glVertex3f(x, -max_xy, picking_plane_height);
                glVertex3f(x, max_xy, picking_plane_height);
            }
            for (float y = -max_xy; y <= max_xy; y += 0.01)
            {
                glVertex3f(-max_xy, y, picking_plane_height);
                glVertex3f(max_xy, y, picking_plane_height);
            }
            glEnd();
        }
    }

    glColor3f(1, 1, 1);
    glPointSize(5);
    glBegin(GL_POINTS);
    for (const auto& [key, value] : current_observation)
    {
        glVertex3f(value.x(), value.y(), value.z());
    }
    glEnd();

    glColor3f(1.0, 0.2, 0.2);
    glBegin(GL_LINES);
    for (const auto& [key1, value1] : current_observation)
    {
        for (const auto& [key2, value2] : current_observation)
        {
            if (key1 != key2)
            {
                glVertex3f(value1.x(), value1.y(), value1.z());
                glVertex3f(value2.x(), value2.y(), value2.z());
            }
        }
    }
    glEnd();

    for (auto& i : intersections)
    {
        i.render();
    }

    glColor3f(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < intersections.size(); i++)
    {
        glBegin(GL_LINES);
        glVertex3f(intersections[i].translation[0], intersections[i].translation[1], intersections[i].translation[2]);
        glVertex3f(label_dist - i * 2, intersections[i].translation[1], intersections[i].translation[2]);
        glEnd();

        glRasterPos3f(label_dist + 2 - i * 2, intersections[i].translation[1], intersections[i].translation[2]);
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, std::to_string(i).c_str()[0]);
    }
}

void ObservationPicking::add_picked_to_current_observation(int index_picked, Eigen::Vector3d p)
{
    std::cout << "inserting observation: point cloud[" << index_picked << "], coordinates [" << p.x() << "," << p.y() << "," << p.z() << "]"
              << std::endl;

    // current_observation.insert_or_assign({ index_picked, p });
    auto [iterator, inserted] = current_observation.try_emplace(index_picked, p);
    if (!inserted)
    {
        iterator->second = p;
    }

    std::cout << "current_observation" << std::endl;
    for (const auto& [key, value] : current_observation)
    {
        std::cout << '[' << key << "] = " << value.x() << " " << value.y() << " " << value.z() << std::endl;
    }
}

void ObservationPicking::accept_current_observation(const std::vector<Eigen::Affine3d>& m_poses)
{
    for (auto& [key, value] : current_observation)
    {
        // glVertex3f(value.x(), value.y(), value.z());
        value = m_poses[key].inverse() * value;
    }

    observations.push_back(current_observation);

    current_observation.clear();
    std::cout << "current_observation.size(): " << current_observation.size() << std::endl;
}

void ObservationPicking::import_observations(/*std::vector<std::map<int, Eigen::Vector3d>>& observations,*/ const std::string& filename)
{
    observations.clear();
    std::ifstream fs(filename);
    nlohmann::json data = nlohmann::json::parse(fs);
    fs.close();
    for (auto& observation_json : data["observations"])
    {
        std::map<int, Eigen::Vector3d> visible_point;
        for (auto& visible_point_json : observation_json)
        {
            const auto local_coords = visible_point_json["local_coords"];
            const int scan_id = visible_point_json["scan_id"];
            assert(local_coords.size() == 3);
            visible_point[scan_id] = Eigen::Vector3d(local_coords[0], local_coords[1], local_coords[2]);
        }
        observations.push_back(visible_point);
    }
    std::cout << "number of loaded observations: " << observations.size() << std::endl;
}

void ObservationPicking::export_observation(/*std::vector<std::map<int, Eigen::Vector3d>>& observations,*/ const std::string& filename)
{
    nlohmann::json json;
    for (const auto& observation : observations)
    {
        nlohmann::json json_obs;
        for (const auto& [scan_id, local_coords] : observation)
        {
            nlohmann::json visible_point{ { "scan_id", scan_id },
                                          { "local_coords", { local_coords.x(), local_coords.y(), local_coords.z() } } };
            json_obs.push_back(visible_point);
        }
        json["observations"].push_back(json_obs);
    }
    std::ofstream fs(filename);
    fs << json.dump(2);
    fs.close();
    std::cout << "number of saved observations: " << observations.size() << std::endl;
}

void ObservationPicking::add_intersection(const Eigen::Vector3d& translation)
{
    std::cout << "ObservationPicking::add_intersection()" << std::endl;

    Intersection intersection;
    intersection.color[0] = 1.0;
    intersection.color[1] = 0.0;
    intersection.color[2] = 0.0;
    intersection.translation[0] = (float)translation.x();
    intersection.translation[1] = (float)translation.y();
    intersection.translation[2] = (float)translation.z();
    intersection.rotation[0] = 0.0f;
    intersection.rotation[1] = 0.0f;
    intersection.rotation[2] = 0.0f;
    intersection.width_length_height[0] = 2.0;
    intersection.width_length_height[1] = 2.0;
    intersection.width_length_height[2] = 2.0;

    this->intersections.push_back(intersection);
}