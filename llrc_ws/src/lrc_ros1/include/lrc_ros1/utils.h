#ifndef LZ4_STREAM_T_DEFINED
#define LZ4_STREAM_T_DEFINED
#endif

#ifndef LZ4_STREAMDECODE_T_DEFINED
#define LZ4_STREAMDECODE_T_DEFINED
#endif

#pragma once

// #include "radarshow_core.h"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>
#include <experimental/filesystem>
// #define gravity 9.81

inline Eigen::MatrixXf read_lidar_initial_value(std::string path, std::string city, std::string position)
{
    Eigen::MatrixXf matrix(4, 4);

    YAML::Node first_node = YAML::LoadFile(path);
    YAML::Node second_node = first_node[city];

    // int type = second_node["type"].as<int>();

    YAML::Node third_node = second_node["extrinsic"];

    std::vector<double> flat_matrix = third_node[position].as<std::vector<double>>();
    if (flat_matrix.size() == 16)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                matrix(i, j) = flat_matrix[i * 4 + j];
            }
        }
    }
    return matrix;
}
