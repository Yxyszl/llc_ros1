#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <eigen3/Eigen/Dense>
#include <condition_variable>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/calib3d.hpp>
#include <json/json.h>
#include <unordered_map>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filter/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <thread>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <dynamic_reconfigure/server.h>
#include <lrc_ros1/boundsConfig.h>

#include "radar_msgs/RadarObject.h"
#include "radar_msgs/RadarObjectList.h"

#include <utils.h>
#include <pcl/common/pca.h>

#include <unordered_map>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ceres/ceres.h>

struct ContourPoint
{
    int index;
    double angle;
};

struct Line3D
{
    Eigen::Vector3f point;     // 直线上的一个点
    Eigen::Vector3f direction; // 直线的方向向量
};

struct ContourPointCompare
{
    bool operator()(const ContourPoint &point1, const ContourPoint &point2)
    {
        return point1.angle > point2.angle;
    }
};
// 新增一个结构体来存储处理结果
struct ChessboardProcessResult
{
    Line3D upRightLineEquation;
    Line3D downRightLineEquation;
    Line3D downLeftLineEquation;
    Line3D upLeftLineEquation;
    Eigen::Vector3f uprightcentroid;
    Eigen::Vector3f downrightcentroid;
    Eigen::Vector3f downleftcentroid;
    Eigen::Vector3f upleftcentroid;
    Eigen::Vector3f planecentroid;
    std::vector<double> planelidar_equation;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectuprightpoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectdownrightpoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectdownleftpoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectupleftpoints;
};

struct MatchedPoints
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_lidar_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_lidar_points;

    MatchedPoints()
    {
        radar_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
        left_lidar_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
        right_lidar_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
};

// 定义优化问题的残差结构
struct RadarLidarCalibrationError
{
    RadarLidarCalibrationError(
        const Eigen::Vector3d &radar_point,
        const Eigen::Vector3d &left_lidar_point,
        const Eigen::Vector3d &right_lidar_point,
        const Eigen::Matrix4d &T_right_left // 右激光到左激光的变换矩阵
        ) : radar_point_(radar_point),
            left_lidar_point_(left_lidar_point),
            right_lidar_point_(right_lidar_point),
            T_right_left_(T_right_left)
    {
    }

    template <typename T>
    bool operator()(const T *const transform_params, T *residuals) const
    {
        // transform_params: [tx, ty, tz, rx, ry, rz]
        // 雷达到左激光的变换参数

        // 1. 构建变换矩阵 T_radar_left
        Eigen::Matrix<T, 4, 4> T_radar_left = Eigen::Matrix<T, 4, 4>::Identity();

        // 1.1 旋转部分 (使用罗德里格斯公式)
        T angle_axis[3] = {transform_params[3], transform_params[4], transform_params[5]};
        Eigen::Matrix<T, 3, 3> R;
        ceres::AngleAxisToRotationMatrix(angle_axis, R.data());
        T_radar_left.template block<3, 3>(0, 0) = R;

        // 1.2 平移部分
        T_radar_left(0, 3) = transform_params[0];
        T_radar_left(1, 3) = transform_params[1];
        T_radar_left(2, 3) = transform_params[2];

        // 2. 计算雷达点在左激光系下的坐标
        Eigen::Matrix<T, 4, 1> radar_point_homogeneous;
        radar_point_homogeneous << T(radar_point_[0]), T(radar_point_[1]), T(radar_point_[2]), T(1.0);
        Eigen::Matrix<T, 4, 1> radar_in_left = T_radar_left * radar_point_homogeneous;

        // 3. 计算雷达点在右激光系下的坐标
        Eigen::Matrix<T, 4, 4> T_radar_right = T_right_left_.cast<T>().inverse() * T_radar_left;
        Eigen::Matrix<T, 4, 1> radar_in_right = T_radar_right * radar_point_homogeneous;

        // 4. 计算残差
        // 4.1 左激光残差
        residuals[0] = radar_in_left[0] - T(left_lidar_point_[0]);
        residuals[1] = radar_in_left[1] - T(left_lidar_point_[1]);
        residuals[2] = radar_in_left[2] - T(left_lidar_point_[2]);

        // 4.2 右激光残差
        residuals[3] = radar_in_right[0] - T(right_lidar_point_[0]);
        residuals[4] = radar_in_right[1] - T(right_lidar_point_[1]);
        residuals[5] = radar_in_right[2] - T(right_lidar_point_[2]);

        return true;
    }

    static ceres::CostFunction *Create(
        const Eigen::Vector3d &radar_point,
        const Eigen::Vector3d &left_lidar_point,
        const Eigen::Vector3d &right_lidar_point,
        const Eigen::Matrix4d &T_right_left)
    {
        return new ceres::AutoDiffCostFunction<RadarLidarCalibrationError, 6, 6>(
            new RadarLidarCalibrationError(radar_point, left_lidar_point, right_lidar_point, T_right_left));
    }

private:
    const Eigen::Vector3d radar_point_;
    const Eigen::Vector3d left_lidar_point_;
    const Eigen::Vector3d right_lidar_point_;
    const Eigen::Matrix4d T_right_left_;
};

class LRC
{
private:
    ros::NodeHandle nh;
    ros::Subscriber radar_front_sub;

public:
    static bool compareByZ(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
    {
        return point1.z > point2.z; // 按照 Z 值从大到小排序
    }
    static bool compareByY(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
    {
        return point1.y < point2.y; // 按照 Y 值从xiao到da排序
    }
    static bool compareByX(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
    {
        return point1.x < point2.x; // 按照 X 值从xiao到da排序
    }
    LRC(ros::NodeHandle nh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr visualizeLine(const Line3D &line);
    void projectPointCloudOnLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Line3D &line, pcl::PointCloud<pcl::PointXYZ>::Ptr &projected_cloud);
    // Eigen::Vector3f computelidarplaneCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    Eigen::Vector3f computeLidarPlaneCentroid(const cv::Mat &lidar_corner);

    void pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &position);
    void pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, std::vector<pcl::PointIndices> &pcd_clusters);
    bool extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                           const pcl::PointIndices &cluster,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd);
    bool extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &plane_pcds,
                           const std::string &position);

    Eigen::Vector3f processChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, std::string position);
    // ChessboardProcessResult processChessboard_right(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);

    // radar function
    void bounds_callback(LRC_ROS::boundsConfig &config, uint32_t level);
    void vRadarCallback(const radar_msgs::RadarObjectList &radar_msg);
    void pass_filter_radar(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &radar_position);

    // lidar function
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> lidarprocess(std::string lidar_position, std::string lrc_path,std::string city);
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> transformLidarPointClouds(
        const std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> &topic_cloud_map,
        const std::string &yaml_path,
        const std::string &city);

    Eigen::Vector3f computeCenterWithPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // calibration function
    Eigen::Matrix4d calibrateRadarToLidar(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &radar_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &left_lidar_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &right_lidar_cloud,
        const Eigen::Matrix4d &T_right_left,
        const Eigen::Matrix4d &initial_guess);

    std::string city, radar_position, lidar_position, lrc_path;
    LRC_ROS::boundsConfig bound_;
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> topic_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr po_radar_cloud;
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    ~LRC();

    // std::string radar_positin, lidar_position, city;
};