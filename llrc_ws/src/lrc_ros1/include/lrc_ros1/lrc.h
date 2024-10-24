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

    // std::vector<ContourPoint> calculateCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull, int contour_point_size);
    void getfourpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud, std::string position);
    void extractPointsInUpRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position);
    void extractPointsInDownRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position);
    void extractPointsInUpLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position);
    void extractPointsInDownLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position);
    void filterUpandDownRightPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string position);
    void filterUpandDownLeftPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string position);
    Line3D getLidarLineEquation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr visualizeLine(const Line3D &line);
    Eigen::Vector3f computeLineIntersection(const Line3D &line1, const Line3D &line2);

    void projectPointCloudOnLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Line3D &line, pcl::PointCloud<pcl::PointXYZ>::Ptr &projected_cloud);
    // Eigen::Vector3f computelidarplaneCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    Eigen::Vector3f computeLidarPlaneCentroid(const cv::Mat &lidar_corner);
    std::vector<double> calculatelidar_plane_equation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud);
    std::vector<double> calculatecam_plane_equation(const cv::Mat &points);
    Line3D calculatecamLineEquation(const cv::Mat &points, int index1, int index2);
    Line3D translateLineupright(const Line3D &line, const Line3D &directionline, float distance);
    // Line3D translateLineupleft(const Line3D &line, const Line3D &directionline, float distance);

    Eigen::Vector3f computeLidarLineCentroid(const cv::Mat &lidar_corner, int index1, int index2);

    void display_colored_by_depth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &position);
    void pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, std::vector<pcl::PointIndices> &pcd_clusters);
    bool check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd);
    bool extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                           const pcl::PointIndices &cluster,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd);
    bool extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &plane_pcds,
                           const std::string &position);

    void Preexecute(const std::string &lidar_path_left, const std::string &lidar_path_right);

    ChessboardProcessResult processChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, std::string position);
    // ChessboardProcessResult processChessboard_right(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);
    Eigen::Vector3f computeCenterWithPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void visualizePointClouds(pcl::visualization::PCLVisualizer &viewer,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected,
                              const Line3D &upRightLineEquation,
                              const Line3D &downRightLineEquation,
                              const Line3D &downLeftLineEquation,
                              const Line3D &upLeftLineEquation,
                              int viewer_id);

    void visualizeMultiplePointClouds(const std::vector<ChessboardProcessResult> &results);

    // radar function
    void bounds_callback(LRC_ROS::boundsConfig &config, uint32_t level);
    void vRadarCallback(const radar_msgs::RadarObjectList &radar_msg);
    
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> lidarprocess(std::string lidar_position,std::string lrc_path);

    Line3D upRightCamLineEquation, downRightCamLineEquation, downLeftCamLineEquation, upLeftCamLineEquation;
    pcl::PointCloud<pcl::PointXYZ>::Ptr linea, lineb, linec, lined;

    Eigen::Vector3f r_estimate_vector_radian, r_estimate_vector_degree;
    double T_error, R_error, Reproject_error;

    int boardwidth = 850;
    int boardlength = 1200;
    int squaresize = 120;
    // std::string kuangshan;
    int viewer_id = 0;
    std::string city, radar_position, lidar_position, lrc_path;
    LRC_ROS::boundsConfig bound_;
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> base_pc;
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> topic_cloud;

    ~LRC();

    // std::string radar_positin, lidar_position, city;
};