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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.hpp>
#include "opencv2/aruco.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/calib3d.hpp>
#include <json/json.h>
#include <unordered_map>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_cloud;
    Line3D upRightLineEquation;
    Line3D downRightLineEquation;
    Line3D downLeftLineEquation;
    Line3D upLeftLineEquation;
    Eigen::Vector3d uprightlidarcorner;
    Eigen::Vector3d downrightlidarcorner;
    Eigen::Vector3d downleftlidarcorner;
    Eigen::Vector3d upleftlidarcorner;
    Eigen::Vector3d uprightcentroid;
    Eigen::Vector3d downrightcentroid;
    Eigen::Vector3d downleftcentroid;
    Eigen::Vector3d upleftcentroid;
    Eigen::Vector3d planecentroid;
    Eigen::Vector4f planelidar_equation;
};

class LLC
{
private:
    ros::NodeHandle nh;
    ros::Publisher chatter_pub;

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
    LLC(ros::NodeHandle nh);

    void execute();

    double *converto_imgpts(double x, double y, double z, cv::Mat &cameraMatrix);

    void extractROI(const sensor_msgs::Image::ConstPtr &img, cv::Mat &corner_points, cv::Mat &cameraMatrix);
    void getfourpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud);
    void extractPointsInUpRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    void extractPointsInDownRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    void extractPointsInUpLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    void extractPointsInDownLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    void filterUpandDownRightPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void filterUpandDownLeftPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
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
    Line3D translateLineupleft(const Line3D &line, const Line3D &directionline, float distance);
    Eigen::Matrix3f init_estimate_R_one_pose(const std::vector<double> &camplane_equation, const std::vector<double> &lidarplane_equation,
                                             const Line3D &upRightCamLineEquation, const Line3D &downRightCamLineEquation, const Line3D &downLeftCamLineEquation, const Line3D &upLeftCamLineEquation,
                                             const Line3D &upRightLineEquation, const Line3D &downRightLineEquation, const Line3D &downLeftLineEquation, const Line3D &upLeftLineEquation,
                                             Eigen::Vector3f &r_estimate_vector_radian, Eigen::Vector3f &r_estimate_vector_degree);
    Eigen::Vector3f init_estimate_t_one_pose(const std::vector<double> &camplane_equation, const Line3D &upRightCamLineEquation,
                                             const Line3D &downRightCamLineEquation, const Line3D &downLeftCamLineEquation, const Line3D &upLeftCamLineEquation,
                                             const Eigen::Matrix3f &estimated_rotation_matrix, const Eigen::Vector3f &planecentroid, const Eigen::Vector3f &uprightcentroid,
                                             const Eigen::Vector3f &downrightcentroid, const Eigen::Vector3f &downleftcentroid, const Eigen::Vector3f &upleftcentroid);
    Eigen::Matrix3f calculate_A(const Eigen::Vector3f &line_direction);
    Eigen::Vector3f computeLidarLineCentroid(const cv::Mat &lidar_corner, int index1, int index2);

    void pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &kuangshan);
    void pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, std::vector<pcl::PointIndices> &pcd_clusters);
    bool check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd);
    bool extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                           const pcl::PointIndices &cluster,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd);
    bool extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &plane_pcds,
                           const std::string &kuangshan);
                           
    ChessboardProcessResult processChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);


    Line3D upRightLineEquation,
        downRightLineEquation, downLeftLineEquation, upLeftLineEquation;
    Line3D upRightCamLineEquation, downRightCamLineEquation, downLeftCamLineEquation, upLeftCamLineEquation;
    pcl::PointCloud<pcl::PointXYZ>::Ptr linea, lineb, linec, lined;
    Eigen::Vector3f uprightlidarcorner, downrightlidarcorner, downleftlidarcorner, upleftlidarcorner;
    Eigen::Vector3f uprightcentroid, downrightcentroid, downleftcentroid, upleftcentroid, planecentroid;
    std::vector<double> planelidar_equation, planecam_equation;
    Eigen::Vector3f r_estimate_vector_radian, r_estimate_vector_degree;
    double T_error, R_error, Reproject_error;
    std::string image_path, lidar_path;
    cv::Mat corner_points;
    int boardwidth = 850;
    int boardlength = 1200;
    int squaresize = 120;
    std::string kuangshan;

    ~LLC();
};