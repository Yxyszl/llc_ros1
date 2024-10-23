#include "lrc.h"
#include <iostream>
#include <thread>
#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"

std::condition_variable condition_variable;

Eigen::Vector3d getPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int index)
{
    return Eigen::Vector3d(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z);
}

double distance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    return (p1 - p2).norm();
}

bool is_exit = false;

// 自定义打印矩阵的函数
void printmatrixwithcommas(const Eigen::Matrix4f &matrix)
{
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            // 设置输出精度为6位小数，并且固定格式
            std::cout << std::fixed << std::setprecision(6) << matrix(i, j);
            // 添加逗号和空格作为分隔符，最后一个元素不加
            if (j < matrix.cols() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << (i < matrix.rows() - 1 ? ",\n" : "\n"); // 每行之间添加换行符
    }
}

LRC::LRC(ros::NodeHandle nh)
{
    nh.getParam("city", city);
    nh.getParam("lidar_position", lidar_position);
    nh.getParam("radar_position", lidar_position);

    if (radar_position == "front" || radar_position == "front_right" || radar_position == "front_left")
    {
        if (city == "wuhu")
        {
            radar_front_sub = nh.subscribe("/" + radar_position + "_radar", 10, &LRC::vRadarCallback, this);


        }
        else if (city == "tongling")
        {
            // src_points_front = nh.subscribe("/front_lidar", 10, &LRC::calibration_show_front, this);
            radar_front_sub = nh.subscribe("/" + radar_position + "_radar", 10, &LRC::vRadarCallback, this);


        }
    }
    else if (radar_position == "back")
    {
        if (city == "wuhu")
        {
            // src_points_back = nh.subscribe("/back_lidar", 10, &LRC::calibration_show_back, this);
            radar_front_sub = nh.subscribe("/" + radar_position + "_radar", 10, &LRC::vRadarCallback, this);

        }
        else if (city == "tongling")
        {
            // src_points_back = nh.subscribe("/back_lidar", 10, &LRC::calibration_show_back, this);
            radar_front_sub = nh.subscribe("/" + radar_position + "_radar", 10, &LRC::vRadarCallback, this);

        }
    }
}
// radar config
void LRC::bounds_callback(LRC_ROS::boundsConfig &config, uint32_t level)
{
    bound_ = config;
}
// radar callback
void LRC::vRadarCallback(const radar_msgs::RadarObjectList &radar_msg)
{
    // 按RCS值降序排列雷达点云
    std::vector<radar_msgs::RadarObject> sortedObjects(radar_msg.ObjectList.begin(), radar_msg.ObjectList.end());
    std::sort(sortedObjects.begin(), sortedObjects.end(), [](const radar_msgs::RadarObject &a, const radar_msgs::RadarObject &b)
              { return a.RCS > b.RCS; });

    pcl::PointCloud<pcl::PointXYZI>::Ptr po_radar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // po_radar_cloud->header.frame_id = "sensor";

    for (uint i = 0; i < radar_msg.ObjNum; ++i)
    {
        pcl::PointXYZI single_point;
        single_point.x = radar_msg.ObjectList[i].Rel_Pos.x;
        single_point.y = radar_msg.ObjectList[i].Rel_Pos.y;
        po_radar_cloud->push_back(single_point);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LRC::visualizeLine(const Line3D &line)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ start_point(line.point.x(), line.point.y(), line.point.z());

    line_cloud->push_back(start_point);
    line_cloud->push_back(pcl::PointXYZ(line.point.x() + line.direction.x(),
                                        line.point.y() + line.direction.y(),
                                        line.point.z() + line.direction.z()));

    return line_cloud;
}

bool LRC::extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &plane_pcds,
                            const std::string &position)
{
    pass_filter(input_cloud, position); // 带通滤波
    std::vector<pcl::PointIndices> indices_clusters;
    std::cout << input_cloud->size() << std::endl;
    pcd_clustering(input_cloud, indices_clusters); // 聚类

    bool found_chessboard = true;

    for (std::size_t i = 0; i < indices_clusters.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::size_t j = 0; j < indices_clusters[i].indices.size(); ++j)
        {
            cluster->points.push_back(input_cloud->points[indices_clusters[i].indices[j]]);
        }

        plane_pcds.push_back(cluster);
    }
    return found_chessboard;
}

void LRC::pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &position)
{
    auto &pcd_in_roi = input_pcd;
    pcl::PassThrough<pcl::PointXYZ> filter;
    if (position.find("left") != std::string::npos)
    {
        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-3, 3);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(-10, 10);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(0, 20);
        filter.filter(*pcd_in_roi);
    }
    else if (position.find("right") != std::string::npos)
    {
        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-3, 3);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(0, 20);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-10, 10);
        filter.filter(*pcd_in_roi);
    }
}

void LRC::pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, std::vector<pcl::PointIndices> &pcd_clusters)
{
    // pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    // normal_estimator.setSearchMethod(tree);
    // normal_estimator.setInputCloud(input_pcd);
    // normal_estimator.setKSearch(120);
    // normal_estimator.compute(*normals);

    // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    // reg.setMinClusterSize(90);
    // reg.setMaxClusterSize(6000);
    // reg.setSearchMethod(tree);
    // reg.setNumberOfNeighbours(60);
    // reg.setInputCloud(input_pcd);
    // reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    // reg.setCurvatureThreshold(0.5);
    // reg.extract(pcd_clusters);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_pcd);

    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(5);
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_pcd);
    ec.extract(pcd_clusters);
}

bool LRC::check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd)
{
    double max_len = sqrt(0.85 * 0.85 + 1.2 * 1.2); // 对角线长
    max_len *= 1.5;                                 // 裕度
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*board_pcd, minPt, maxPt);

    if (maxPt.x - minPt.x > max_len)
        return false;
    if (maxPt.y - minPt.y > max_len)
        return false;
    if (maxPt.z - minPt.z > max_len)
        return false;
    return true;
}


bool LRC::extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                            const pcl::PointIndices &cluster,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd)
{
    pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new pcl::ExtractIndices<pcl::PointXYZ>);
    extract->setInputCloud(input_pcd);
    extract->setIndices(boost::make_shared<pcl::PointIndices>(cluster));
    extract->setNegative(false);
    extract->filter(*chessboard_pcd);

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(chessboard_pcd));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
    ransac.setDistanceThreshold(0.06);
    ransac.setMaxIterations(10);
    if (!ransac.computeModel())
        return false;

    std::vector<int> inliers;
    ransac.getInliers(inliers);
    Eigen::VectorXf coeff(4);
    ransac.getModelCoefficients(coeff);
    double inliers_percentage = double(inliers.size()) / chessboard_pcd->points.size();

    if (inliers_percentage > 0.8)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*chessboard_pcd, inliers, *temp_cloud);
        chessboard_pcd = temp_cloud;

        if (!check_board_size(chessboard_pcd))
            return false;

        return true;
    }

    return false;
}

// 从得到的激光边界凸出最大的十个点里，筛选出板子的四个角点，四个点顺序顺时针，板子最上面的点为第一个点
void LRC::getfourpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud, std::string position)
{
    std::cout << position << std::endl;

    if (position.find("left") != std::string::npos)
    {
        pcl::PointXYZ y_min_point, y_max_point, z_min_point, z_max_point;
        // 寻找最小和最大 y 值以及最小和最大 z 值的四个点
        y_min_point = corners_cloud->points[0];
        y_max_point = corners_cloud->points[0];
        z_min_point = corners_cloud->points[0];
        z_max_point = corners_cloud->points[0];

        for (const auto &point : corners_cloud->points)
        {
            if (point.y < y_min_point.y)
                y_min_point = point;
            if (point.y > y_max_point.y)
                y_max_point = point;

            if (point.z < z_min_point.z)
                z_min_point = point;
            if (point.z > z_max_point.z)
                z_max_point = point;
        }
        // 删除除了最小和最大 y 值以及最小和最大 z 值的四个点之外的其他点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : corners_cloud->points)
        {
            if (point.x == y_min_point.x && point.y == y_min_point.y && point.z == y_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == y_max_point.x && point.y == y_max_point.y && point.z == y_max_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_min_point.x && point.y == z_min_point.y && point.z == z_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_max_point.x && point.y == z_max_point.y && point.z == z_max_point.z)
                filtered_cloud->points.push_back(point);
        }
        // 如果 filtered_cloud 的大小大于 4，删除重复的点
        if (filtered_cloud->points.size() > 4)
        {
            std::sort(filtered_cloud->points.begin(), filtered_cloud->points.end(),
                      [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                      {
                          return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
                      });
            auto last = std::unique(filtered_cloud->points.begin(), filtered_cloud->points.end(),
                                    [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                                    {
                                        return a.x == b.x && a.y == b.y && a.z == b.z;
                                    });
            filtered_cloud->points.erase(last, filtered_cloud->points.end());
        }

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;

        std::cout << "成功提取并删除其他点！" << std::endl;

        std::sort(corners_cloud->points.begin(), corners_cloud->points.end(), compareByZ);
        if (corners_cloud->points[1].y > corners_cloud->points[2].y)
        {
            std::swap(corners_cloud->points[1], corners_cloud->points[2]);
            std::swap(corners_cloud->points[2], corners_cloud->points[3]);
        }
    }
    else if (position.find("right") != std::string::npos)
    {
        pcl::PointXYZ x_min_point, x_max_point, z_min_point, z_max_point;
        // 寻找最小和最大 x 值以及最小和最大 z 值的四个点
        x_min_point = corners_cloud->points[0];
        x_max_point = corners_cloud->points[0];
        z_min_point = corners_cloud->points[0];
        z_max_point = corners_cloud->points[0];

        for (const auto &point : corners_cloud->points)
        {
            if (point.x < x_min_point.x)
                x_min_point = point;
            if (point.x > x_max_point.x)
                x_max_point = point;

            if (point.z < z_min_point.z)
                z_min_point = point;
            if (point.z > z_max_point.z)
                z_max_point = point;
        }
        // 删除除了最小和最大 x 值以及最小和最大 z 值的四个点之外的其他点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : corners_cloud->points)
        {
            if (point.x == x_min_point.x && point.y == x_min_point.y && point.z == x_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == x_max_point.x && point.y == x_max_point.y && point.z == x_max_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_min_point.x && point.y == z_min_point.y && point.z == z_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_max_point.x && point.y == z_max_point.y && point.z == z_max_point.z)
                filtered_cloud->points.push_back(point);
        }

        // 如果 filtered_cloud 的大小大于 4，删除重复的点
        if (filtered_cloud->points.size() > 4)
        {
            std::sort(filtered_cloud->points.begin(), filtered_cloud->points.end(),
                      [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                      {
                          return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
                      });
            auto last = std::unique(filtered_cloud->points.begin(), filtered_cloud->points.end(),
                                    [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                                    {
                                        return a.x == b.x && a.y == b.y && a.z == b.z;
                                    });
            filtered_cloud->points.erase(last, filtered_cloud->points.end());
        }

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;

        // std::cout << "成功提取并删除其他点！" << "shuliang" << corners_cloud->size() << std::endl;

        std::sort(corners_cloud->points.begin(), corners_cloud->points.end(), compareByZ);
        if (corners_cloud->points[1].x < corners_cloud->points[2].x)
        {
            std::swap(corners_cloud->points[1], corners_cloud->points[2]);
            std::swap(corners_cloud->points[2], corners_cloud->points[3]);
        }

        for (size_t i = 0; i < corners_cloud->points.size(); ++i)
        {
            std::cout << "Corner " << i << ": "
                      << corners_cloud->points[i].x << ", "
                      << corners_cloud->points[i].y << ", "
                      << corners_cloud->points[i].z << std::endl;
        }
    }
}

// 过滤右上右下影响拟合直线的点
void LRC::filterUpandDownRightPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string position)
{
    // 定义阈值，用于判断Z值相近的点
    float z_threshold = 0.05;

    // 创建临时容器保存满足条件的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->points.reserve(cloud->size());

    if (position.find("left") != std::string::npos)
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            float z = point.z;
            float y = point.y;
            float x = point.x;

            // 检查是否存在Z值相近的点
            bool is_close = false;
            for (int j = 0; j < filtered_cloud->size(); ++j)
            {
                pcl::PointXYZ filtered_point = filtered_cloud->points[j];
                if (std::abs(filtered_point.z - z) <= z_threshold)
                {
                    is_close = true;
                    // 保留Y值最小的点
                    if (y < filtered_point.y)
                    {
                        filtered_cloud->points[j] = point;
                    }
                    break;
                }
            }

            // 如果不存在Z值相近的点，则将当前点添加到临时容器中
            if (!is_close)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }
    else if (position.find("right") != std::string::npos)
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            float x = point.x;
            float z = point.z;
            float y = point.y;

            // 检查是否存在Z值相近的点
            bool is_close = false;
            for (int j = 0; j < filtered_cloud->size(); ++j)
            {
                pcl::PointXYZ filtered_point = filtered_cloud->points[j];
                if (std::abs(filtered_point.z - z) <= z_threshold)
                {
                    is_close = true;
                    // 保留x值 zuida 的点
                    if (x > filtered_point.x)
                    {
                        filtered_cloud->points[j] = point;
                    }
                    break;
                }
            }

            // 如果不存在Z值相近的点，则将当前点添加到临时容器中
            if (!is_close)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }
    // 更新原始点云数据
    cloud->swap(*filtered_cloud);
}

// 过滤左上左下影响拟合直线的点
void LRC::filterUpandDownLeftPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string position)
{
    // 定义阈值，用于判断Z值相近的点
    float z_threshold = 0.05;

    // 创建临时容器保存满足条件的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->points.reserve(cloud->size());
    if (position.find("left") != std::string::npos)
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            float z = point.z;
            float x = point.x;
            float y = point.y;

            // 检查是否存在Z值相近的点
            bool is_close = false;
            for (int j = 0; j < filtered_cloud->size(); ++j)
            {
                pcl::PointXYZ filtered_point = filtered_cloud->points[j];
                if (std::abs(filtered_point.z - z) <= z_threshold)
                {
                    is_close = true;
                    // 保留x值最大的点
                    if (y > filtered_point.y)
                    {
                        filtered_cloud->points[j] = point;
                    }
                    break;
                }
            }

            // 如果不存在Z值相近的点，则将当前点添加到临时容器中
            if (!is_close)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }
    else if (position.find("right") != std::string::npos)
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            float x = point.x;
            float z = point.z;
            float y = point.y;

            // 检查是否存在Z值相近的点
            bool is_close = false;
            for (int j = 0; j < filtered_cloud->size(); ++j)
            {
                pcl::PointXYZ filtered_point = filtered_cloud->points[j];
                if (std::abs(filtered_point.z - z) <= z_threshold)
                {
                    is_close = true;
                    // 保留x值最 xiao的点
                    if (x < filtered_point.x)
                    {
                        filtered_cloud->points[j] = point;
                    }
                    break;
                }
            }

            // 如果不存在Z值相近的点，则将当前点添加到临时容器中
            if (!is_close)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }

    // 更新原始点云数据
    cloud->swap(*filtered_cloud);
}

// 提取banzi右上边界点--front
void LRC::extractPointsInUpRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position)
{
    output_cloud->clear();
    if (position.find("left") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.y >= fourpoints->points[1].y && point.y <= fourpoints->points[0].y && point.z >= fourpoints->points[1].z && point.z <= fourpoints->points[0].z)
            {
                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else if (position.find("right") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.x <= fourpoints->points[1].x && point.x >= fourpoints->points[0].x && point.z >= fourpoints->points[1].z && point.z <= fourpoints->points[0].z)
            {
                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
}

// 提取banzi右下边界点--front
void LRC::extractPointsInDownRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position)
{
    output_cloud->clear();
    if (position.find("left") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.y >= fourpoints->points[1].y && point.y <= fourpoints->points[2].y && point.z >= fourpoints->points[2].z && point.z <= fourpoints->points[1].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else if (position.find("right") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.x <= fourpoints->points[1].x && point.x >= fourpoints->points[2].x && point.z >= fourpoints->points[2].z && point.z <= fourpoints->points[1].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
}

// 提取banzi左下边界点--front
void LRC::extractPointsInDownLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position)
{
    output_cloud->clear();
    if (position.find("left") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.y >= fourpoints->points[2].y && point.y <= fourpoints->points[3].y && point.z >= fourpoints->points[2].z && point.z <= fourpoints->points[3].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else if (position.find("right") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x <= fourpoints->points[2].x && point.x >= fourpoints->points[3].x && point.z >= fourpoints->points[2].z && point.z <= fourpoints->points[3].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
}

// 提取banzi左上边界点--front
void LRC::extractPointsInUpLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, std::string position)
{
    output_cloud->clear();
    if (position.find("left") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.y >= fourpoints->points[0].y && point.y <= fourpoints->points[3].y && point.z >= fourpoints->points[3].z && point.z <= fourpoints->points[0].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else if (position.find("right") != std::string::npos)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.x <= fourpoints->points[0].x && point.x >= fourpoints->points[3].x && point.z >= fourpoints->points[3].z && point.z <= fourpoints->points[0].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
}

// 将边界点投影到直线上
void LRC::projectPointCloudOnLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Line3D &line, pcl::PointCloud<pcl::PointXYZ>::Ptr &projected_cloud)
{
    projected_cloud->clear();
    projected_cloud->reserve(cloud->size());

    for (const auto &point : cloud->points)
    {
        Eigen::Vector3f point_vec(point.x, point.y, point.z);
        Eigen::Vector3f projection = line.point + (point_vec - line.point).dot(line.direction) * line.direction;

        pcl::PointXYZ projected_point;
        projected_point.x = projection.x();
        projected_point.y = projection.y();
        projected_point.z = projection.z();

        projected_cloud->push_back(projected_point);
    }
}

// 根据输入的边界点集合，拟合直线方程
Line3D LRC::getLidarLineEquation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Line3D line;
    // 检查点云是否为空
    if (cloud->size() == 0)
    {
        std::cout << "点云为空，无法拟合直线！" << std::endl;
        return line; // 返回默认的直线
    }

    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(0.1);
    ransac.setMaxIterations(10);

    bool success = ransac.computeModel();
    if (success)
    {
        Eigen::VectorXf coefficients;
        ransac.getModelCoefficients(coefficients);

        line.point = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
        line.direction = Eigen::Vector3f(coefficients[3], coefficients[4], coefficients[5]);
    }
    else
    {
        std::cout << cloud->points.size() << std::endl;
        std::cout << "无法拟合直线！" << std::endl;
    }

    return line;
}



// 计算两个直线的交点，得到激光数据中板子的角点
Eigen::Vector3f LRC::computeLineIntersection(const Line3D &line1, const Line3D &line2)
{
    // 解线性方程组，求解交点坐标
    Eigen::Vector3f p1 = line1.point;
    Eigen::Vector3f p2 = line2.point;
    Eigen::Vector3f v1 = line1.direction;
    Eigen::Vector3f v2 = line2.direction;

    double t = ((p1.x() - p2.x()) * v2.y() - (p1.y() - p2.y()) * v2.x()) / (v1.x() * v2.y() - v1.y() * v2.x());
    t = -t;
    Eigen::Vector3f intersectionPoint = p1 + t * v1;

    return intersectionPoint;
}

// 根据四条直线交点得到激光下板子四个角点的中点
Eigen::Vector3f LRC::computeLidarPlaneCentroid(const cv::Mat &lidar_corner)
{
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 4; i++)
    {
        centroid[0] += lidar_corner.at<double>(i, 0);
        centroid[1] += lidar_corner.at<double>(i, 1);
        centroid[2] += lidar_corner.at<double>(i, 2);
    }

    centroid = centroid / 4;
    std::cout << centroid(0) << ";" << centroid(1) << ";" << centroid(2) << std::endl;
    return centroid;
}

// 根据四条直线交点得到相邻俩俩角点的中点，即板子四个边的中点
Eigen::Vector3f LRC::computeLidarLineCentroid(const cv::Mat &lidar_corner, int index1, int index2)
{
    Eigen::Vector3f centroid((lidar_corner.at<double>(index1, 0) + lidar_corner.at<double>(index2, 0)) * 0.5,
                             (lidar_corner.at<double>(index1, 1) + lidar_corner.at<double>(index2, 1)) * 0.5,
                             (lidar_corner.at<double>(index1, 2) + lidar_corner.at<double>(index2, 2)) * 0.5);

    return centroid;
}

// 计算在激光下的板子的平面方程
std::vector<double> LRC::calculatelidar_plane_equation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud)
{
    if (point_cloud->size() < 3)
    {
        std::cerr << "Error: Point cloud should contain at least 3 points." << std::endl;
        return std::vector<double>();
    }

    std::vector<double> plane_equation(4);

    // Randomly select three points
    std::vector<int> indices;
    for (int i = 0; i < point_cloud->size(); ++i)
    {
        indices.push_back(i);
    }
    std::random_shuffle(indices.begin(), indices.end());
    int index1 = indices[0];
    int index2 = indices[1];
    int index3 = indices[2];

    pcl::PointXYZ point1 = point_cloud->at(index1);
    pcl::PointXYZ point2 = point_cloud->at(index2);
    pcl::PointXYZ point3 = point_cloud->at(index3);

    Eigen::Vector3f vec_1 = point2.getVector3fMap() - point1.getVector3fMap();
    Eigen::Vector3f vec_2 = point3.getVector3fMap() - point1.getVector3fMap();

    Eigen::Vector3f normal = vec_1.cross(vec_2);
    if (point_cloud->points[0].x < 0 && point_cloud->points[0].y < 0)
    {
        if (normal.y() > 0)
            normal *= -1;
    }
    else
    {
        if (normal.x() < 0)
            normal *= -1;
    }

    normal.normalize();
    double d = -(normal.x() * point1.x + normal.y() * point1.y + normal.z() * point1.z);
    plane_equation[0] = normal.x();
    plane_equation[1] = normal.y();
    plane_equation[2] = normal.z();
    plane_equation[3] = d;

    return plane_equation;
}

void LRC::visualizePointClouds(pcl::visualization::PCLVisualizer &viewer,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected,
                               const Line3D &upRightLineEquation,
                               const Line3D &downRightLineEquation,
                               const Line3D &downLeftLineEquation,
                               const Line3D &upLeftLineEquation,
                               int viewer_id)
{
    viewer.setBackgroundColor(255, 255, 255, viewer_id);
    // viewer.addPolygon(polygon, 0, 0, 0, "polygon", viewer_id);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(cloud_projected, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, singleColor, "head", viewer_id);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head", viewer_id);

    viewer.addLine<pcl::PointXYZ>(visualizeLine(upRightLineEquation)->points[0], visualizeLine(upRightLineEquation)->points[1], "line1");
    viewer.addLine<pcl::PointXYZ>(visualizeLine(downRightLineEquation)->points[0], visualizeLine(downRightLineEquation)->points[1], "line2");
    viewer.addLine<pcl::PointXYZ>(visualizeLine(downLeftLineEquation)->points[0], visualizeLine(downLeftLineEquation)->points[1], "line3");
    viewer.addLine<pcl::PointXYZ>(visualizeLine(upLeftLineEquation)->points[0], visualizeLine(upLeftLineEquation)->points[1], "line4");
}

// 主成分分析(PCA)来找到标定板的主方向和中心
Eigen::Vector3f LRC::computeCenterWithPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    // 获取中心点
    Eigen::Vector3f center = pca.getMean().head<3>();

    // 获取主方向(可选)
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector3f normal = eigenVectors.col(2); // 最小特征值对应的特征向量

    return center;
}

// 处理激光标定板点云
ChessboardProcessResult LRC::processChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, std::string position)
{

    ChessboardProcessResult result;
    // result.cloud_projected.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
    // Eigen::Vector3f uprightlidarcorner, downrightlidarcorner, downleftlidarcorner, upleftlidarcorner;
    Eigen::Vector3f planecentroid;
    // Line3D upRightLineEquation, downRightLineEquation, downLeftLineEquation, upLeftLineEquation;
    // std::vector<double> planelidar_equation;

    // 把点云投影到一个平面上
    //********************************
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);

    seg.setInputCloud(plane_cloud);
    seg.segment(*inliers, *coefficients);
    std::cout << "PointCloud after segmentation has: "
              << inliers->indices.size() << " inliers." << std::endl;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(plane_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
    std::cerr << "PointCloud after projection has: " << cloud_projected->size() << " data points." << std::endl;
    planecentroid = computeCenterWithPCA(cloud_projected);

    result.planecentroid = planecentroid;

    return result;
}

void LRC::visualizeMultiplePointClouds(const std::vector<ChessboardProcessResult> &results)
{
    std::vector<std::shared_ptr<pcl::visualization::PCLVisualizer>> viewers;
    std::vector<std::thread> viewer_threads;

    for (size_t i = 0; i < results.size(); ++i)
    {
        auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("biaodingban " + std::to_string(i + 1));
        viewers.push_back(viewer);

        viewer_threads.emplace_back([this, viewer, &result = results[i], i]()
                                    {
            visualizePointClouds(*viewer,
                                 result.cloud_projected,
                                 result.upRightLineEquation,
                                 result.downRightLineEquation,
                                 result.downLeftLineEquation,
                                 result.upLeftLineEquation,
                                 i);

            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } });
    }

    // 等待所有查看器线程结束
    for (auto &thread : viewer_threads)
    {
        thread.join();
    }
}

void LRC::Preexecute(const std::string &lidar_path_left, const std::string &lidar_path_right)
{

    std::string left = "left";
    std::string right = "right";
    // pcl::visualization::PCLVisualizer viewer("pc_viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_left;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_right;

    extractPlaneCloud(cloud_left, plane_pcds_left, left);
    extractPlaneCloud(cloud_right, plane_pcds_right, right);

    std::vector<ChessboardProcessResult> left_results;
    std::vector<ChessboardProcessResult> right_results;

    for (const auto &plane_pcd : plane_pcds_left)
    {
        ChessboardProcessResult result = processChessboard(plane_pcd, left);
        left_results.push_back(result);
    }
    for (const auto &plane_pcd1 : plane_pcds_right)
    {
        ChessboardProcessResult result = processChessboard(plane_pcd1, right);
        right_results.push_back(result);
    }

}

LRC::~LRC()
{
}

int main(int argc, char *argv[])
{
    std::cout << "-------------" << std::endl;
    ros::init(argc, argv, "mulity lidar calibration");
    ros::NodeHandle nh;



    return 0;
}