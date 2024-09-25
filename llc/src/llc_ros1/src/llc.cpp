#include "llc.h"
#include <iostream>


bool LLC::extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &plane_pcds,
                            const std::string &kuangshan)
{
    pass_filter(input_cloud, kuangshan); // 带通滤波
    std::vector<pcl::PointIndices> indices_clusters;
    pcd_clustering(input_cloud, indices_clusters); // 聚类

    bool found_chessboard = false;
    for (const auto &cluster : indices_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr potential_plane(new pcl::PointCloud<pcl::PointXYZ>);
        if (extractChessboard(input_cloud, cluster, potential_plane))
        {
            plane_pcds.push_back(potential_plane);
            found_chessboard = true;
            std::cout << "提取到标定板点云!" << std::endl;
            // display_colored_by_depth(potential_plane); // 如果需要显示每个标定板
        }
    }

    return found_chessboard;
}

void LLC::pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &kuangshan)
{
    auto &pcd_in_roi = input_pcd;
    pcl::PassThrough<pcl::PointXYZ> filter;
    if (kuangshan.find("wuhu") != std::string::npos)
    {
        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-2, 2);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(-12, 0);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-5, 5);
        filter.filter(*pcd_in_roi);
    }
    else
    {
        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-2, 2);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(-5, 5);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(0, 12);
        filter.filter(*pcd_in_roi);
    }
}

void LLC::pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, std::vector<pcl::PointIndices> &pcd_clusters)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input_pcd);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(150);
    reg.setMaxClusterSize(6000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(60);
    reg.setInputCloud(input_pcd);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.5);
    reg.extract(pcd_clusters);
}

bool LLC::check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd)
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

bool LLC::extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                            const pcl::PointIndices &cluster,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_pcd);
    extract.setIndices(boost::make_shared<pcl::PointIndices>(cluster));
    extract.setNegative(false);
    extract.filter(*chessboard_pcd);

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

// 中距:从得到的激光边界凸出最大的十个点里，筛选出板子的四个角点，四个点顺序顺时针，板子最上面的点为第一个点
void LLC::getfourpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud)
{
    if (corners_cloud->points[0].y < 0 && corners_cloud->points[0].x < 0)
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

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;

        std::cout << "成功提取并删除其他点！" << std::endl;

        std::sort(corners_cloud->points.begin(), corners_cloud->points.end(), compareByZ);
        if (corners_cloud->points[1].x > corners_cloud->points[2].x)
        {
            std::swap(corners_cloud->points[1], corners_cloud->points[2]);
            std::swap(corners_cloud->points[2], corners_cloud->points[3]);
        }
    }
    else
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
}

// 过滤右上右下影响拟合直线的点
void LLC::filterUpandDownRightPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 定义阈值，用于判断Z值相近的点
    float z_threshold = 0.05;

    // 创建临时容器保存满足条件的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->points.reserve(cloud->size());
    if (cloud->points[0].y < 0 && cloud->points[0].x < 0)
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            float z = point.z;
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
    else
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
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
    // 更新原始点云数据
    cloud->swap(*filtered_cloud);
}

// 过滤左上左下影响拟合直线的点
void LLC::filterUpandDownLeftPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 定义阈值，用于判断Z值相近的点
    float z_threshold = 0.05;

    // 创建临时容器保存满足条件的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->points.reserve(cloud->size());
    if (cloud->points[0].y < 0 && cloud->points[0].x < 0)
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            float z = point.z;
            float x = point.x;

            // 检查是否存在Z值相近的点
            bool is_close = false;
            for (int j = 0; j < filtered_cloud->size(); ++j)
            {
                pcl::PointXYZ filtered_point = filtered_cloud->points[j];
                if (std::abs(filtered_point.z - z) <= z_threshold)
                {
                    is_close = true;
                    // 保留x值最大的点
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
    else
    {
        for (int i = 0; i < cloud->size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
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
                    // 保留Y值最大的点
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

    // 更新原始点云数据
    cloud->swap(*filtered_cloud);
}

// 提取中距右上边界点
void LLC::extractPointsInUpRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].x < 0 && fourpoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x >= fourpoints->points[1].x && point.x <= fourpoints->points[0].x && point.z >= fourpoints->points[1].z && point.z <= fourpoints->points[0].z)
            {
                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
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
}

// 提取中距右下边界点
void LLC::extractPointsInDownRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].x < 0 && fourpoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x >= fourpoints->points[1].x && point.x <= fourpoints->points[2].x && point.z >= fourpoints->points[2].z && point.z <= fourpoints->points[1].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
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
}

// 提取中距左下边界点
void LLC::extractPointsInDownLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].x < 0 && fourpoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x >= fourpoints->points[2].x && point.x <= fourpoints->points[3].x && point.z >= fourpoints->points[2].z && point.z <= fourpoints->points[3].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else
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
}

// 提取中距左上边界点
void LLC::extractPointsInUpLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].x < 0 && fourpoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x >= fourpoints->points[0].x && point.x <= fourpoints->points[3].x && point.z >= fourpoints->points[3].z && point.z <= fourpoints->points[0].z)
            {

                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;
        std::cout << output_cloud->points.size() << std::endl;
    }
    else
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
}

// 根据输入的边界点集合，拟合直线方程
Line3D LLC::getLidarLineEquation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Line3D line;
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

// 计算两个直线的交点，得到激光数据中棋盘格板子的角点
Eigen::Vector3f LLC::computeLineIntersection(const Line3D &line1, const Line3D &line2)
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
Eigen::Vector3f LLC::computeLidarPlaneCentroid(const cv::Mat &lidar_corner)
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
Eigen::Vector3f LLC::computeLidarLineCentroid(const cv::Mat &lidar_corner, int index1, int index2)
{
    Eigen::Vector3f centroid((lidar_corner.at<double>(index1, 0) + lidar_corner.at<double>(index2, 0)) * 0.5,
                             (lidar_corner.at<double>(index1, 1) + lidar_corner.at<double>(index2, 1)) * 0.5,
                             (lidar_corner.at<double>(index1, 2) + lidar_corner.at<double>(index2, 2)) * 0.5);

    return centroid;
}

// 计算在激光下的板子的平面方程
std::vector<double> LLC::calculatelidar_plane_equation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud)
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

// 输入向量A，计算I - AA转置
Eigen::Matrix3f LLC::calculate_A(const Eigen::Vector3f &line_direction)
{
    Eigen::Vector3f direction = line_direction;

    Eigen::Matrix3f matrix_a = Eigen::Matrix3f::Identity() - direction * direction.transpose();

    return matrix_a;
}






void LLC::Preexecute(const std::string &kuangshan, const std::string &path)
{
    pcl::visualization::PCLVisualizer viewer("pc_viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile(lidar_path, *cloud))
    {
        exit(-1);
    }

    // std::unordered_map<int, pcl::PointXYZ> leftPoints, radarPoints, rightPoints, finalPoints;
    // extractPlaneCloud(cloud, plane_cloud, kuangshan);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds;
    if (extractPlaneCloud(input_cloud, plane_pcds, kuangshan))
    {
        std::cout << "找到 " << plane_pcds.size() << " 个标定板点云!" << std::endl;
        // 处理每个标定板点云
        for (const auto &plane_pcd : plane_pcds)
        {
            // 对每个标定板进行后续处理
        }
    }
    else
    {
        std::cout << "未找到标定板点云!" << std::endl;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

    //********************************
    // 计算整个平面点云的边框点
    //********************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_projected);
    chull.setAlpha(0.1);
    chull.reconstruct(*cloud_hull);

    int contour_point_size = cloud_hull->points.size();

    std::vector<ContourPoint> contour_point_vec;
    contour_point_vec.resize(contour_point_size);

    //********************************
    // 计算边框点相邻俩俩点向量的夹角，并记录下对应夹角的点的索引，进行由大到小排序
    //********************************
    for (int i = 0; i < contour_point_size; i++)
    {
        if (i == 0)
        {
            Eigen::Vector3d front_point, middle_point, back_point;
            front_point << cloud_hull->points[i + 1].x, cloud_hull->points[i + 1].y, cloud_hull->points[i + 1].z;
            middle_point << cloud_hull->points[i].x, cloud_hull->points[i].y, cloud_hull->points[i].z;
            back_point << cloud_hull->points[contour_point_size - 1].x, cloud_hull->points[contour_point_size - 1].y, cloud_hull->points[contour_point_size - 1].z;

            Eigen::Vector3d vector1 = front_point - middle_point;
            Eigen::Vector3d vector2 = middle_point - back_point;
            double cos_theta = vector1.dot(vector2) / (vector1.norm() * vector2.norm());
            contour_point_vec[i].angle = std::acos(cos_theta);
            continue;
        }

        if (i == contour_point_size - 1)
        {
            Eigen::Vector3d front_point, middle_point, back_point;
            front_point << cloud_hull->points[0].x, cloud_hull->points[0].y, cloud_hull->points[0].z;
            middle_point << cloud_hull->points[i].x, cloud_hull->points[i].y, cloud_hull->points[i].z;
            back_point << cloud_hull->points[i - 1].x, cloud_hull->points[i - 1].y, cloud_hull->points[i - 1].z;

            Eigen::Vector3d vector1 = front_point - middle_point;
            Eigen::Vector3d vector2 = middle_point - back_point;
            double cos_theta = vector1.dot(vector2) / (vector1.norm() * vector2.norm());
            contour_point_vec[i].angle = std::acos(cos_theta);
            continue;
        }
        contour_point_vec[i].index = i;

        Eigen::Vector3d front_point, middle_point, back_point;
        front_point << cloud_hull->points[i + 1].x, cloud_hull->points[i + 1].y, cloud_hull->points[i + 1].z;
        middle_point << cloud_hull->points[i].x, cloud_hull->points[i].y, cloud_hull->points[i].z;
        back_point << cloud_hull->points[i - 1].x, cloud_hull->points[i - 1].y, cloud_hull->points[i - 1].z;

        Eigen::Vector3d vector1 = front_point - middle_point;
        Eigen::Vector3d vector2 = middle_point - back_point;
        double cos_theta = vector1.dot(vector2) / (vector1.norm() * vector2.norm());
        contour_point_vec[i].angle = std::acos(cos_theta);
        // std::cout << i << " " << std::acos(cos_theta) << '\n';
        // std::cout << a << std::endl;
    }
    std::sort(contour_point_vec.begin(), contour_point_vec.end(), ContourPointCompare());

    //********************************
    // 取出角度大的前十个，并从中筛选出板子的四个角点
    //********************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    corners_cloud->width = 20;
    corners_cloud->height = 1;
    corners_cloud->is_dense = false;
    corners_cloud->resize(10 * 2);
    for (int i = 0; i < 20; i++)
    {
        int index = contour_point_vec[i].index;
        // std::cout << index << std::endl;
        corners_cloud->points[i] = cloud_hull->points[index];
    }
    getfourpoints(corners_cloud);

    //********************************

    pcl::PointCloud<pcl::PointXYZ>::Ptr uprightpoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downrightpoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr upleftpoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downleftpoints(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ> contour;
    contour.width = cloud_hull->width;
    contour.height = 1;
    contour.is_dense = false;
    contour.resize(contour.height * contour.width);
    for (int i = 0; i < cloud_hull->points.size(); i++)
    {
        contour.points[i] = cloud_hull->points[i];
    }

    // 提取边框对应四条边的点
    extractPointsInUpRight(contour, corners_cloud, uprightpoints);
    extractPointsInDownRight(contour, corners_cloud, downrightpoints);
    extractPointsInDownLeft(contour, corners_cloud, downleftpoints);
    extractPointsInUpLeft(contour, corners_cloud, upleftpoints);
    // 过滤影响拟合直线的点
    filterUpandDownRightPoints(uprightpoints);
    filterUpandDownRightPoints(downrightpoints);
    filterUpandDownLeftPoints(downleftpoints);
    filterUpandDownLeftPoints(upleftpoints);
    // 拟合激光直线
    upRightLineEquation = getLidarLineEquation(uprightpoints);
    downRightLineEquation = getLidarLineEquation(downrightpoints);
    downLeftLineEquation = getLidarLineEquation(downleftpoints);
    upLeftLineEquation = getLidarLineEquation(upleftpoints);

    // 对激光和相机得到的板子四条边直线方向归一化

    if (corners_cloud->points[0].x < 0 && corners_cloud->points[0].y < 0)
    {
        if (upRightLineEquation.direction[0] > 0)
        {
            upRightLineEquation.direction *= -1;
        }
        if (downRightLineEquation.direction[0] < 0)
        {
            downRightLineEquation.direction *= -1;
        }
        if (downLeftLineEquation.direction[0] < 0)
        {
            downLeftLineEquation.direction *= -1;
        }
        if (upLeftLineEquation.direction[0] > 0)
        {
            upLeftLineEquation.direction *= -1;
        }
    }
    else
    {
        if (upRightLineEquation.direction[1] > 0)
        {
            upRightLineEquation.direction *= -1;
        }
        if (downRightLineEquation.direction[1] < 0)
        {
            downRightLineEquation.direction *= -1;
        }
        if (downLeftLineEquation.direction[1] < 0)
        {
            downLeftLineEquation.direction *= -1;
        }
        if (upLeftLineEquation.direction[1] > 0)
        {
            upLeftLineEquation.direction *= -1;
        }
    }

    // 投影边界点到对应的直线上
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectuprightpoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectdownrightpoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectdownleftpoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectupleftpoints(new pcl::PointCloud<pcl::PointXYZ>);
    projectPointCloudOnLine(uprightpoints, upRightLineEquation, projectuprightpoints);
    projectPointCloudOnLine(downrightpoints, downRightLineEquation, projectdownrightpoints);
    projectPointCloudOnLine(downleftpoints, downLeftLineEquation, projectdownleftpoints);
    projectPointCloudOnLine(upleftpoints, upLeftLineEquation, projectupleftpoints);

    // 根据算的直线方程计算交点得到激光下板子的角点
    uprightlidarcorner = computeLineIntersection(upRightLineEquation, upLeftLineEquation);
    downrightlidarcorner = computeLineIntersection(upRightLineEquation, downRightLineEquation);
    downleftlidarcorner = computeLineIntersection(downRightLineEquation, downLeftLineEquation);
    upleftlidarcorner = computeLineIntersection(downLeftLineEquation, upLeftLineEquation);
    cv::Mat lidar_corner = (cv::Mat_<double>(4, 3) << uprightlidarcorner(0), uprightlidarcorner(1), uprightlidarcorner(2),
                            downrightlidarcorner(0), downrightlidarcorner(1), downrightlidarcorner(2),
                            downleftlidarcorner(0), downleftlidarcorner(1), downleftlidarcorner(2),
                            upleftlidarcorner(0), upleftlidarcorner(1), upleftlidarcorner(2));

    // 计算激光下板子四个边的中点
    uprightcentroid = computeLidarLineCentroid(lidar_corner, 0, 1);
    downrightcentroid = computeLidarLineCentroid(lidar_corner, 1, 2);
    downleftcentroid = computeLidarLineCentroid(lidar_corner, 2, 3);
    upleftcentroid = computeLidarLineCentroid(lidar_corner, 3, 0);

    // 板子在激光下的中点
    planecentroid = computeLidarPlaneCentroid(lidar_corner);
    // computelidarplaneCentroid(cloud_projected);

    // 板子在激光下的平面方程
    planelidar_equation = calculatelidar_plane_equation(cloud_projected);
}

void LLC::execute()
{
}

LLC::~LLC()
{
}

int main(int argc, char *argv[])
{
    std::cout << "-------------" << std::endl;
    ros::init(argc, argv, "ban_zhu_ren");
    ros::NodeHandle nh;

    LLC read_pcd(nh);
    read_pcd.execute();

    return 0;
}