#include "llc.h"
#include <iostream>
#include <thread>

std::condition_variable condition_variable;

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

LLC::LLC(ros::NodeHandle nh)
{
    // nh.getParam("lidar_path_left",lidar_path_left);
    // nh.getParam("lidar_path_right",lidar_path_right);
    // signal(SIGINT, signal_handler);
    chatter_pub = nh.advertise<sensor_msgs::PointCloud2>("chatter", 1000);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LLC::visualizeLine(const Line3D &line)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ start_point(line.point.x(), line.point.y(), line.point.z());

    line_cloud->push_back(start_point);
    line_cloud->push_back(pcl::PointXYZ(line.point.x() + line.direction.x(),
                                        line.point.y() + line.direction.y(),
                                        line.point.z() + line.direction.z()));

    return line_cloud;
}

bool LLC::extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &plane_pcds,
                            const std::string &position)
{
    pass_filter(input_cloud, position); // 带通滤波
    std::vector<pcl::PointIndices> indices_clusters;
    std::cout<<input_cloud->size()<<std::endl;
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
        }else{
            std::cout<<"GG"<<std::endl;
        }
    }

    return found_chessboard;
}

void LLC::pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &position)
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
        filter.setFilterLimits(-20, 0);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-10, 10);
        filter.filter(*pcd_in_roi);
    }
    else if (position.find("left") != std::string::npos)
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
void LLC::getfourpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud)
{
    if (corners_cloud->points[0].y < 0)
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
        if (corners_cloud->points[1].x > corners_cloud->points[2].x && corners_cloud->points[1].z > corners_cloud->points[2].z)
        {
            std::swap(corners_cloud->points[1], corners_cloud->points[2]);
            std::swap(corners_cloud->points[2], corners_cloud->points[3]);
        }
        else
        {
            std::swap(corners_cloud->points[2], corners_cloud->points[3]);
        }
    }
    else
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
        if (corners_cloud->points[1].x < corners_cloud->points[2].x && corners_cloud->points[1].z > corners_cloud->points[2].z)
        {
            std::swap(corners_cloud->points[1], corners_cloud->points[2]);
            std::swap(corners_cloud->points[2], corners_cloud->points[3]);
        }
        else
        {
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
    if (cloud->points[0].y < 0)
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

// 过滤左上左下影响拟合直线的点
void LLC::filterUpandDownLeftPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 定义阈值，用于判断Z值相近的点
    float z_threshold = 0.05;

    // 创建临时容器保存满足条件的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->points.reserve(cloud->size());
    if (cloud->points[0].y < 0)
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
void LLC::extractPointsInUpRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
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
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.x <= fourpoints->points[1].x && point.x >= fourpoints->points[0].y && point.z >= fourpoints->points[1].z && point.z <= fourpoints->points[0].z)
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
void LLC::extractPointsInDownRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
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
void LLC::extractPointsInDownLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].y < 0)
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
void LLC::extractPointsInUpLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr fourpoints,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (fourpoints->points[0].y < 0)
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
void LLC::projectPointCloudOnLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Line3D &line, pcl::PointCloud<pcl::PointXYZ>::Ptr &projected_cloud)
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
Line3D LLC::getLidarLineEquation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Line3D line;
    // 检查点云是否为空
    if (cloud->empty())
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

void LLC::visualizePointClouds(pcl::visualization::PCLVisualizer &viewer,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &projectuprightpoints,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &projectdownrightpoints,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &projectdownleftpoints,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &projectupleftpoints,
                            //    const pcl::PlanarPolygon<pcl::PointXYZ> &polygon,
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

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upright_color(projectuprightpoints, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(projectuprightpoints, upright_color, "head1", viewer_id);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head1", viewer_id);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downright_color(projectdownrightpoints, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(projectdownrightpoints, downright_color, "head2", viewer_id);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head2", viewer_id);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downleft_color(projectdownleftpoints, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(projectdownleftpoints, downleft_color, "head3", viewer_id);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head3", viewer_id);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upleft_color(projectupleftpoints, 0, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(projectupleftpoints, upleft_color, "head4", viewer_id);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head4", viewer_id);

    viewer.addLine<pcl::PointXYZ>(visualizeLine(upRightLineEquation)->points[0], visualizeLine(upRightLineEquation)->points[1], "line1");
    viewer.addLine<pcl::PointXYZ>(visualizeLine(downRightLineEquation)->points[0], visualizeLine(downRightLineEquation)->points[1], "line2");
    viewer.addLine<pcl::PointXYZ>(visualizeLine(downLeftLineEquation)->points[0], visualizeLine(downLeftLineEquation)->points[1], "line3");
    viewer.addLine<pcl::PointXYZ>(visualizeLine(upLeftLineEquation)->points[0], visualizeLine(upLeftLineEquation)->points[1], "line4");
}

// 处理激光标定板点云
ChessboardProcessResult LLC::processChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud)
{
    std::cout << viewer_id << std::endl;
    ChessboardProcessResult result;
    // result.cloud_projected.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3f uprightlidarcorner, downrightlidarcorner, downleftlidarcorner, upleftlidarcorner;
    Eigen::Vector3f uprightcentroid, downrightcentroid, downleftcentroid, upleftcentroid, planecentroid;
    Line3D upRightLineEquation, downRightLineEquation, downLeftLineEquation, upLeftLineEquation;
    std::vector<double> planelidar_equation;

    // 把点云投影到一个平面上
    //********************************
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

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

    // 计算整个平面点云的边框点
    //********************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_projected);
    chull.setAlpha(0.2);
    chull.reconstruct(*cloud_hull);

    int contour_point_size = cloud_hull->points.size();

    std::vector<ContourPoint> contour_point_vec;
    contour_point_vec.resize(contour_point_size);

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

    // 对激光板子四条边直线方向归一化
    //********************************
    if (corners_cloud->points[0].y < 0)
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
        if (upRightLineEquation.direction[1] < 0)
        {
            upRightLineEquation.direction *= -1;
        }
        if (downRightLineEquation.direction[1] > 0)
        {
            downRightLineEquation.direction *= -1;
        }
        if (downLeftLineEquation.direction[1] > 0)
        {
            downLeftLineEquation.direction *= -1;
        }
        if (upLeftLineEquation.direction[1] < 0)
        {
            upLeftLineEquation.direction *= -1;
        }
    }

    //********************************

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

    result.planelidar_equation = planelidar_equation;

    result.upLeftLineEquation = upLeftLineEquation;
    result.downLeftLineEquation = downLeftLineEquation;
    result.downRightLineEquation = downRightLineEquation;
    result.upRightLineEquation = upRightLineEquation;

    result.uprightcentroid = uprightcentroid;
    result.downleftcentroid = downleftcentroid;
    result.downrightcentroid = downrightcentroid;
    result.upleftcentroid = upleftcentroid;

    result.planecentroid = planecentroid;


    result.cloud_projected = cloud_projected;
    result.projectdownrightpoints = projectdownrightpoints;
    result.projectuprightpoints = projectuprightpoints;
    result.projectdownleftpoints = projectdownleftpoints;
    result.projectupleftpoints = projectupleftpoints;

    return result;
}

Eigen::Matrix3f LLC::init_estimate_R(const std::vector<ChessboardProcessResult> &left_results,
                                     const std::vector<ChessboardProcessResult> &right_results)
{
    assert(left_results.size() == right_results.size());
    int N = left_results.size(); // Number of chessboards

    // Initialize matrices ML and MC
    Eigen::MatrixXd ML(3, 5 * N);
    Eigen::MatrixXd MC(3, 5 * N);

    for (int i = 0; i < N; ++i)
    {
        const auto &left = left_results[i];
        const auto &right = right_results[i];

        // Fill ML
        ML.block<3, 1>(0, 5 * i) = Eigen::Vector3d(left.planelidar_equation.data());
        ML.block<3, 1>(0, 5 * i + 1) = left.downLeftLineEquation.direction.cast<double>();
        ML.block<3, 1>(0, 5 * i + 2) = left.upLeftLineEquation.direction.cast<double>();
        ML.block<3, 1>(0, 5 * i + 3) = left.upRightLineEquation.direction.cast<double>();
        ML.block<3, 1>(0, 5 * i + 4) = left.downRightLineEquation.direction.cast<double>();

        // Fill MC
        MC.block<3, 1>(0, 5 * i) = Eigen::Vector3d(right.planelidar_equation.data());
        MC.block<3, 1>(0, 5 * i + 1) = right.downLeftLineEquation.direction.cast<double>();
        MC.block<3, 1>(0, 5 * i + 2) = right.upLeftLineEquation.direction.cast<double>();
        MC.block<3, 1>(0, 5 * i + 3) = right.upRightLineEquation.direction.cast<double>();
        MC.block<3, 1>(0, 5 * i + 4) = right.downRightLineEquation.direction.cast<double>();
    }

    // Compute SVD
    Eigen::Matrix3d M = ML * MC.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Compute rotation matrix
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    return R.cast<float>();
}

Eigen::Vector3f LLC::init_estimate_t(const std::vector<ChessboardProcessResult> &left_results,
                                     const std::vector<ChessboardProcessResult> &right_results,
                                     const Eigen::Matrix3f &R_C_L)
{
    assert(left_results.size() == right_results.size());
    int N = left_results.size(); // Number of chessboards

    Eigen::MatrixXf A(13 * N, 3);
    Eigen::VectorXf b(13 * N);

    for (int i = 0; i < N; ++i)
    {
        const auto &left = left_results[i];
        const auto &right = right_results[i];

        Eigen::Vector3f n_c(right.planelidar_equation[0], right.planelidar_equation[1], right.planelidar_equation[2]);
        double d_c = right.planelidar_equation[3];

        std::vector<Eigen::Vector3f> p_c = {
            Eigen::Vector3f(right.downLeftLineEquation.point.data()),
            Eigen::Vector3f(right.upLeftLineEquation.point.data()),
            Eigen::Vector3f(right.upRightLineEquation.point.data()),
            Eigen::Vector3f(right.downRightLineEquation.point.data())};

        std::vector<Eigen::Vector3f> l_c = {
            Eigen::Vector3f(right.downLeftLineEquation.direction.data()),
            Eigen::Vector3f(right.upLeftLineEquation.direction.data()),
            Eigen::Vector3f(right.upRightLineEquation.direction.data()),
            Eigen::Vector3f(right.downRightLineEquation.direction.data())};

        std::vector<Eigen::Vector3f> centroids_L = {
            left.downleftcentroid, left.upleftcentroid,
            left.uprightcentroid, left.downrightcentroid};

        // Fill matrix A and vector b
        A.block<1, 3>(13 * i, 0) = n_c.transpose();
        b(13 * i) = -(n_c.transpose() * (R_C_L * left.planecentroid) + d_c);

        for (int j = 0; j < 4; ++j)
        {
            Eigen::Matrix3f A_ij = calculate_A(l_c[j]);
            A.block<3, 3>(13 * i + 1 + 3 * j, 0) = A_ij;
            b.segment<3>(13 * i + 1 + 3 * j) = -(A_ij * (R_C_L * centroids_L[j] - p_c[j]));
        }
    }

    // Solve the linear least squares problem
    Eigen::Vector3f t_C_L = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    return t_C_L;
}

// 输入向量A，计算I - AA转置
Eigen::Matrix3f LLC::calculate_A(const Eigen::Vector3f &l)
{
    return Eigen::Matrix3f::Identity() - l * l.transpose();
}

void LLC::visualizeMultiplePointClouds(const std::vector<ChessboardProcessResult>& results) {
    std::vector<std::shared_ptr<pcl::visualization::PCLVisualizer>> viewers;
    std::vector<std::thread> viewer_threads;

    for (size_t i = 0; i < results.size(); ++i) {
        auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("biaodingban " + std::to_string(i + 1));
        viewers.push_back(viewer);

        viewer_threads.emplace_back([this, viewer, &result = results[i], i]() {
            visualizePointClouds(*viewer,
                                 result.cloud_projected,
                                 result.projectuprightpoints,
                                 result.projectdownrightpoints,
                                 result.projectdownleftpoints,
                                 result.projectupleftpoints,
                                //  result.planelidar_equation,
                                 result.upRightLineEquation,
                                 result.downRightLineEquation,
                                 result.downLeftLineEquation,
                                 result.upLeftLineEquation,
                                 i);

            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    // 等待所有查看器线程结束
    for (auto& thread : viewer_threads) {
        thread.join();
    }
}


void LLC::Preexecute(const std::string &lidar_path_left, const std::string &lidar_path_right)
{

    std::string left, right;
    // pcl::visualization::PCLVisualizer viewer("pc_viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile(lidar_path_left, *cloud_left))
    {
        std::cout<<"chucuole"<<std::endl;
        exit(-1);
    }
    if (pcl::io::loadPCDFile(lidar_path_right, *cloud_right))
    {
        exit(-1);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_left;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_right;

    extractPlaneCloud(cloud_left, plane_pcds_left, left);
    extractPlaneCloud(cloud_right, plane_pcds_right, right);

    std::vector<ChessboardProcessResult> left_results;
    std::vector<ChessboardProcessResult> right_results;

    for (const auto &plane_pcd : plane_pcds_left)
    {
        ChessboardProcessResult result = processChessboard(plane_pcd);
        left_results.push_back(result);
    }
    for (const auto &plane_pcd1 : plane_pcds_right)
    {
        ChessboardProcessResult result = processChessboard(plane_pcd1);
        right_results.push_back(result);
    }
    // 可视化左侧结果
    LLC::visualizeMultiplePointClouds(left_results);

    // 可视化右侧结果
    LLC::visualizeMultiplePointClouds(right_results);


    Eigen::Matrix3f rotation_matrix = init_estimate_R(left_results, right_results);
    Eigen::Vector3f estimated_t = init_estimate_t(left_results, right_results, rotation_matrix);

    Eigen::Matrix4f transformMatrix;
    transformMatrix.setIdentity();

    // Set rotation matrix (transpose for left to right)
    transformMatrix.block(0, 0, 3, 3) = rotation_matrix.transpose();

    // Set translation vector (adjusted for the new coordinate system)
    transformMatrix.block(0, 3, 3, 1) = -rotation_matrix.transpose() * estimated_t;

    printmatrixwithcommas(transformMatrix);
}

LLC::~LLC()
{
}

int main(int argc, char *argv[])
{
    std::cout << "-------------" << std::endl;
    ros::init(argc, argv, "mulity lidar calibration");
    ros::NodeHandle nh;

    std::string lidar_path_left = "/home/conan/llc_ros1/llc/in_out/left.pcd";
    std::string lidar_path_right = "/home/conan/llc_ros1/llc/in_out/right.pcd";

    LLC read_pcd(nh);
    read_pcd.Preexecute(lidar_path_left, lidar_path_right);

    return 0;
}