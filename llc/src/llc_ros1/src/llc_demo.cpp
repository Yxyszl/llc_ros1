#include "llc.h"
#include <iostream>

bool is_exit = false;
std::condition_variable condition_variable;

void signal_handler(int number)
{
    is_exit = true;
    std::cout << "signal number is " << number << std::endl;
    condition_variable.notify_all();
}

LLC::LLC(ros::NodeHandle nh)
{
    signal(SIGINT, signal_handler);
    chatter_pub = nh.advertise<sensor_msgs::PointCloud2>("chatter", 1000);
}
// 根据直线点向式计算两个点，用来画直线，直线可视化
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
// 鱼眼：从得到的激光边界凸出最大的十个点里，筛选出板子的下左右三个角点，三个点顺序顺时针，板子最右面的点为第一个点
void LLC::getthreepoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud)
{
    if (corners_cloud->points[0].y < 0 && corners_cloud->points[0].x < 0)
    {
        pcl::PointXYZ x_min_point, x_max_point, z_min_point;
        // 寻找最小和最大 x 值以及最小 z 值的四个点
        x_min_point = corners_cloud->points[0];
        x_max_point = corners_cloud->points[0];
        z_min_point = corners_cloud->points[0];

        for (const auto &point : corners_cloud->points)
        {
            if (point.x < x_min_point.x)
                x_min_point = point;
            if (point.x > x_max_point.x)
                x_max_point = point;

            if (point.z < z_min_point.z)
                z_min_point = point;
        }
        // 删除除了最小和最大 x 值以及最小 z 值的四个点之外的其他点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : corners_cloud->points)
        {
            if (point.x == x_min_point.x && point.y == x_min_point.y && point.z == x_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == x_max_point.x && point.y == x_max_point.y && point.z == x_max_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_min_point.x && point.y == z_min_point.y && point.z == z_min_point.z)
                filtered_cloud->points.push_back(point);
        }

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;

        std::cout << "成功提取并删除其他点！" << std::endl;

        std::sort(corners_cloud->points.begin(), corners_cloud->points.end(), compareByZ);
        if (corners_cloud->points[0].x > corners_cloud->points[1].x)
        {
            std::swap(corners_cloud->points[0], corners_cloud->points[1]);
            std::swap(corners_cloud->points[1], corners_cloud->points[2]);
        }
    }
    else
    {
        pcl::PointXYZ y_min_point, y_max_point, z_min_point;
        // 寻找最小和最大 y 值以及最小 z 值的3个点
        y_min_point = corners_cloud->points[0];
        y_max_point = corners_cloud->points[0];
        z_min_point = corners_cloud->points[0];

        for (const auto &point : corners_cloud->points)
        {
            if (point.y < y_min_point.y)
                y_min_point = point;
            if (point.y > y_max_point.y)
                y_max_point = point;

            if (point.z < z_min_point.z)
                z_min_point = point;
        }
        // 删除除了最小和最大 y 值以及最小 z 值的3个点之外的其他点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : corners_cloud->points)
        {
            if (point.x == y_min_point.x && point.y == y_min_point.y && point.z == y_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == y_max_point.x && point.y == y_max_point.y && point.z == y_max_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_min_point.x && point.y == z_min_point.y && point.z == z_min_point.z)
                filtered_cloud->points.push_back(point);
        }

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;

        std::cout << "成功提取并删除其他点！" << std::endl;

        std::sort(corners_cloud->points.begin(), corners_cloud->points.end(), compareByY);
        // if (corners_cloud->points[0].y > corners_cloud->points[1].y)
        // {
        //     std::swap(corners_cloud->points[0],corners_cloud->points[1]);
        //     std::swap(corners_cloud->points[1],corners_cloud->points[2]);
        // }
        for (const auto &point : corners_cloud->points)
        {
            std::cout << point << std::endl;
        }
    }
}

void LLC::wuhugetthreepoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &corners_cloud)
{
    if (corners_cloud->points[0].y < 0 && corners_cloud->points[0].x < 0)
    {
        pcl::PointXYZ x_min_point, x_max_point, z_max_point;
        // 寻找最小和最大 x 值以及最小 z 值的3个点
        x_min_point = corners_cloud->points[0];
        x_max_point = corners_cloud->points[0];
        z_max_point = corners_cloud->points[0];

        for (const auto &point : corners_cloud->points)
        {
            if (point.x < x_min_point.x)
                x_min_point = point;
            if (point.x > x_max_point.x)
                x_max_point = point;

            if (point.z > z_max_point.z)
                z_max_point = point;
        }
        // 删除除了最小和最大 x 值以及最da z 值的3个点之外的其他点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : corners_cloud->points)
        {
            if (point.x == x_min_point.x && point.y == x_min_point.y && point.z == x_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == x_max_point.x && point.y == x_max_point.y && point.z == x_max_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_max_point.x && point.y == z_max_point.y && point.z == z_max_point.z)
                filtered_cloud->points.push_back(point);
        }

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;

        std::cout << "成功提取并删除其他点！" << std::endl;

        std::sort(corners_cloud->points.begin(), corners_cloud->points.end(), compareByX);
    }
    else
    {
        pcl::PointXYZ y_min_point, y_max_point, z_max_point;
        // 寻找最小和最大 y 值以及最da z 值的3个点
        y_min_point = corners_cloud->points[0];
        y_max_point = corners_cloud->points[0];
        z_max_point = corners_cloud->points[0];

        for (const auto &point : corners_cloud->points)
        {
            if (point.y < y_min_point.y)
                y_min_point = point;
            if (point.y > y_max_point.y)
                y_max_point = point;

            if (point.z > z_max_point.z)
                z_max_point = point;
        }
        // 删除除了最小和最大 y 值以及最da z 值的3个点之外的其他点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : corners_cloud->points)
        {
            if (point.x == y_min_point.x && point.y == y_min_point.y && point.z == y_min_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == y_max_point.x && point.y == y_max_point.y && point.z == y_max_point.z)
                filtered_cloud->points.push_back(point);
            else if (point.x == z_max_point.x && point.y == z_max_point.y && point.z == z_max_point.z)
                filtered_cloud->points.push_back(point);
        }

        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered_cloud->points;
        corners_cloud->width = filtered_cloud->width;
        corners_cloud->height = filtered_cloud->height;
        std::cout << "成功提取并删除其他点！" << std::endl;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(corners_cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered);
        std::sort(filtered->points.begin(), filtered->points.end(), compareByY);
        for (const auto &point : filtered->points)
        {
            std::cout << point.y << std::endl;
        }
        // 更新点云数据为筛选后的结果
        corners_cloud->points = filtered->points;
        corners_cloud->width = filtered->width;
        corners_cloud->height = filtered->height;
    }
}

// 输入左下边直线，平移方向向量，和平移距离，得到右上直线点向式
Line3D LLC::translateLineupright(const Line3D &line, const Line3D &directionline, float distance)
{
    Line3D newLine;
    if (directionline.direction[1] > 0)
    {
        newLine.point = line.point + distance * (-directionline.direction);
        newLine.direction = line.direction;
    }
    else
    {
        newLine.point = line.point + distance * directionline.direction;
        newLine.direction = line.direction;
    }

    // std::cout << "平移后的点向式：" << std::endl;
    // std::cout << "点 P': (" << newLine.point[0] << ", " << newLine.point[1] << ", " << newLine.point[2] << ")" << std::endl;
    // std::cout << "方向向量 V: (" << newLine.direction[0] << ", " << newLine.direction[1] << ", " << newLine.direction[2] << ")" << std::endl;
    return newLine;
}
// 输入右下边直线，平移方向向量，和平移距离，得到左上直线点向式
Line3D LLC::translateLineupleft(const Line3D &line, const Line3D &directionline, float distance)
{
    Line3D newLine;
    if (directionline.direction[1] < 0)
    {
        newLine.point = line.point + distance * (-directionline.direction);
        newLine.direction = line.direction;
    }
    else
    {
        newLine.point = line.point + distance * directionline.direction;
        newLine.direction = line.direction;
    }

    // std::cout << "平移后的点向式：" << std::endl;
    // std::cout << "点 P': (" << newLine.point[0] << ", " << newLine.point[1] << ", " << newLine.point[2] << ")" << std::endl;
    // std::cout << "方向向量 V: (" << newLine.direction[0] << ", " << newLine.direction[1] << ", " << newLine.direction[2] << ")" << std::endl;
    return newLine;
}

// 输入左下边直线，平移方向向量，和平移距离，得到右上直线点向式
Line3D LLC::translateLinedownright(const Line3D &line, const Line3D &directionline, float distance)
{
    Line3D newLine;
    if (directionline.direction[0] > 0)
    {
        newLine.point = line.point + distance * (-directionline.direction);
        newLine.direction = line.direction;
    }
    else
    {
        newLine.point = line.point + distance * directionline.direction;
        newLine.direction = line.direction;
    }

    // std::cout << "平移后的点向式：" << std::endl;
    // std::cout << "点 P': (" << newLine.point[0] << ", " << newLine.point[1] << ", " << newLine.point[2] << ")" << std::endl;
    // std::cout << "方向向量 V: (" << newLine.direction[0] << ", " << newLine.direction[1] << ", " << newLine.direction[2] << ")" << std::endl;
    return newLine;
}
// 输入右下边直线，平移方向向量，和平移距离，得到左上直线点向式
Line3D LLC::translateLinedownleft(const Line3D &line, const Line3D &directionline, float distance)
{
    Line3D newLine;
    if (directionline.direction[0] < 0)
    {
        newLine.point = line.point + distance * (-directionline.direction);
        newLine.direction = line.direction;
    }
    else
    {
        newLine.point = line.point + distance * directionline.direction;
        newLine.direction = line.direction;
    }

    // std::cout << "平移后的点向式：" << std::endl;
    // std::cout << "点 P': (" << newLine.point[0] << ", " << newLine.point[1] << ", " << newLine.point[2] << ")" << std::endl;
    // std::cout << "方向向量 V: (" << newLine.direction[0] << ", " << newLine.direction[1] << ", " << newLine.direction[2] << ")" << std::endl;
    return newLine;
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
// 提取鱼眼右下边界点
void LLC::extractfisheyePointsInDownRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr threepoints,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (threepoints->points[0].x < 0 && threepoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x >= threepoints->points[0].x && point.x <= threepoints->points[1].x && point.z >= threepoints->points[1].z && point.z <= threepoints->points[0].z)
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
            if (point.y >= threepoints->points[0].y && point.y <= threepoints->points[1].y && point.z >= threepoints->points[1].z && point.z <= threepoints->points[0].z)
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

// 提取鱼眼右shang边界点
void LLC::extractfisheyePointsInUpRight(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr threepoints,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (threepoints->points[0].x < 0 && threepoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.x >= threepoints->points[0].x && point.x <= threepoints->points[1].x && point.z >= threepoints->points[0].z && point.z <= threepoints->points[1].z)
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
            if (point.y >= threepoints->points[0].y && point.y <= threepoints->points[1].y && point.z >= threepoints->points[0].z && point.z <= threepoints->points[1].z)
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

// // Convert 3D points w.r.t camera frame to 2D pixel points in image frame
double *LLC::converto_imgpts(double x, double y, double z, cv::Mat &cameraMatrix)
{
    double tmpxC = x / z;
    double tmpyC = y / z;
    cv::Point2d planepointsC;
    planepointsC.x = tmpxC;
    planepointsC.y = tmpyC;
    double r2 = tmpxC * tmpxC + tmpyC * tmpyC;

    if (0)
    {
        double r1 = pow(r2, 0.5);
        double a0 = std::atan(r1);
        // distortion function for a fisheye lens
        double a1 = a0 * (1 + distortionCoefficients.at<double>(0) * pow(a0, 2) +
                          distortionCoefficients.at<double>(1) * pow(a0, 4) + distortionCoefficients.at<double>(2) * pow(a0, 6) +
                          distortionCoefficients.at<double>(3) * pow(a0, 8));
        planepointsC.x = (a1 / r1) * tmpxC;
        planepointsC.y = (a1 / r1) * tmpyC;
        planepointsC.x = cameraMatrix.at<double>(0, 0) * planepointsC.x + cameraMatrix.at<double>(0, 2);
        planepointsC.y = cameraMatrix.at<double>(1, 1) * planepointsC.y + cameraMatrix.at<double>(1, 2);
    }
    else // For pinhole camera model
    {
        double tmpdist = 1 + distortionCoefficients.at<double>(0) * r2 + distortionCoefficients.at<double>(1) * r2 * r2 +
                         distortionCoefficients.at<double>(4) * r2 * r2 * r2;
        planepointsC.x = tmpxC * tmpdist + 2 * distortionCoefficients.at<double>(2) * tmpxC * tmpyC +
                         distortionCoefficients.at<double>(3) * (r2 + 2 * tmpxC * tmpxC);
        planepointsC.y = tmpyC * tmpdist + distortionCoefficients.at<double>(2) * (r2 + 2 * tmpyC * tmpyC) +
                         2 * distortionCoefficients.at<double>(3) * tmpxC * tmpyC;
        planepointsC.x = cameraMatrix.at<double>(0, 0) * planepointsC.x + cameraMatrix.at<double>(0, 2);
        planepointsC.y = cameraMatrix.at<double>(1, 1) * planepointsC.y + cameraMatrix.at<double>(1, 2);
    }

    double *img_coord = new double[2];
    *(img_coord) = planepointsC.x;
    *(img_coord + 1) = planepointsC.y;

    return img_coord;
}
// 提取鱼眼左下边界点
void LLC::extractfisheyePointsInDownLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr threepoints,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (threepoints->points[0].x < 0 && threepoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 Y 和 Z 值是否在所需的范围内
            if (point.x >= threepoints->points[1].x && point.x <= threepoints->points[2].x && point.z >= threepoints->points[1].z && point.z <= threepoints->points[2].z)
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
            if (point.y >= threepoints->points[1].y && point.y <= threepoints->points[2].y && point.z >= threepoints->points[1].z && point.z <= threepoints->points[2].z)
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

// 提取鱼眼左shang边界点
void LLC::extractfisheyePointsInUpLeft(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr threepoints,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    output_cloud->clear();
    if (threepoints->points[0].x < 0 && threepoints->points[0].y < 0)
    {
        for (const auto &point : input_cloud.points)
        {
            // 检查 x 和 Z 值是否在所需的范围内
            if (point.x >= threepoints->points[1].x && point.x <= threepoints->points[2].x && point.z >= threepoints->points[2].z && point.z <= threepoints->points[1].z)
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
            // 检查 y 和 Z 值是否在所需的范围内
            if (point.y >= threepoints->points[1].y && point.y <= threepoints->points[2].y && point.z >= threepoints->points[2].z && point.z <= threepoints->points[1].z)
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
// 计算板子上点云的质心
//  Eigen::Vector3f LLC::computelidarplaneCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
//  {
//      Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);

//     for (const auto& point : cloud->points)
//     {
//         centroid[0] += point.x;
//         centroid[1] += point.y;
//         centroid[2] += point.z;
//     }

//     size_t numPoints = cloud->size();
//     centroid /= static_cast<float>(numPoints);
//     std::cout << centroid(0) <<";"<< centroid(1) << ";"<<centroid(2) <<std::endl;
//     return centroid;
// }
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
// 计算在相机下的板子的平面方程
std::vector<double> LLC::calculatecam_plane_equation(const cv::Mat &points)
{
    if (points.rows != 5 || points.cols != 3)
    {
        std::cerr << "Error: Input matrix should be 5x3." << std::endl;
        return std::vector<double>();
    }

    Eigen::Vector3f vec1(points.at<double>(1, 0) - points.at<double>(0, 0),
                         points.at<double>(1, 1) - points.at<double>(0, 1),
                         points.at<double>(1, 2) - points.at<double>(0, 2));

    Eigen::Vector3f vec2(points.at<double>(2, 0) - points.at<double>(1, 0),
                         points.at<double>(2, 1) - points.at<double>(1, 1),
                         points.at<double>(2, 2) - points.at<double>(1, 2));

    Eigen::Vector3f normal = vec1.cross(vec2);

    if (normal.z() < 0)
    {
        normal *= -1;
    }
    normal.normalize();

    double d = -(normal.x() * points.at<double>(0, 0) + normal.y() * points.at<double>(0, 1) + normal.z() * points.at<double>(0, 2));

    Eigen::Vector3f vec11(points.at<double>(3, 0) - points.at<double>(2, 0),
                          points.at<double>(3, 1) - points.at<double>(2, 1),
                          points.at<double>(3, 2) - points.at<double>(2, 2));

    Eigen::Vector3f vec21(points.at<double>(3, 0) - points.at<double>(0, 0),
                          points.at<double>(3, 1) - points.at<double>(0, 1),
                          points.at<double>(3, 2) - points.at<double>(0, 2));

    Eigen::Vector3f normal1 = vec11.cross(vec21);

    if (normal1.z() < 0)
    {
        normal1 *= -1;
    }
    normal1.normalize();

    double d1 = -(normal1.x() * points.at<double>(0, 0) + normal1.y() * points.at<double>(0, 1) + normal1.z() * points.at<double>(0, 2));
    std::vector<double> plane_equation = {(normal[0] + normal1[0]) / 2, (normal[1] + normal1[1]) / 2, (normal[2] + normal1[2]) / 2, (d + d1) / 2};
    return plane_equation;
}
// 从json文件读取对应key的参数
std::string LLC::readStringFromJsonFile(const std::string &filename, const std::string &key)
{
    std::ifstream file(filename);
    Json::Value root;
    file >> root;
    file.close();
    if (root.isMember(key))
    {
        return root[key].asString();
    }
    else
    {
        std::cout << "找不到参数 " << key << std::endl;
    }
}

cv::Mat LLC::readMatrixFromJSON(const std::string &filename, const std::string &key)
{
    // 读取 JSON 文件
    int index = 0;
    cv::Mat matrix(3, 3, CV_64F);
    std::ifstream file(filename);
    Json::Value root;
    file >> root;
    for (auto &element : root[key])
    {
        matrix.at<double>(index / 3, index % 3) = element.asDouble();
        ++index;
    }
    std::cout << matrix << std::endl;
    return matrix;
}

// 读取是否使用鱼眼标定
bool LLC::readfisheyeFromJsonFile(const std::string &filename, const std::string &key)
{

    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open JSON file: " << filename << std::endl;
        return "";
    }
    else
    {
        Json::Value root;
        file >> root;
        file.close();
        if (root.isMember(key))
        {
            return root[key].asBool();
        }
        else
        {
            std::cout << "找不到参数 " << key << std::endl;
        }
    }
}

// std::string LLC::readlidarStringFromJsonFile(const std::string& filename, const std::string& key) {
//     nlohmann::json data;
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open JSON file: " << filename << std::endl;
//         return "";
//     } else {
//         file >> data;
//         return data[key];
//     }

// }

// 点云数据根据得出的TF投影到图像上
void LLC::projectPointCloudToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, const cv::Mat &cameraMatrix, const cv::Mat &distortionCoefficients,
                                   const Eigen::Matrix3f &estimated_rotation_matrix, const Eigen::Vector3f &translationVector,
                                   const std::string &image_path)
{
    std::vector<Eigen::Vector3f> lidarpoint;
    std::vector<cv::Point3f> lidarpointsinput;
    std::vector<cv::Point2f> lidarpointsoutput;
    for (size_t i = 0; i < pointCloud->size(); ++i)
    {
        Eigen::Vector3f point;
        point(0) = pointCloud->points[i].x;
        point(1) = pointCloud->points[i].y;
        point(2) = pointCloud->points[i].z;
        lidarpoint.push_back(point);
    }
    for (size_t i = 0; i < pointCloud->size(); ++i)
    {
        cv::Point3f cvpoint;
        Eigen::Vector3f transf;
        transf = estimated_rotation_matrix * lidarpoint[i] + translationVector;
        cvpoint.x = transf(0);
        cvpoint.y = transf(1);
        cvpoint.z = transf(2);
        lidarpointsinput.push_back(cvpoint);
    }

    // cv::Mat imagePoints;
    cv::projectPoints(lidarpointsinput, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distortionCoefficients, lidarpointsoutput);
    cv::Mat image = cv::imread(image_path);
    cv::Mat lidarprojecttoimage = image.clone();
    for (size_t i = 0; i < pointCloud->size(); ++i)
    {
        cv::circle(lidarprojecttoimage, lidarpointsoutput[i], 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::namedWindow("lidarpoints with Projection", cv::WINDOW_AUTOSIZE);
    cv::imshow("lidarpoints with Projection", lidarprojecttoimage);
    cv::waitKey(0);
    // std::cout << imagePoints << std::endl;
}
// 计算重投影误差
double LLC::computeReprojectError(const cv::Mat &points, const Eigen::Vector3f &planecentroid, const Eigen::Matrix3f &estimated_rotation_matrix,
                                  const cv::Mat &cameraMatrix, const cv::Mat &distortionCoefficients,
                                  const Eigen::Vector3f &translationVector)
{
    std::vector<cv::Point3f> camcentroid;
    std::vector<cv::Point3f> lidarcentroid;
    cv::Point3f cam;
    cv::Point3f lidar;
    std::vector<cv::Point2f> campixpoint, lidarpixpoint;
    Eigen::Vector3f lidartocam = estimated_rotation_matrix * planecentroid + translationVector;
    int numPoints = points.rows;
    for (int i = 0; i < numPoints; ++i)
    {
        cam.x += points.at<double>(i, 0);
        cam.y += points.at<double>(i, 1);
        cam.z += points.at<double>(i, 2);
    }
    cam /= numPoints;
    camcentroid.push_back(cam);

    lidar.x = lidartocam(0);
    lidar.y = lidartocam(1);
    lidar.z = lidartocam(2);
    lidarcentroid.push_back(lidar);

    cv::projectPoints(camcentroid, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distortionCoefficients, campixpoint);
    cv::projectPoints(lidarcentroid, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distortionCoefficients, lidarpixpoint);
    std::cout << campixpoint[0] << std::endl;
    std::cout << lidarpixpoint[0] << std::endl;
    return std::sqrt(std::pow((campixpoint[0].x - lidarpixpoint[0].x), 2) + std::pow((campixpoint[0].y - lidarpixpoint[0].y), 2));
}
// 计算旋转误差
double LLC::computeRotationError(const std::vector<double> &planelidar_equation, const std::vector<double> &planecam_equation, const Eigen::Matrix3f &estimated_rotation_matrix)
{
    Eigen::Vector3f n_l(planelidar_equation[0], planelidar_equation[1], planelidar_equation[2]);
    Eigen::Vector3f n_c(planecam_equation[0], planecam_equation[1], planecam_equation[2]);
    Eigen::Vector3f n_l_c = estimated_rotation_matrix * n_l;
    double dot_product = n_l_c.dot(n_c);
    double n_l_c_norm = n_l_c.norm();
    double n_c_norm = n_c.norm();
    std::cout << dot_product / (n_l_c_norm * n_c_norm) << std::endl;
    double rotationError = std::acos(std::max(std::min(dot_product / (n_l_c_norm * n_c_norm), 1.0), -1.0)) * (180.0 / M_PI);
    ;
    if (rotationError > 90.0f)
    {
        rotationError = 180.0f - rotationError;
    }
    return rotationError;
}
// 计算平移误差
double LLC::computeTranslationError(const cv::Mat &points, const Eigen::Vector3f &lidarplanecentroid, const Eigen::Matrix3f &estimated_rotation_matrix, const Eigen::Vector3f &translationVector)
{
    cv::Point3f centroid;
    int numPoints = points.rows;
    for (int i = 0; i < numPoints; ++i)
    {
        centroid.x += points.at<double>(i, 0);
        centroid.y += points.at<double>(i, 1);
        centroid.z += points.at<double>(i, 2);
    }
    centroid.x /= numPoints;
    centroid.y /= numPoints;
    centroid.z /= numPoints;

    Eigen::Vector3f planecentroid = estimated_rotation_matrix * lidarplanecentroid + translationVector;

    return std::sqrt(std::pow(planecentroid(0) - centroid.x, 2) + std::pow(planecentroid(1) - centroid.y, 2) + std::pow(planecentroid(2) - centroid.z, 2));
}

// 把得出的TF保存到json文件中
void LLC::writeTransformMatrixToJson(const Eigen::Matrix4f &transform_matrix, const std::string &filename)
{

    Json::Value root;
    Json::Value tf(Json::arrayValue);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            tf.append(transform_matrix(i, j));
        }
    }
    root["transform_matrix"] = tf;
    // 将JSON转换为字符串
    Json::StreamWriterBuilder writerBuilder;
    writerBuilder["indentation"] = ""; // 禁用缩进
    std::ostringstream oss;
    std::unique_ptr<Json::StreamWriter> writer(writerBuilder.newStreamWriter());
    writer->write(root, &oss);

    // 根据字符串进行手动格式化
    std::string jsonStr = oss.str();
    std::string formattedJsonStr;

    int count = 0;
    for (char c : jsonStr)
    {
        formattedJsonStr += c;
        if (c == ',')
            count++;
        if (c == ',' && count % 4 == 0)
        {
            formattedJsonStr += "\n";
        }
    }
    // 将格式化后的字符串写入文件
    std::ofstream file(filename);
    if (file.is_open())
    {
        file << formattedJsonStr;
        file.close();
        std::cout << "JSON写入文件成功！" << std::endl;
    }
    else
    {
        std::cout << "无法打开文件！" << std::endl;
    }
}

// 输入向量A，计算I - AA转置
Eigen::Matrix3f LLC::calculate_A(const Eigen::Vector3f &line_direction)
{
    Eigen::Vector3f direction = line_direction;

    Eigen::Matrix3f matrix_a = Eigen::Matrix3f::Identity() - direction * direction.transpose();

    return matrix_a;
}
// 计算旋转矩阵
Eigen::Matrix3f LLC::init_estimate_R_one_pose(const std::vector<double> &camplane_equation, const std::vector<double> &lidarplane_equation,
                                              const Line3D &upRightCamLineEquation, const Line3D &downRightCamLineEquation, const Line3D &downLeftCamLineEquation, const Line3D &upLeftCamLineEquation,
                                              const Line3D &upRightLineEquation, const Line3D &downRightLineEquation, const Line3D &downLeftLineEquation, const Line3D &upLeftLineEquation,
                                              Eigen::Vector3f &r_estimate_vector_radian, Eigen::Vector3f &r_estimate_vector_degree)
{
    Eigen::Vector3f n_l(lidarplane_equation[0], lidarplane_equation[1], lidarplane_equation[2]);
    Eigen::Vector3f l1_l(downLeftLineEquation.direction[0], downLeftLineEquation.direction[1], downLeftLineEquation.direction[2]);
    Eigen::Vector3f l2_l(upLeftLineEquation.direction[0], upLeftLineEquation.direction[1], upLeftLineEquation.direction[2]);
    Eigen::Vector3f l3_l(upRightLineEquation.direction[0], upRightLineEquation.direction[1], upRightLineEquation.direction[2]);
    Eigen::Vector3f l4_l(downRightLineEquation.direction[0], downRightLineEquation.direction[1], downRightLineEquation.direction[2]);

    Eigen::Matrix<float, 3, 5> m_l;
    m_l << n_l, l1_l, l2_l, l3_l, l4_l;

    Eigen::Vector3f n_c(camplane_equation[0], camplane_equation[1], camplane_equation[2]);
    Eigen::Vector3f l1_c(downLeftCamLineEquation.direction[0], downLeftCamLineEquation.direction[1], downLeftCamLineEquation.direction[2]);
    Eigen::Vector3f l2_c(upLeftCamLineEquation.direction[0], upLeftCamLineEquation.direction[1], upLeftCamLineEquation.direction[2]);
    Eigen::Vector3f l3_c(upRightCamLineEquation.direction[0], upRightCamLineEquation.direction[1], upRightCamLineEquation.direction[2]);
    Eigen::Vector3f l4_c(downRightCamLineEquation.direction[0], downRightCamLineEquation.direction[1], downRightCamLineEquation.direction[2]);

    Eigen::Matrix<float, 3, 5> m_c;
    m_c << n_c, l1_c, l2_c, l3_c, l4_c;

    Eigen::Matrix3f m = m_l * m_c.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f r_estimate_matrix = svd.matrixV() * svd.matrixU().transpose();
    Eigen::Vector3f estimate_vector_radian = r_estimate_matrix.eulerAngles(0, 1, 2);
    Eigen::Vector3f estimate_vector_degree = estimate_vector_radian * (180.0f / M_PI);
    r_estimate_vector_radian = estimate_vector_radian;
    r_estimate_vector_degree = estimate_vector_degree;

    std::cout << "eulerangles" << r_estimate_vector_radian << std::endl;
    // cv::eigen2cv(estimate_vector_degree, r_estimate_vector_degree);
    // cv::Mat rotation_matrix;
    // cv::eigen2cv(r_estimate_matrix, rotation_matrix);
    return r_estimate_matrix;
}
// 计算平移向量
Eigen::Vector3f LLC::init_estimate_t_one_pose(const std::vector<double> &camplane_equation, const Line3D &upRightCamLineEquation,
                                              const Line3D &downRightCamLineEquation, const Line3D &downLeftCamLineEquation, const Line3D &upLeftCamLineEquation,
                                              const Eigen::Matrix3f &estimated_rotation_matrix, const Eigen::Vector3f &planecentroid, const Eigen::Vector3f &uprightcentroid,
                                              const Eigen::Vector3f &downrightcentroid, const Eigen::Vector3f &downleftcentroid, const Eigen::Vector3f &upleftcentroid)
{
    Eigen::Vector3f n_c(camplane_equation[0], camplane_equation[1], camplane_equation[2]);
    double d_c = camplane_equation[3];

    Eigen::Vector3f p1_c(downLeftCamLineEquation.point[0], downLeftCamLineEquation.point[1], downLeftCamLineEquation.point[2]);
    Eigen::Vector3f p2_c(upLeftCamLineEquation.point[0], upLeftCamLineEquation.point[1], upLeftCamLineEquation.point[2]);
    Eigen::Vector3f p3_c(upRightCamLineEquation.point[0], upRightCamLineEquation.point[1], upRightCamLineEquation.point[2]);
    Eigen::Vector3f p4_c(downRightCamLineEquation.point[0], downRightCamLineEquation.point[1], downRightCamLineEquation.point[2]);

    Eigen::Vector3f l1_c(downLeftCamLineEquation.direction[0], downLeftCamLineEquation.direction[1], downLeftCamLineEquation.direction[2]);
    Eigen::Vector3f l2_c(upLeftCamLineEquation.direction[0], upLeftCamLineEquation.direction[1], upLeftCamLineEquation.direction[2]);
    Eigen::Vector3f l3_c(upRightCamLineEquation.direction[0], upRightCamLineEquation.direction[1], upRightCamLineEquation.direction[2]);
    Eigen::Vector3f l4_c(downRightCamLineEquation.direction[0], downRightCamLineEquation.direction[1], downRightCamLineEquation.direction[2]);

    Eigen::Matrix3f matrix_A_1 = calculate_A(l1_c);
    Eigen::Matrix3f matrix_A_2 = calculate_A(l2_c);
    Eigen::Matrix3f matrix_A_3 = calculate_A(l3_c);
    Eigen::Matrix3f matrix_A_4 = calculate_A(l4_c);

    Eigen::MatrixXf matrix_left(13, 3);
    matrix_left.row(0) = n_c.transpose();
    matrix_left.block(1, 0, 3, 3) = matrix_A_1;
    matrix_left.block(4, 0, 3, 3) = matrix_A_2;
    matrix_left.block(7, 0, 3, 3) = matrix_A_3;
    matrix_left.block(10, 0, 3, 3) = matrix_A_4;

    Eigen::VectorXf matrix_right(13);
    matrix_right[0] = -(n_c.transpose() * (estimated_rotation_matrix * planecentroid) + d_c);
    matrix_right.segment(1, 3) = -(matrix_A_1 * (estimated_rotation_matrix * downleftcentroid - p1_c));
    matrix_right.segment(4, 3) = -(matrix_A_2 * (estimated_rotation_matrix * upleftcentroid - p2_c));
    matrix_right.segment(7, 3) = -(matrix_A_3 * (estimated_rotation_matrix * uprightcentroid - p3_c));
    matrix_right.segment(10, 3) = -(matrix_A_4 * (estimated_rotation_matrix * downrightcentroid - p4_c));

    Eigen::Vector3f estimated_t = (matrix_left.transpose() * matrix_left).inverse() * matrix_left.transpose() * matrix_right;
    return estimated_t;
}
// 计算相机下板子边的直线点向式
Line3D LLC::calculatecamLineEquation(const cv::Mat &points, int index1, int index2)
{
    Line3D line_equation;
    // std::cout << points.at<double>(index1, 0) << std::endl;
    line_equation.point = Eigen::Vector3f(points.at<double>(index1, 0), points.at<double>(index1, 1), points.at<double>(index1, 2));
    line_equation.direction = Eigen::Vector3f(points.at<double>(index2, 0), points.at<double>(index2, 1), points.at<double>(index2, 2)) - line_equation.point;
    line_equation.direction.normalize();
    return line_equation;
}
// 计算相机下棋盘格板子的四个角点
void LLC::extractROI(const sensor_msgs::Image::ConstPtr &img, cv::Mat &corner_points, cv::Mat &cameraMatrix)
{
    ROS_INFO_STREAM("enter extractROI");
    cv::Mat corner_vectors = cv::Mat::eye(3, 5, CV_64F);
    cv::Mat chessboard_normal = cv::Mat(1, 3, CV_64F);
    // checkerboard corners, middle square corners, board corners and centre
    std::vector<cv::Point2f> image_points, imagePoints1, imagePoints;

    //////////////// IMAGE FEATURES //////////////////
    if (true)
    {
        std::cout << " ========    IMAGE FEATURES    ======== " << std::endl;
        ROS_INFO_STREAM(" start to extract image feature ... ");
        cv_bridge::CvImagePtr cv_ptr;
        cv::Size2i patternNum(8, 6);
        cv::Size2i patternSize(squaresize, squaresize);

        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        //  cv::Mat ori_img = cv_ptr->image.clone();
        //  undistort_img(ori_img, cv_ptr->image);

        cv::Mat gray;
        std::vector<cv::Point2f> corners, corners_undistorted; // corners will be filled by the detected corners
        std::vector<cv::Point3f> grid3dpoint;
        cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
        // ROS_INFO_STREAM("gray map cols: " << gray.cols << " gray map rows: " << gray.rows);
        // Find checkerboard pattern in the image
        bool patternfound = cv::findChessboardCorners(gray, patternNum, corners,
                                                      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        if (patternfound)
        {
            // Find corner points with sub-pixel accuracy
            ROS_INFO_STREAM("patternfound!");
            cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            // 角点绘制
            cv::drawChessboardCorners(cv_ptr->image, patternNum, corners, patternfound);
            cv::Size imgsize;
            imgsize.height = cv_ptr->image.rows;
            imgsize.width = cv_ptr->image.cols;
            double tx, ty; // Translation values
            // Location of board frame origin from the bottom left inner corner of the checkerboard
            tx = (patternNum.height - 1) * patternSize.height / 2;
            ty = (patternNum.width - 1) * patternSize.width / 2;
            // Board corners w.r.t board frame
            for (int i = 0; i < patternNum.height; i++)
            {
                for (int j = 0; j < patternNum.width; j++)
                {
                    cv::Point3f tmpgrid3dpoint;
                    // Translating origin from bottom left corner to the centre of the checkerboard
                    tmpgrid3dpoint.x = i * patternSize.height - tx;
                    tmpgrid3dpoint.y = j * patternSize.width - ty;
                    tmpgrid3dpoint.z = 0;
                    grid3dpoint.push_back(tmpgrid3dpoint);
                }
            }
            std::vector<cv::Point3f> boardcorners;
            // Board corner coordinates from the centre of the checkerboard
            boardcorners.push_back(
                cv::Point3f((boardwidth) / 2,
                            (boardlength) / 2, 0.0));
            boardcorners.push_back(
                cv::Point3f(-(boardwidth) / 2,
                            (boardlength) / 2, 0.0));
            boardcorners.push_back(
                cv::Point3f(-(boardwidth) / 2,
                            -(boardlength) / 2, 0.0));
            boardcorners.push_back(
                cv::Point3f((boardwidth) / 2,
                            -(boardlength) / 2, 0.0));
            // Board centre coordinates from the centre of the checkerboard (due to incorrect placement of checkerbord on board)
            boardcorners.push_back(cv::Point3f(-0 / 2,
                                               -0 / 2, 0.0));

            std::vector<cv::Point3f> square_edge;
            // centre checkerboard square corner coordinates wrt the centre of the checkerboard (origin)
            square_edge.push_back(cv::Point3f(-squaresize / 2, -squaresize / 2, 0.0));
            square_edge.push_back(cv::Point3f(squaresize / 2, squaresize / 2, 0.0));
            cv::Mat rvec(3, 3, cv::DataType<double>::type); // Initialization for pinhole and fisheye cameras
            cv::Mat tvec(3, 1, cv::DataType<double>::type);

            cv::solvePnP(grid3dpoint, corners, cameraMatrix, distortionCoefficients, rvec, tvec);
            cv::projectPoints(grid3dpoint, rvec, tvec, cameraMatrix, distortionCoefficients, image_points);
            // Mark the centre square corner points
            cv::projectPoints(square_edge, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints1);
            cv::projectPoints(boardcorners, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);
            // }

            // chessboardpose is a 3*4 transform matrix that transforms points in board frame to camera frame | R&T
            cv::Mat chessboardpose = cv::Mat::eye(4, 4, CV_64F);
            cv::Mat tmprmat = cv::Mat(3, 3, CV_64F); // rotation matrix
            cv::Rodrigues(rvec, tmprmat);            // Euler angles to rotation matrix

            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    chessboardpose.at<double>(j, k) = tmprmat.at<double>(j, k);
                }
                chessboardpose.at<double>(j, 3) = tvec.at<double>(j);
            }

            chessboard_normal.at<double>(0) = 0;
            chessboard_normal.at<double>(1) = 0;
            chessboard_normal.at<double>(2) = 1;
            chessboard_normal = chessboard_normal * chessboardpose(cv::Rect(0, 0, 3, 3)).t();

            for (int k = 0; k < boardcorners.size(); k++)
            {
                // take every point in boardcorners set
                cv::Point3f pt(boardcorners[k]);
                for (int i = 0; i < 3; i++)
                {
                    // Transform it to obtain the coordinates in cam frame
                    corner_vectors.at<double>(i, k) = chessboardpose.at<double>(i, 0) * pt.x +
                                                      chessboardpose.at<double>(i, 1) * pt.y +
                                                      chessboardpose.at<double>(i, 3);
                }

                // convert 3D coordinates to image coordinates
                double *img_coord = LLC::converto_imgpts(corner_vectors.at<double>(0, k),
                                                         corner_vectors.at<double>(1, k),
                                                         corner_vectors.at<double>(2, k), cameraMatrix);
                std::cout << "img_coord: (" << img_coord[0] << ", " << img_coord[1] << ")" << std::endl;
                // Mark the corners and the board centre
                if (k == 0)
                    cv::circle(cv_ptr->image, cv::Point(img_coord[0], img_coord[1]),
                               8, CV_RGB(0, 255, 0), -1); // green
                else if (k == 1)
                    cv::circle(cv_ptr->image, cv::Point(img_coord[0], img_coord[1]),
                               8, CV_RGB(255, 255, 0), -1); // yellow
                else if (k == 2)
                    cv::circle(cv_ptr->image, cv::Point(img_coord[0], img_coord[1]),
                               8, CV_RGB(0, 0, 255), -1); // blue
                else if (k == 3)
                    cv::circle(cv_ptr->image, cv::Point(img_coord[0], img_coord[1]),
                               8, CV_RGB(255, 0, 0), -1); // red
                else
                    cv::circle(cv_ptr->image, cv::Point(img_coord[0], img_coord[1]),
                               8, CV_RGB(255, 255, 255), -1); // white for centre
                // delete[] img_coord;
            }

            std::cout << "Translation Vector" << std::endl
                      << chessboardpose << std::endl;
            std::cout << "5 Vector" << std::endl
                      << corner_vectors << std::endl;
            cv::namedWindow("corner points", cv::WINDOW_AUTOSIZE);
            cv::imshow("corner points", cv_ptr->image);
            // cv::waitKey(0);
        }
        else
        { // if (patternfound)
            ROS_ERROR("PATTERN NOT FOUND");
            return;
        }
        ROS_INFO_STREAM(" extract image feature end... ");
        corner_points = corner_vectors.t() * 0.001;
    }
}
// 使用相机和激光得到的板子信息计算tf
void LLC::execute()
{

    fisheye = readfisheyeFromJsonFile(parameters_path, "fisheye");
    if (!fisheye)
    {
        midcameraMatrix = readMatrixFromJSON(parameters_path, "mid_matrix");
        std::cout << "execute" << std::endl;
        image_path = readStringFromJsonFile(parameters_path, "image_path");
        cv_bridge::CvImage cv_image;
        cv::Mat image = cv::imread(image_path);
        cv_image.header.stamp = ros::Time::now();
        cv_image.header.frame_id = "camera_frame";
        cv_image.encoding = "bgr8";
        cv_image.image = image;
        sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
        extractROI(msg, corner_points, midcameraMatrix);

        pcl::visualization::PCLVisualizer viewer("pc_viewer");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());

        lidar_path = readStringFromJsonFile(parameters_path, "lidar_path");
        if (pcl::io::loadPCDFile(lidar_path, *cloud))
        {
            exit(-1);
        }
        // 把点云投影到一个平面上
        //********************************
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        std::cout << "PointCloud after segmentation has: "
                  << inliers->indices.size() << " inliers." << std::endl;

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);
        std::cerr << "PointCloud after projection has: " << cloud_projected->size() << " data points." << std::endl;
        //********************************

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
        // for (int i = 0; i < contour_point_size; i++) {
        //     std::cout << cloud_hull->points[i] << std::endl;
        // }
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
        // for (int i = 0; i < contour_point_vec.size(); i++)
        // {
        //     std::cout << contour_point_vec[i].index << " " << contour_point_vec[i].angle << '\n';
        //     // std::cout << a << std::endl;
        // }
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
        //********************************
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

        if (upRightCamLineEquation.direction[0] < 0)
        {
            upRightCamLineEquation.direction *= -1;
        }
        if (downRightCamLineEquation.direction[0] > 0)
        {
            downRightCamLineEquation.direction *= -1;
        }
        if (downLeftCamLineEquation.direction[0] > 0)
        {
            downLeftCamLineEquation.direction *= -1;
        }
        if (upLeftCamLineEquation.direction[0] < 0)
        {
            upLeftCamLineEquation.direction *= -1;
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
        // uprightcentroid = computelidarplaneCentroid(uprightpoints);
        // downrightcentroid = computelidarplaneCentroid(downrightpoints);
        // downleftcentroid = computelidarplaneCentroid(downleftpoints);
        // upleftcentroid = computelidarplaneCentroid(upleftpoints);

        // 板子在激光下的中点
        planecentroid = computeLidarPlaneCentroid(lidar_corner);
        // computelidarplaneCentroid(cloud_projected);

        // 板子在激光下的平面方程
        planelidar_equation = calculatelidar_plane_equation(cloud_projected);
        // 板子在相机下的平面方程
        planecam_equation = calculatecam_plane_equation(corner_points);

        Eigen::Matrix3f rotation_matrix = init_estimate_R_one_pose(planecam_equation, planelidar_equation, upRightCamLineEquation, downRightCamLineEquation,
                                                                   downLeftCamLineEquation, upLeftCamLineEquation, upRightLineEquation, downRightLineEquation,
                                                                   downLeftLineEquation, upLeftLineEquation, r_estimate_vector_radian, r_estimate_vector_degree);
        Eigen::Vector3f estimated_t = init_estimate_t_one_pose(planecam_equation, upRightCamLineEquation, downRightCamLineEquation, downLeftCamLineEquation,
                                                               upLeftCamLineEquation, rotation_matrix, planecentroid, uprightcentroid, downrightcentroid,
                                                               downleftcentroid, upleftcentroid);
        // 保存tf为4*4格式
        Eigen::Matrix4f transformMatrix;
        transformMatrix.setIdentity();
        // Set rotation matrix in top-left 3x3 submatrix
        transformMatrix.block(0, 0, 3, 3) = rotation_matrix;
        // Set translation vector in last column
        transformMatrix.block(0, 3, 3, 1) = estimated_t;

        T_error = computeTranslationError(corner_points, planecentroid, rotation_matrix, estimated_t);
        R_error = computeRotationError(planelidar_equation, planecam_equation, rotation_matrix);
        // Reproject_error = computeReprojectError(corner_points, planecentroid, rotation_matrix, midcameraMatrix, distortionCoefficients, estimated_t);

        std::cout << "平移误差:" << T_error << std::endl;
        std::cout << "旋转误差:" << R_error << std::endl;
        std::cout << "重投影误差:" << Reproject_error << std::endl;

        std::cout << "交点坐标：" << uprightlidarcorner.x() << ", " << uprightlidarcorner.y() << ", " << uprightlidarcorner.z() << std::endl;
        std::cout << "交点坐标：" << downrightlidarcorner.x() << ", " << downrightlidarcorner.y() << ", " << downrightlidarcorner.z() << std::endl;
        std::cout << "交点坐标：" << downleftlidarcorner.x() << ", " << downleftlidarcorner.y() << ", " << downleftlidarcorner.z() << std::endl;
        std::cout << "交点坐标：" << upleftlidarcorner.x() << ", " << upleftlidarcorner.y() << ", " << upleftlidarcorner.z() << std::endl;
        // std::cout << "直线的点向式表达：" << std::endl;
        // std::cout << "P = (" << upRightLineEquation.point.x() << ", " << upRightLineEquation.point.y() << ", " << upRightLineEquation.point.z() << ") + t * ";
        // std::cout << "V = (" << upRightLineEquation.direction.x() << ", " << upRightLineEquation.direction.y() << ", " << upRightLineEquation.direction.z() << ")" << std::endl;
        // std::cout << "直线的点向式表达：" << std::endl;
        // std::cout << "P = (" << downLeftLineEquation.point.x() << ", " << downLeftLineEquation.point.y() << ", " << downLeftLineEquation.point.z() << ") + t * ";
        // std::cout << "V = (" << downLeftLineEquation.direction.x() << ", " << downLeftLineEquation.direction.y() << ", " << downLeftLineEquation.direction.z() << ")" << std::endl;

        if (Reproject_error < 20)
        {
            writeTransformMatrixToJson(transformMatrix, saveTfmatrix_path);
            projectPointCloudToImage(cloud_projected, midcameraMatrix, distortionCoefficients, rotation_matrix, estimated_t, image_path);
        }

        // 结果可视化
        //*******************************************
        pcl::PlanarPolygon<pcl::PointXYZ> polygon;
        polygon.setContour(contour);
        // std::cout<< uprightpoints->points.size() + upleftpoints->points.size() + downrightpoints->points.size() + upleftpoints->points.size() << std::endl;
        int viewer1(0);
        // viewer.createViewPort(0.0, 0.0, 0.5, 1, viewer1);
        viewer.setBackgroundColor(255, 255, 255, viewer1);
        viewer.addPolygon(polygon, 0, 0, 0, "polygon", viewer1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(cloud_projected, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, singleColor, "head", viewer1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head", viewer1);

        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_hull, 0, 255, 255);
        // viewer.addPointCloud<pcl::PointXYZ>(cloud_hull, color, "head1", viewer1);
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head1", viewer1);

        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> corners_color(corners_cloud, 255, 0, 0);
        // viewer.addPointCloud<pcl::PointXYZ>(corners_cloud, corners_color, "head2", viewer1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upright_color(projectuprightpoints, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZ>(projectuprightpoints, upright_color, "head1", viewer1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head1", viewer1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downright_color(projectdownrightpoints, 255, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ>(projectdownrightpoints, downright_color, "head2", viewer1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head2", viewer1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downleft_color(projectdownleftpoints, 0, 0, 255);
        viewer.addPointCloud<pcl::PointXYZ>(projectdownleftpoints, downleft_color, "head3", viewer1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head3", viewer1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upleft_color(projectupleftpoints, 0, 255, 255);
        viewer.addPointCloud<pcl::PointXYZ>(projectupleftpoints, upleft_color, "head4", viewer1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head4", viewer1);
        linea = visualizeLine(upRightLineEquation);
        lineb = visualizeLine(downRightLineEquation);
        linec = visualizeLine(downLeftLineEquation);
        lined = visualizeLine(upLeftLineEquation);
        viewer.addLine<pcl::PointXYZ>(linea->points[0], linea->points[1], "line1");
        viewer.addLine<pcl::PointXYZ>(lineb->points[0], lineb->points[1], "line2");
        viewer.addLine<pcl::PointXYZ>(linec->points[0], linec->points[1], "line3");
        viewer.addLine<pcl::PointXYZ>(lined->points[0], lined->points[1], "line4");
        //*******************************************

        while (!viewer.wasStopped())
        {
            if (is_exit)
            {
                break;
            }
            viewer.spinOnce(100);
        }
    }
    else
    {
        kuangshan = readStringFromJsonFile(parameters_path, "kuangshan");
        std::cout << "execute" << std::endl;
        fishcameraMatrix = readMatrixFromJSON(parameters_path, "fish_matrix");
        // 读取图像信息
        image_path = readStringFromJsonFile(parameters_path, "image_path");
        cv_bridge::CvImage cv_image;
        cv::Mat image = cv::imread(image_path);
        cv_image.header.stamp = ros::Time::now();
        cv_image.header.frame_id = "camera_frame";
        cv_image.encoding = "bgr8";
        cv_image.image = image;
        sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
        extractROI(msg, corner_points, fishcameraMatrix);

        pcl::visualization::PCLVisualizer viewer("pc_viewer");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());

        // 读取激光信息
        lidar_path = readStringFromJsonFile(parameters_path, "lidar_path");
        if (pcl::io::loadPCDFile(lidar_path, *cloud))
        {
            exit(-1);
        }

        // 把点云投影到一个平面上
        //********************************
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        std::cout << "PointCloud after segmentation has: " << inliers->indices.size() << " inliers." << std::endl;

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);
        std::cerr << "PointCloud after projection has: " << cloud_projected->size() << " data points." << std::endl;
        //********************************

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
        // for (int i = 0; i < contour_point_size; i++) {
        //     std::cout << cloud_hull->points[i] << std::endl;
        // }
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

        // for (int i = 0; i < contour_point_vec.size(); i++)
        // {
        //     std::cout << contour_point_vec[i].index << " " << contour_point_vec[i].angle << '\n';
        //     // std::cout << a << std::endl;
        // }
        //********************************

        // 取出角度大的前十个，并从中筛选出板子的3个角点
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

        if (kuangshan == "wuhu")
        {
            wuhugetthreepoints(corners_cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr uprightpoints(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr upleftpoints(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ> contour;
            contour.width = cloud_hull->width;
            contour.height = 1;
            contour.is_dense = false;
            contour.resize(contour.height * contour.width);
            for (int i = 0; i < cloud_hull->points.size(); i++)
            {
                contour.points[i] = cloud_hull->points[i];
            }
            // 提取左shang右shang边界上的点
            extractfisheyePointsInUpRight(contour, corners_cloud, uprightpoints);
            extractfisheyePointsInUpLeft(contour, corners_cloud, upleftpoints);
            // 过滤影响拟合直线的点
            filterUpandDownRightPoints(uprightpoints);
            filterUpandDownLeftPoints(upleftpoints);
            // 拟合激光直线
            upRightLineEquation = getLidarLineEquation(uprightpoints);
            upLeftLineEquation = getLidarLineEquation(upleftpoints);
            // 通过平移计算另外两条缺失的直线
            downRightLineEquation = translateLinedownright(upLeftLineEquation, upRightLineEquation, 1.22);
            downLeftLineEquation = translateLinedownleft(upRightLineEquation, upLeftLineEquation, 0.85);
            // 根据相机得到的板子角点计算四条边直线
            // upRightCamLineEquation = calculatecamLineEquation(corner_points, 1, 2);
            // downRightCamLineEquation = calculatecamLineEquation(corner_points, 0, 1);
            // downLeftCamLineEquation = calculatecamLineEquation(corner_points, 3, 0);
            // upLeftCamLineEquation = calculatecamLineEquation(corner_points, 2, 3);

            // 对激光和相机得到的板子四条边直线方向归一化
            //********************************
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

            if (upRightCamLineEquation.direction[0] < 0)
            {
                upRightCamLineEquation.direction *= -1;
            }
            if (downRightCamLineEquation.direction[0] > 0)
            {
                downRightCamLineEquation.direction *= -1;
            }
            if (downLeftCamLineEquation.direction[0] > 0)
            {
                downLeftCamLineEquation.direction *= -1;
            }
            if (upLeftCamLineEquation.direction[0] < 0)
            {
                upLeftCamLineEquation.direction *= -1;
            }
            //********************************

            // 根据算的直线方程计算交点得到激光下板子的角点
            uprightlidarcorner = computeLineIntersection(upRightLineEquation, upLeftLineEquation);
            downrightlidarcorner = computeLineIntersection(upRightLineEquation, downRightLineEquation);
            downleftlidarcorner = computeLineIntersection(downRightLineEquation, downLeftLineEquation);
            upleftlidarcorner = computeLineIntersection(downLeftLineEquation, upLeftLineEquation);
            cv::Mat lidar_corner = (cv::Mat_<double>(4, 3) << uprightlidarcorner(0), uprightlidarcorner(1), uprightlidarcorner(2),
                                    downrightlidarcorner(0), downrightlidarcorner(1), downrightlidarcorner(2),
                                    downleftlidarcorner(0), downleftlidarcorner(1), downleftlidarcorner(2),
                                    upleftlidarcorner(0), upleftlidarcorner(1), upleftlidarcorner(2));
            ////计算激光下板子四个边的中点
            uprightcentroid = computeLidarLineCentroid(lidar_corner, 0, 1);
            downrightcentroid = computeLidarLineCentroid(lidar_corner, 1, 2);
            downleftcentroid = computeLidarLineCentroid(lidar_corner, 2, 3);
            upleftcentroid = computeLidarLineCentroid(lidar_corner, 3, 0);
            // 板子在激光下的中点
            planecentroid = computeLidarPlaneCentroid(lidar_corner);
            // 板子在激光下的平面方程

            // computelidarplaneCentroid(cloud_projected);
            planelidar_equation = calculatelidar_plane_equation(cloud_projected);
            // 板子在相机下的平面方程
            // planecam_equation = calculatecam_plane_equation(corner_points);

            Eigen::Matrix3f rotation_matrix = init_estimate_R_one_pose(planecam_equation, planelidar_equation, upRightCamLineEquation, downRightCamLineEquation,
                                                                       downLeftCamLineEquation, upLeftCamLineEquation, upRightLineEquation, downRightLineEquation,
                                                                       downLeftLineEquation, upLeftLineEquation, r_estimate_vector_radian, r_estimate_vector_degree);
            Eigen::Vector3f estimated_t = init_estimate_t_one_pose(planecam_equation, upRightCamLineEquation, downRightCamLineEquation, downLeftCamLineEquation,
                                                                   upLeftCamLineEquation, rotation_matrix, planecentroid, uprightcentroid, downrightcentroid,
                                                                   downleftcentroid, upleftcentroid);
            // 保存tf为4*4格式
            Eigen::Matrix4f transformMatrix;
            transformMatrix.setIdentity();
            // Set rotation matrix in top-left 3x3 submatrix
            transformMatrix.block(0, 0, 3, 3) = rotation_matrix;
            // Set translation vector in last column
            transformMatrix.block(0, 3, 3, 1) = estimated_t;

            T_error = computeTranslationError(corner_points, planecentroid, rotation_matrix, estimated_t);
            R_error = computeRotationError(planelidar_equation, planecam_equation, rotation_matrix);
            // Reproject_error = computeReprojectError(corner_points, planecentroid, rotation_matrix, fishcameraMatrix, distortionCoefficients, estimated_t);

            std::cout << "平移误差:" << T_error << std::endl;
            std::cout << "旋转误差:" << R_error << std::endl;
            std::cout << "重投影误差:" << Reproject_error << std::endl;

            std::cout << "交点坐标：" << uprightlidarcorner.x() << ", " << uprightlidarcorner.y() << ", " << uprightlidarcorner.z() << std::endl;
            std::cout << "交点坐标：" << downrightlidarcorner.x() << ", " << downrightlidarcorner.y() << ", " << downrightlidarcorner.z() << std::endl;
            std::cout << "交点坐标：" << downleftlidarcorner.x() << ", " << downleftlidarcorner.y() << ", " << downleftlidarcorner.z() << std::endl;
            std::cout << "交点坐标：" << upleftlidarcorner.x() << ", " << upleftlidarcorner.y() << ", " << upleftlidarcorner.z() << std::endl;
            // std::cout << "直线的点向式表达：" << std::endl;
            // std::cout << "P = (" << upRightLineEquation.point.x() << ", " << upRightLineEquation.point.y() << ", " << upRightLineEquation.point.z() << ") + t * ";
            // std::cout << "V = (" << upRightLineEquation.direction.x() << ", " << upRightLineEquation.direction.y() << ", " << upRightLineEquation.direction.z() << ")" << std::endl;
            // std::cout << "直线的点向式表达：" << std::endl;
            // std::cout << "P = (" << downLeftLineEquation.point.x() << ", " << downLeftLineEquation.point.y() << ", " << downLeftLineEquation.point.z() << ") + t * ";
            // std::cout << "V = (" << downLeftLineEquation.direction.x() << ", " << downLeftLineEquation.direction.y() << ", " << downLeftLineEquation.direction.z() << ")" << std::endl;

            if (Reproject_error < 20)
            {
                writeTransformMatrixToJson(transformMatrix, saveTfmatrix_path);
                projectPointCloudToImage(cloud_projected, fishcameraMatrix, distortionCoefficients, rotation_matrix, estimated_t, image_path);
            }

            // 结果可视化
            //*******************************************
            pcl::PlanarPolygon<pcl::PointXYZ> polygon;
            polygon.setContour(contour);
            // std::cout<< uprightpoints->points.size() + upleftpoints->points.size() + downrightpoints->points.size() + upleftpoints->points.size() << std::endl;
            int viewer1(0);
            // viewer.createViewPort(0.0, 0.0, 0.5, 1, viewer1);
            viewer.setBackgroundColor(255, 255, 255, viewer1);
            viewer.addPolygon(polygon, 0, 0, 0, "polygon", viewer1);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(cloud_projected, 0, 255, 0);
            viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, singleColor, "head", viewer1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head", viewer1);
            linea = visualizeLine(upRightLineEquation);
            lineb = visualizeLine(downRightLineEquation);
            linec = visualizeLine(downLeftLineEquation);
            lined = visualizeLine(upLeftLineEquation);
            viewer.addLine<pcl::PointXYZ>(linea->points[0], linea->points[1], "line1");
            viewer.addLine<pcl::PointXYZ>(lineb->points[0], lineb->points[1], "line2");
            viewer.addLine<pcl::PointXYZ>(linec->points[0], linec->points[1], "line3");
            viewer.addLine<pcl::PointXYZ>(lined->points[0], lined->points[1], "line4");
            //*******************************************
        }
        else
        {
            getthreepoints(corners_cloud);
            //********************************

            pcl::PointCloud<pcl::PointXYZ>::Ptr downrightpoints(new pcl::PointCloud<pcl::PointXYZ>);
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
            // 提取左下右下边界上的点
            extractfisheyePointsInDownRight(contour, corners_cloud, downrightpoints);
            extractfisheyePointsInDownLeft(contour, corners_cloud, downleftpoints);
            // 过滤影响拟合直线的点
            filterUpandDownRightPoints(downrightpoints);
            filterUpandDownLeftPoints(downleftpoints);

            // 拟合激光直线
            downRightLineEquation = getLidarLineEquation(downrightpoints);
            downLeftLineEquation = getLidarLineEquation(downleftpoints); // huan cheng zhijie jisuan lianggdian zhixian fangcheng
            // 通过平移计算另外两条缺失的直线
            upRightLineEquation = translateLineupright(downLeftLineEquation, downRightLineEquation, 0.90);
            upLeftLineEquation = translateLineupleft(downRightLineEquation, downLeftLineEquation, 1.27);
            // 根据相机得到的板子角点计算四条边直线
            upRightCamLineEquation = calculatecamLineEquation(corner_points, 1, 2);
            downRightCamLineEquation = calculatecamLineEquation(corner_points, 0, 1);
            downLeftCamLineEquation = calculatecamLineEquation(corner_points, 3, 0);
            upLeftCamLineEquation = calculatecamLineEquation(corner_points, 2, 3);

            // 对激光和相机得到的板子四条边直线方向归一化
            //********************************
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

            if (upRightCamLineEquation.direction[0] < 0)
            {
                upRightCamLineEquation.direction *= -1;
            }
            if (downRightCamLineEquation.direction[0] > 0)
            {
                downRightCamLineEquation.direction *= -1;
            }
            if (downLeftCamLineEquation.direction[0] > 0)
            {
                downLeftCamLineEquation.direction *= -1;
            }
            if (upLeftCamLineEquation.direction[0] < 0)
            {
                upLeftCamLineEquation.direction *= -1;
            }
            //********************************

            // 根据算的直线方程计算交点得到激光下板子的角点
            uprightlidarcorner = computeLineIntersection(upRightLineEquation, upLeftLineEquation);
            downrightlidarcorner = computeLineIntersection(upRightLineEquation, downRightLineEquation);
            downleftlidarcorner = computeLineIntersection(downRightLineEquation, downLeftLineEquation);
            upleftlidarcorner = computeLineIntersection(downLeftLineEquation, upLeftLineEquation);
            cv::Mat lidar_corner = (cv::Mat_<double>(4, 3) << uprightlidarcorner(0), uprightlidarcorner(1), uprightlidarcorner(2),
                                    downrightlidarcorner(0), downrightlidarcorner(1), downrightlidarcorner(2),
                                    downleftlidarcorner(0), downleftlidarcorner(1), downleftlidarcorner(2),
                                    upleftlidarcorner(0), upleftlidarcorner(1), upleftlidarcorner(2));
            ////计算激光下板子四个边的中点
            uprightcentroid = computeLidarLineCentroid(lidar_corner, 0, 1);
            downrightcentroid = computeLidarLineCentroid(lidar_corner, 1, 2);
            downleftcentroid = computeLidarLineCentroid(lidar_corner, 2, 3);
            upleftcentroid = computeLidarLineCentroid(lidar_corner, 3, 0);
            // uprightcentroid = computelidarplaneCentroid(uprightpoints);
            // downrightcentroid = computelidarplaneCentroid(downrightpoints);
            // downleftcentroid = computelidarplaneCentroid(downleftpoints);
            // upleftcentroid = computelidarplaneCentroid(upleftpoints);

            // 板子在激光下的中点
            planecentroid = computeLidarPlaneCentroid(lidar_corner);
            // 板子在激光下的平面方程

            // computelidarplaneCentroid(cloud_projected);
            planelidar_equation = calculatelidar_plane_equation(cloud_projected);
            // 板子在相机下的平面方程
            // planecam_equation = calculatecam_plane_equation(corner_points);

            Eigen::Matrix3f rotation_matrix = init_estimate_R_one_pose(planecam_equation, planelidar_equation, upRightCamLineEquation, downRightCamLineEquation,
                                                                       downLeftCamLineEquation, upLeftCamLineEquation, upRightLineEquation, downRightLineEquation,
                                                                       downLeftLineEquation, upLeftLineEquation, r_estimate_vector_radian, r_estimate_vector_degree);
            Eigen::Vector3f estimated_t = init_estimate_t_one_pose(planecam_equation, upRightCamLineEquation, downRightCamLineEquation, downLeftCamLineEquation,
                                                                   upLeftCamLineEquation, rotation_matrix, planecentroid, uprightcentroid, downrightcentroid,
                                                                   downleftcentroid, upleftcentroid);
            // 保存tf为4*4格式
            Eigen::Matrix4f transformMatrix;
            transformMatrix.setIdentity();
            // Set rotation matrix in top-left 3x3 submatrix
            transformMatrix.block(0, 0, 3, 3) = rotation_matrix;
            // Set translation vector in last column
            transformMatrix.block(0, 3, 3, 1) = estimated_t;

            T_error = computeTranslationError(corner_points, planecentroid, rotation_matrix, estimated_t);
            R_error = computeRotationError(planelidar_equation, planecam_equation, rotation_matrix);
            // Reproject_error = computeReprojectError(corner_points, planecentroid, rotation_matrix, fishcameraMatrix, distortionCoefficients, estimated_t);

            std::cout << "平移误差:" << T_error << std::endl;
            std::cout << "旋转误差:" << R_error << std::endl;
            std::cout << "重投影误差:" << Reproject_error << std::endl;

            std::cout << "交点坐标：" << uprightlidarcorner.x() << ", " << uprightlidarcorner.y() << ", " << uprightlidarcorner.z() << std::endl;
            std::cout << "交点坐标：" << downrightlidarcorner.x() << ", " << downrightlidarcorner.y() << ", " << downrightlidarcorner.z() << std::endl;
            std::cout << "交点坐标：" << downleftlidarcorner.x() << ", " << downleftlidarcorner.y() << ", " << downleftlidarcorner.z() << std::endl;
            std::cout << "交点坐标：" << upleftlidarcorner.x() << ", " << upleftlidarcorner.y() << ", " << upleftlidarcorner.z() << std::endl;
            // std::cout << "直线的点向式表达：" << std::endl;
            // std::cout << "P = (" << upRightLineEquation.point.x() << ", " << upRightLineEquation.point.y() << ", " << upRightLineEquation.point.z() << ") + t * ";
            // std::cout << "V = (" << upRightLineEquation.direction.x() << ", " << upRightLineEquation.direction.y() << ", " << upRightLineEquation.direction.z() << ")" << std::endl;
            // std::cout << "直线的点向式表达：" << std::endl;
            // std::cout << "P = (" << downLeftLineEquation.point.x() << ", " << downLeftLineEquation.point.y() << ", " << downLeftLineEquation.point.z() << ") + t * ";
            // std::cout << "V = (" << downLeftLineEquation.direction.x() << ", " << downLeftLineEquation.direction.y() << ", " << downLeftLineEquation.direction.z() << ")" << std::endl;

            if (Reproject_error < 20)
            {
                writeTransformMatrixToJson(transformMatrix, saveTfmatrix_path);
                projectPointCloudToImage(cloud_projected, fishcameraMatrix, distortionCoefficients, rotation_matrix, estimated_t, image_path);
            }

            // 结果可视化
            //*******************************************
            pcl::PlanarPolygon<pcl::PointXYZ> polygon;
            polygon.setContour(contour);
            // std::cout<< uprightpoints->points.size() + upleftpoints->points.size() + downrightpoints->points.size() + upleftpoints->points.size() << std::endl;
            int viewer1(0);
            // viewer.createViewPort(0.0, 0.0, 0.5, 1, viewer1);
            viewer.setBackgroundColor(255, 255, 255, viewer1);
            viewer.addPolygon(polygon, 0, 0, 0, "polygon", viewer1);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(cloud_projected, 0, 255, 0);
            viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, singleColor, "head", viewer1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "head", viewer1);
            linea = visualizeLine(upRightLineEquation);
            lineb = visualizeLine(downRightLineEquation);
            linec = visualizeLine(downLeftLineEquation);
            lined = visualizeLine(upLeftLineEquation);
            viewer.addLine<pcl::PointXYZ>(linea->points[0], linea->points[1], "line1");
            viewer.addLine<pcl::PointXYZ>(lineb->points[0], lineb->points[1], "line2");
            viewer.addLine<pcl::PointXYZ>(linec->points[0], linec->points[1], "line3");
            viewer.addLine<pcl::PointXYZ>(lined->points[0], lined->points[1], "line4");
            //*******************************************
        }

        while (!viewer.wasStopped())
        {
            if (is_exit)
            {
                break;
            }
            viewer.spinOnce(100);
        }
    }
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