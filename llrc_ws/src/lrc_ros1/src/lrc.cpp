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
    nh.getParam("radar_position", radar_position);
    nh.getParam("lrc_path", lrc_path);

    // lidar informarion from rosbag

    topic_cloud = lidarprocess(lidar_position, lrc_path, city);

    std::string position;
    if (lidar_position == "front")
    {
        // std::vector<ChessboardProcessResult> left_results;
        // std::vector<ChessboardProcessResult> right_results;

        // 2. 构建KD树
        pcl::KdTreeFLANN<pcl::PointXYZ> left_kdtree;
        pcl::KdTreeFLANN<pcl::PointXYZ> right_kdtree;

        // 假设left_results和right_results中包含了处理后的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // std::vector<float> left_intensities;
        // std::vector<float> right_intensities;

        for (const auto &topic_pair : topic_cloud)
        {
            position.clear();
            const std::string &topic = topic_pair.first;                   // 获取话题名称
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = topic_pair.second; // 获取对应的点云

            // 根据话题名称进行不同的处理
            if (topic == "/front_left_lidar")
            {
                std::unordered_map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> topic_cloud_left;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_left; // 临时存储平面点云
                std::string position = "left";

                // 调用函数
                if (extractPlaneCloud(cloud, plane_pcds_left, position))
                {
                    // 将结果存储到 topic_cloud_left 中
                    topic_cloud_left[position] = plane_pcds_left;
                    for (const auto &plane_pcd_left : plane_pcds_left)
                    {
                        Eigen::Vector3f result = processChessboard(plane_pcd_left, position);
                        pcl::PointXYZ single_point;
                        single_point.x = result[0];
                        single_point.y = result[1];
                        single_point.z = result[2];
                        left_cloud->push_back(single_point);
                    }
                }
            }
            else if (topic == "/front_right_lidar")
            {
                std::unordered_map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> topic_cloud_right;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_right; // 临时存储平面点云
                std::string position = "right";

                // 调用函数
                if (extractPlaneCloud(cloud, plane_pcds_right, position))
                {
                    // 将结果存储到 topic_cloud_left 中
                    topic_cloud_right[position] = plane_pcds_right;

                    for (const auto &plane_pcd_right : plane_pcds_right)
                    {
                        Eigen::Vector3f result = processChessboard(plane_pcd_right, position);
                        pcl::PointXYZ single_point;
                        single_point.x = result[0];
                        single_point.y = result[1];
                        single_point.z = result[2];
                        right_cloud->push_back(single_point);
                    }
                }
                // 对/front_right_lidar对应的点云进行处理
            }
            else
            {
                break;
            }
        }

        
        left_kdtree.setInputCloud(left_cloud);
        right_kdtree.setInputCloud(right_cloud);
        radar_front_sub = nh.subscribe("/" + radar_position + "_radar", 10, &LRC::vRadarCallback, this);


    }
    else if (lidar_position == "back")
    {

        // 2. 构建KD树
        pcl::KdTreeFLANN<pcl::PointXYZ> back_kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr back_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &topic_pair : topic_cloud)
        {
            position.clear();
            const std::string &topic = topic_pair.first;                   // 获取话题名称
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = topic_pair.second; // 获取对应的点云

            if(topic=="/back_lidar")
            {
                std::unordered_map<std::string, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> topic_cloud_back;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_pcds_back; // 临时存储平面点云
                std::string position = "back";

                // 调用函数
                if (extractPlaneCloud(cloud, plane_pcds_back, position))
                {
                    // 将结果存储到 topic_cloud_left 中
                    topic_cloud_back[position] = plane_pcds_back;
                    for (const auto &plane_pcd_back : plane_pcds_back)
                    {
                        Eigen::Vector3f result = processChessboard(plane_pcd_back, position);
                        pcl::PointXYZ single_point;
                        single_point.x = result[0];
                        single_point.y = result[1];
                        single_point.z = result[2];
                        back_cloud->push_back(single_point);
                    }
                }
            }

            back_kdtree.setInputCloud(back_cloud);

            radar_front_sub = nh.subscribe("/" + radar_position + "_radar", 10, &LRC::vRadarCallback, this);

        }
    }

    // radar infomation process
}
// radar config
void LRC::bounds_callback(LRC_ROS::boundsConfig &config, uint32_t level)
{
    bound_ = config;
}

// lidar infomation from rosbag
std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> LRC::lidarprocess(std::string lidar_position, std::string lrc_path, std::string city)
{
    bool is_extrinsic_calibration = true;
    std::vector<std::string> topic_vec;
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> topic_cloud_map;

    // 根据城市初始化话题向量

    topic_vec = {"/front_left_lidar", "/front_right_lidar", "/back_lidar"};

    // 为每个话题初始化点云指针
    for (const auto &topic : topic_vec)
    {
        topic_cloud_map[topic] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    if (is_extrinsic_calibration)
    {
        std::cout << "Starting extrinsic calibration process..." << std::endl;
        rosbag::Bag bag;
        // std::string bag_path;

        // 选择对应的bag文件

        if (lidar_position == "front")
        {
            bag.open(lrc_path + "/in_out/initial_value/lidar_front.bag");
            // bag.open(mlc_ros1_path + "/in_out/initial_value/2024-01-31-15-59-44.bag");
        }
        else if (lidar_position == "back")
        {
            bag.open(lrc_path + "/in_out/initial_value/lidar_back.bag");
            // bag.open(mlc_ros1_path + "/in_out/initial_value/2024-01-31-15-57-35.bag");
        }

        // 读取bag文件中的点云数据
        for (const rosbag::MessageInstance &m : rosbag::View(bag))
        {
            // 检查当前消息的话题是否在我们的话题列表中 radar_position + "_radar"
            if (std::find(topic_vec.begin(), topic_vec.end(), m.getTopic()) != topic_vec.end())
            {
                sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();

                if (pc_msg != nullptr)
                {
                    // 将ROS消息转换为PCL点云
                    pcl::fromROSMsg(*pc_msg, *topic_cloud_map[m.getTopic()]);

                    // 检查是否所有话题都已获取到点云数据
                    bool all_topics_received = true;
                    for (const auto &topic : topic_vec)
                    {
                        if (topic_cloud_map[topic]->empty())
                        {
                            all_topics_received = false;
                            break;
                        }
                    }
                    // 如果所有话题都收到数据，则退出循环
                    if (all_topics_received)
                    {
                        break;
                    }
                }
            }
        }

        bag.close();
    }

    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_clouds;
    transformed_clouds = transformLidarPointClouds(topic_cloud_map, lrc_path, city);
    return transformed_clouds;
}

std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> LRC::transformLidarPointClouds(
    const std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> &topic_cloud_map,
    const std::string &yaml_path,
    const std::string &city)
{
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_clouds;

    // 为每个话题创建新的转换后的点云
    for (const auto &pair : topic_cloud_map)
    {
        transformed_clouds[pair.first] = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>());
    }

    // 读取不同激光雷达的外参矩阵
    Eigen::MatrixXf front_left_matrix = read_lidar_initial_value(yaml_path, city, "front_left_lidar");
    Eigen::MatrixXf front_right_matrix = read_lidar_initial_value(yaml_path, city, "front_right_lidar");
    Eigen::MatrixXf back_matrix = read_lidar_initial_value(yaml_path, city, "back_lidar");

    // 创建变换矩阵映射
    std::unordered_map<std::string, Eigen::MatrixXf> transform_matrices = {
        {"/front_left_lidar", front_left_matrix},
        {"/front_right_lidar", front_right_matrix},
        {"/back_lidar", back_matrix}};

    // 对每个点云进行变换
    for (const auto &pair : topic_cloud_map)
    {
        const std::string &topic = pair.first;
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud = pair.second;

        if (transform_matrices.find(topic) != transform_matrices.end())
        {
            // 获取对应的变换矩阵
            Eigen::MatrixXf transform_matrix = transform_matrices[topic];

            // 创建仿射变换对象
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.matrix() = transform_matrix.cast<float>();

            // 执行点云变换
            pcl::transformPointCloud(*cloud, *transformed_clouds[topic], transform);

            std::cout << "Transformed point cloud for topic: " << topic
                      << " with points: " << transformed_clouds[topic]->size() << std::endl;
        }
    }

    return transformed_clouds;
}

// radar callback
void LRC::vRadarCallback(const radar_msgs::RadarObjectList &radar_msg)
{
    Eigen::MatrixXf t1(4, 4);

    po_radar_cloud->clear();
    // 按RCS值降序排列雷达点云
    std::vector<radar_msgs::RadarObject> sortedObjects(radar_msg.ObjectList.begin(), radar_msg.ObjectList.end());
    std::sort(sortedObjects.begin(), sortedObjects.end(), [](const radar_msgs::RadarObject &a, const radar_msgs::RadarObject &b)
              { return a.RCS > b.RCS; });

    for (uint i = 0; i < radar_msg.ObjNum; ++i)
    {
        pcl::PointXYZ single_point;
        single_point.x = radar_msg.ObjectList[i].Rel_Pos.x;
        single_point.y = radar_msg.ObjectList[i].Rel_Pos.y;
        single_point.z = 0;

        po_radar_cloud->push_back(single_point);
    }
    std::string position = "/" + radar_position + "_radar";
    t1 = read_lidar_initial_value(lrc_path, city, position);

    pcl::transformPointCloud(*po_radar_cloud, *po_radar_cloud, t1);

    pass_filter_radar(po_radar_cloud, radar_position);


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
    if (position.find("back") != std::string::npos)
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
    else
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
        filter.setFilterLimits(-15, 15);
        filter.filter(*pcd_in_roi);
    }
}

void LRC::pass_filter_radar(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd, const std::string &radar_position)
{
    auto &pcd_in_roi = input_pcd;
    pcl::PassThrough<pcl::PointXYZ> filter;
    if (radar_position.find("back") != std::string::npos)
    {
        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-3, 3);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(-30, 0);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-20, 20);
        filter.filter(*pcd_in_roi);
    }
    else
    {
        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-3, 3);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(0, 30);
        filter.filter(*pcd_in_roi);

        filter.setInputCloud(pcd_in_roi);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-20, 20);
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
Eigen::Vector3f LRC::processChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, std::string position)
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

    // result.planecentroid = planecentroid;

    return planecentroid;
}

Eigen::Matrix4d LRC::calibrateRadarToLidar(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &radar_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &left_lidar_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &right_lidar_cloud,
    const Eigen::Matrix4d &T_right_left,
    const Eigen::Matrix4d &initial_guess)
{

    // 1. 设置优化参数初值
    double transform_params[6] = {
        initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3), // 平移
        0, 0, 0                                                        // 旋转（从旋转矩阵转换为角轴表示）
    };
    Eigen::Matrix3d R_initial = initial_guess.block<3, 3>(0, 0);
    ceres::RotationMatrixToAngleAxis(R_initial.data(), transform_params + 3);

    // 2. 构建优化问题
    ceres::Problem problem;

    // 为每个对应点对添加残差块
    for (size_t i = 0; i < radar_cloud->size(); ++i)
    {
        Eigen::Vector3d radar_point(
            radar_cloud->points[i].x,
            radar_cloud->points[i].y,
            radar_cloud->points[i].z);
        Eigen::Vector3d left_point(
            left_lidar_cloud->points[i].x,
            left_lidar_cloud->points[i].y,
            left_lidar_cloud->points[i].z);
        Eigen::Vector3d right_point(
            right_lidar_cloud->points[i].x,
            right_lidar_cloud->points[i].y,
            right_lidar_cloud->points[i].z);

        ceres::CostFunction *cost_function =
            RadarLidarCalibrationError::Create(radar_point, left_point, right_point, T_right_left);

        problem.AddResidualBlock(
            cost_function,
            new ceres::HuberLoss(0.1), // 使用Huber核函数来处理异常值
            transform_params);
    }

    // 3. 配置求解器
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // 4. 求解优化问题
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // 5. 构建结果变换矩阵
    Eigen::Matrix4d T_radar_left = Eigen::Matrix4d::Identity();

    // 5.1 设置平移部分
    T_radar_left(0, 3) = transform_params[0];
    T_radar_left(1, 3) = transform_params[1];
    T_radar_left(2, 3) = transform_params[2];

    // 5.2 设置旋转部分
    Eigen::Matrix3d R_final;
    ceres::AngleAxisToRotationMatrix(transform_params + 3, R_final.data());
    T_radar_left.block<3, 3>(0, 0) = R_final;

    return T_radar_left;
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