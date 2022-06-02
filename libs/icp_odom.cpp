#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>

float wrap_angle(float angle){return (angle + (2.0*M_PI*std::floor((M_PI - angle)/(2.0*M_PI))));}


Eigen::Matrix3f pair_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Vector3f estimation, Eigen::Matrix4f& z_k_mat, float maxCorrespondenceDistance)
{

    float yaw = estimation(2);

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

    init_guess << std::cos(yaw), -std::sin(yaw), 0, estimation(0), 
                  std::sin(yaw),  std::cos(yaw), 0, estimation(1),
                  0, 0, 1, 0,
                  0, 0, 0, 1;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);

    icp.setMaximumIterations (1000);

    icp.setTransformationEpsilon (1e-9);

    pcl::PointCloud<pcl::PointXYZ> Final;

    icp.align(Final, init_guess);

    Eigen::Matrix3f Rk = Eigen::Matrix3f::Identity(); // Transformation computed between scans

    z_k_mat = icp.getFinalTransformation();

    if (icp.getFitnessScore() < 0.05)
    {

        Rk << 0.001, 0,    0,
              0,    0.001, 0,
              0,    0,    0.01;

        return Rk;
    }

    else if (icp.getFitnessScore() < 0.001)
    {
        Rk << icp.getFitnessScore(), 0,    0,
              0,    icp.getFitnessScore(), 0,
              0,    0,    icp.getFitnessScore();

    return Rk;
    }
    else
    {

        Rk << 15, 0,  0,
              0,  15, 0,
              0,  0,  25;

        return Rk;

    }



}


void pc_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Vector3f robot_pose)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); // Transformation computed between scans
    
    float yaw = robot_pose(2);

    transform << std::cos(yaw), -std::sin(yaw), 0, robot_pose(0), 
                  std::sin(yaw),  std::cos(yaw), 0, robot_pose(1),
                  0, 0, 1, 0,
                  0, 0, 0, 1;

    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);

}
