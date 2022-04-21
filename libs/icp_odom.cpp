#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>


Eigen::Matrix4f pair_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    
    gicp.setInputSource(cloud_in);
    gicp.setInputTarget(cloud_out);
    gicp.setMaximumOptimizerIterations(50);

    pcl::PointCloud<pcl::PointXYZ> Final;

    gicp.align(Final);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); // Transformation computed between scans

    transform = gicp.getFinalTransformation();

    return transform;
}
