
// ICP algorithm for getting the transformation between 2 pointclouds
Eigen::Matrix3f pair_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Vector3f estimation, Eigen::Matrix4f& z_k_mat, float maxCorrespondenceDistance);

// Method to transform a pointcloud using a transformation matrix
void pc_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Vector3f robot_pose);

// wrap angle function to keep angles between pi and -pi
float wrap_angle(float angle);