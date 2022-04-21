#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>



#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>

#include <nav_msgs/Odometry.h>

#include "../libs/icp_odom.h"






class cloud_odom
{
public:
  cloud_odom()
  {
    std::cout << "ostias chaval" << std::endl;

    // periodic timer
    timer_ = n_.createTimer(ros::Duration(1.0 / 2.0),&cloud_odom::get_transform, this);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/turtlebot/rplidar/scan/converted_pc", 1, &cloud_odom::callback, this);

    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::Odometry>("/icp_odom", 1);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr update_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *update_cloud_ptr);

    temp = *update_cloud_ptr;

  }


  void get_transform(const ros::TimerEvent&)
  {
      if (is_init)
      {
            std::cout << "Storing cloud for the first time" << std::endl;
            current_cloud = temp; //Updating current point cloud for the first time
            is_init = false;

      }
      else
      {
          pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointCloud<pcl::PointXYZ>::Ptr past_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);


          std::cout << "Storing old and new cloud" << std::endl;
          past_cloud = current_cloud;
          current_cloud = temp;


          *past_cloud_ptr = past_cloud;
          *current_cloud_ptr = current_cloud;

          odom *=pair_alignment(current_cloud_ptr, past_cloud_ptr); 
          //odom *=pair_alignment(past_cloud_ptr, current_cloud_ptr); //working stuff (kinda)
          //std::cout << "Transformation between point clouds" << std::endl;
          //std::cout << odom << std::endl;
          //std::cout << pair_alignment(cloud_in, cloud_out) << std::endl;



        Eigen::Matrix3f mat; //rotation matrix
        Eigen::Vector3f trans; //translation vector
        
        trans << odom(0,3), odom(1,3), odom(2,3);
        mat << odom(0,0), odom(0,1), odom(0,2),
               odom(1,0), odom(1,1), odom(1,2),
               odom(2,0), odom(2,1), odom(2,2);
        
        std::cout << "Translation\n" << trans << std::endl;

        std::cout << "Rotation\n" << mat << std::endl;


        Eigen::Quaternionf quat(mat); //rotation matrix stored as a quaternion

        nav_msgs::Odometry curr_pose;
        curr_pose.header.stamp = ros::Time::now();
        curr_pose.header.frame_id = "/odom";
        curr_pose.pose.pose.position.x = trans[0];
        curr_pose.pose.pose.position.y = trans[1];        
        curr_pose.pose.pose.position.z = trans[2];        
        curr_pose.pose.pose.orientation.x = quat.x();
        curr_pose.pose.pose.orientation.y = quat.y();        
        curr_pose.pose.pose.orientation.z = quat.z();        
        curr_pose.pose.pose.orientation.w = quat.w();

        pub_.publish(curr_pose); //publishing the current pose


        
      }
    


    //std::cout << "Getting transform height: " << current_cloud.height << std::endl;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;
  //cloud storage
  pcl::PointCloud<pcl::PointXYZ> temp;
  pcl::PointCloud<pcl::PointXYZ> current_cloud;
  pcl::PointCloud<pcl::PointXYZ> past_cloud;
  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();

  bool is_init = true;


};//End of class SubscribeAndPublish





int main(int argc, char **argv)
{

    std::cout << "this is the last time to rock" << std::endl;

  //Initiate ROS
  ros::init(argc, argv, "Visual_Odometry_Node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  cloud_odom node;

  ros::spin();

  return 0;
}