#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>

#include "../libs/icp_odom.h"
#include "../libs/slam.h"
#include "../libs/vis_utils.h"

#include <vector>

#include <Eigen/Dense>

#include <tf/tf.h>

#define max_states 100
#define state_size 3
#define struct_size (max_states*state_size)

Eigen::MatrixXf x_states = Eigen::MatrixXf::Zero(struct_size, 1);
Eigen::MatrixXf P_cov = Eigen::MatrixXf::Zero(struct_size, struct_size);
Eigen::MatrixXf F1_k = Eigen::MatrixXf::Zero(struct_size, struct_size);
Eigen::MatrixXf F2_k = Eigen::MatrixXf::Zero(struct_size, 2);
Eigen::Matrix2f Qk;
Eigen::Matrix3f Rk;

Eigen::MatrixXf current_state = Eigen::MatrixXf::Zero(3,1);


std::vector<pcl::PointCloud<pcl::PointXYZ>> map;


class slam
{
public:
  slam(std::string odom_topic, double max_correspondance, bool rotate_laser_scan_flag)
  {

    // periodic timer for visualizing the estimated position of the robot
    timer_ = n_.createTimer(ros::Duration(1.0 / 2),&slam::robot_estimation, this);

    // lidar subscriber
    lidar_sub_ = n_.subscribe("/laserscan_to_pointcloud/converted_pc", 1, &slam::StoreScan, this);

    // Subscriber to the /odom topic to know the velocity of the robot (linear and angular)
    odom_sub_ = n_.subscribe(odom_topic, 1, &slam::localization, this);

    // publish a pose stamped about the estimation of the robot's pose
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/slam_odom", 1);

    // publish the poses that define the trayectory
    pub_trayectory = n_.advertise<nav_msgs::Odometry>("/pekslam_trayectory", 1);

    // publish the map composed of the stored scans
    pub_map = n_.advertise<sensor_msgs::PointCloud2>("/scanned_map", 1);

    // set the max correspondace parameter for the icp algorithm
    icp_max_correspondance = max_correspondance;

    // rotate scan for physical robot, do not rotate for simulated environment
    this->rotate_laser_scan = rotate_laser_scan_flag;

  }

  void localization(const nav_msgs::Odometry& robot)
  {
    // receiving the current pose of the robot from the odometry message////
     float v = robot.twist.twist.linear.x;      
     float w = robot.twist.twist.angular.z;   
     double dt = ros::Time::now().toSec() - last_time;
     last_time = ros::Time::now().toSec();
    /////////////////////////////////////////////////////////////////////////

    if (is_init_slam)
    {
      // SLAM initialization

      if (!stored_cloud.empty())
      {
        is_init_slam = false;                           
        map.push_back(stored_cloud);                                             // Update the map representation by adding a new pointcloud to it
        P_cov.block<3,3>(0,0)  << 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0;
        scans_num++;                                                             // increase the number of scans by 1
        current_index = struct_size - (state_size*scans_num) - state_size;       // Update the current index for accessing the current state of the robot

      }

    }
    else
    {

      if ((current_state.block<2,1>(0,0) - x_states.block<2,1>(current_index + 3,0)).norm() > 0.5)
      {
        should_add_new_scan = true;
      }
      else
      {
        should_add_new_scan = false;
      }

      if (should_add_new_scan) // Equivalent to ScanAvailable in Line 35
      {
        std::cout << "Number of scans: " << scans_num << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr S_k (new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::MatrixXf x_k(3, 1);


        addnewpose(current_state, x_states, P_cov, v, w, dt, Qk, scans_num);
        map.push_back(stored_cloud);

        current_scan_index = map.size() - 1; // Index of the recenyl added scan to the map

        x_k = x_states.block<3,1>(current_index,0);

        *S_k = map.at(current_scan_index);

        number_of_overlaps = OverlappingScans(overlaps, x_k, x_states, scans_num, 1.5);

        Eigen::MatrixXf h_stack(3*number_of_overlaps, 1);

        Eigen::MatrixXf z_stack(3*number_of_overlaps, 1);

        Eigen::MatrixXf H_k = Eigen::MatrixXf::Zero(number_of_overlaps*3, 3 + scans_num*3);
        Eigen::MatrixXf V_k = Eigen::MatrixXf::Zero(number_of_overlaps*3, number_of_overlaps*3);
        Eigen::MatrixXf R_k = Eigen::MatrixXf::Zero(number_of_overlaps*3, number_of_overlaps*3);


        // z vector and h(x, v) wrt overlapping scans
        for(int o = 0; o < number_of_overlaps; o++)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr S_x (new pcl::PointCloud<pcl::PointXYZ>);
          Eigen::MatrixXf x_x(3, 1);
          Eigen::MatrixXf z_k(3, 1);
          Eigen::Matrix4f z_k_mat;
          Eigen::Vector3f h_k;

          scan_index = (300 - overlaps[o] - 3)/3; 
          
          *S_x = map.at(scan_index);

          x_x = x_states.block<3,1>(overlaps[o],0);

          h_k = h_x(x_k(0,0), x_k(1,0), x_k(2,0), x_x(0,0), x_x(1,0), x_x(2,0));

          h_k(2) = wrap_angle(h_k(2));

          Rk = pair_alignment(S_x, S_k, h_k, z_k_mat, icp_max_correspondance);

          z_k << z_k_mat(0, 3),
                 z_k_mat(1, 3),
                 wrap_angle(std::atan2(z_k_mat(1, 0), z_k_mat(0, 0)));

          h_stack.block<3,1>(3*o, 0) = h_k;
          z_stack.block<3,1>(3*o, 0) = z_k;

          R_k.block(3*o,3*o,3,3) = Rk;
         
        }

        // Fill H, V and R matrices according to corresponding scans
        ObservationMatrix(x_states, current_index, overlaps, number_of_overlaps, scans_num, H_k, V_k);

        //Apply kalman Filter Update
        update(current_index, scans_num, x_states, P_cov, z_stack, h_stack, H_k,  V_k,  R_k);

        current_state = x_states.block(current_index,0,3, 1);

        scans_num++;
        current_index = struct_size - (state_size*scans_num) - state_size;      


      }


      }


    current_state = fx(current_state(0), current_state(1), current_state(2), v, w,  dt);

    
  }

  void StoreScan(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    // Store each received scan into memory, to use the most recent one when required.
    pcl::PointCloud<pcl::PointXYZ>::Ptr update_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud, *update_cloud_ptr);

    if (rotate_laser_scan)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

      Eigen::Vector3f pi_rotation;
    
      pi_rotation << 0, 0, 3.1416;

      pc_transform(update_cloud_ptr, transformed_cloud_ptr, pi_rotation);

      stored_cloud = *transformed_cloud_ptr;

    }
    else
    {
      stored_cloud = *update_cloud_ptr;
    }


  }


  void robot_estimation(const ros::TimerEvent&)
  {
    // Function for calling visualization methods of the current state, the state vector and the map

    current_state_visualization(pub_, current_state);

    state_visualization(pub_trayectory, scans_num, x_states, P_cov);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_plot(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < map.size(); i++)
    {
      *cloud_in = map.at(i);

      pc_transform(cloud_in, cloud_out, x_states.block<3,1>(300-(3*i)-3,0));

      *map_plot += *cloud_out;
       
    }

    sensor_msgs::PointCloud2 map_segment;

    pcl::toROSMsg(*map_plot, map_segment);

    map_segment.header.frame_id = "odom";

    map_segment.header.seq = 0;

    pub_map.publish(map_segment);

  }

private:

  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub_trayectory;
  ros::Publisher pub_map;
  ros::Subscriber lidar_sub_;
  ros::Subscriber odom_sub_;
  ros::Timer timer_;

  //cloud storage
  pcl::PointCloud<pcl::PointXYZ> stored_cloud;
  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();

  // SLAM stuff
  bool is_init_slam = true;
  int scans_num = 0;
  int current_index = 0;
  bool should_add_new_scan = false;
  bool should_update = false;

  double last_time = ros::Time::now().toSec();
  int overlaps[max_states];
  int number_of_overlaps = 0;
  int scan_index = 0;
  int current_scan_index = 0;
  int current_pose_index = 0;

  float icp_max_correspondance = 0;

  //rotate laserscan for simualtion or real life implementation
  bool rotate_laser_scan;


}; //End of class SubscribeAndPublish


int main(int argc, char **argv)
{


  //Covariance matrix initialization
  Eigen::MatrixXf I;
  I.setIdentity(state_size, state_size);

  for (int i = 1; i < max_states; i++)
  {

    F1_k.block<state_size,state_size>(i*state_size,i*state_size)  = I;

  }

  std::cout << "PEKSLAM NODE IGNITED" << std::endl;

  //Initiate ROS

  ros::init(argc, argv, "PEKSLAM_node");


  //variables to store loaded parameters
  std::string odom_topic;
  double Qk_v, Qk_w, Rk_x, Rk_y, Rk_theta, max_correspondance;
  bool rotate_laser_scan_flag;

  // Read parameters from the parameter server
  ros::param::get("/tbot_odometry_topic", odom_topic);
  ros::param::get("/Qk_v", Qk_v);
  ros::param::get("/Qk_w", Qk_w);
  ros::param::get("/Rk_x", Rk_x);
  ros::param::get("/Rk_y", Rk_y);
  ros::param::get("/Rk_theta", Rk_theta);
  ros::param::get("/icp_max_correspondance", max_correspondance);
  ros::param::get("/rotate_laser_scan", rotate_laser_scan_flag);

  // Initialization 
  Qk << Qk_v, 0,
        0, Qk_w;

  Rk << Rk_x, 0,    0,
        0,    Rk_y, 0,
        0,    0,    Rk_theta;

  slam node(odom_topic, max_correspondance, rotate_laser_scan_flag);

  ros::spin();

  return 0;
}