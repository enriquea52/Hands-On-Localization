
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <Eigen/Dense>


void state_visualization(ros::Publisher& vis_pub, int num_scans, Eigen::MatrixXf x_states, Eigen::MatrixXf P_cov, int struct_size = 300, int state_size = 3)
{
  float x, y, theta;
  int index;

  nav_msgs::Odometry robot_pose;
  for(int i = 0; i < num_scans; i ++ )
  {
    index = struct_size - (i*state_size) - state_size;
    x = x_states(index,0); y = x_states(index + 1,0); theta = x_states(index + 2,0);

    robot_pose.header.seq = 0; //i;
    robot_pose.header.frame_id = "odom";
    robot_pose.header.stamp = ros::Time();
    robot_pose.pose.pose.position.x = x;    
    robot_pose.pose.pose.position.y = y;    
    robot_pose.pose.pose.position.z = 0;   

    tf::Quaternion q;

    q.setRPY(0.0,0.0,theta);

    q = q.normalize();
    robot_pose.pose.pose.orientation.x = q.x();
    robot_pose.pose.pose.orientation.y = q.y();
    robot_pose.pose.pose.orientation.z = q.z();
    robot_pose.pose.pose.orientation.w = q.w(); 
    //publish_markers(vis_pub, x, y, theta, i);
    vis_pub.publish( robot_pose );

    robot_pose.pose.covariance[0] = P_cov(3*num_scans - 3*(i + 1), 3*num_scans - 3*(i + 1));
    robot_pose.pose.covariance[7] = P_cov(3*num_scans - 3*(i + 1) + 1, 3*num_scans - 3*(i + 1) + 1);
    robot_pose.pose.covariance[35] = P_cov(3*num_scans - 3*(i + 1) + 2, 3*num_scans - 3*(i + 1) + 2);
  }

}


void current_state_visualization(ros::Publisher& vis_pub, Eigen::MatrixXf current_state)
{
    // FUnction to publish the expected location of the robot
    geometry_msgs::PoseStamped Dead_Reckoning;
    Dead_Reckoning.header.frame_id = "odom";
    Dead_Reckoning.pose.position.x = current_state(0);
    Dead_Reckoning.pose.position.y = current_state(1);
    Dead_Reckoning.pose.position.z = 0;

    tf::Quaternion q;

    q.setRPY(0.0,0.0,current_state(2));

    q = q.normalize();

    Dead_Reckoning.pose.orientation.x = q.x();
    Dead_Reckoning.pose.orientation.y = q.y();
    Dead_Reckoning.pose.orientation.z = q.z();
    Dead_Reckoning.pose.orientation.w = q.w();

    vis_pub.publish(Dead_Reckoning);
}

