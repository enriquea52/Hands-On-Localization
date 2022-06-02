// state visualization function to visualize the poses stored in the state vector
void state_visualization(ros::Publisher& vis_pub, int num_scans, Eigen::MatrixXf x_states, Eigen::MatrixXf P_cov, int struct_size = 300, int state_size = 3);
// Function to visualize the current position of the robot
void current_state_visualization(ros::Publisher& vis_pub, Eigen::MatrixXf current_state);
