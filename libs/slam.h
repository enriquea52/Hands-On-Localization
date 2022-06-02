// Motion model 
Eigen::Vector3f fx(float x, float y, float theta, float v, float w, float dt);
// Motion model's jacobian wrt the state 
Eigen::Matrix3f Jfx(float theta, float v, float dt);

// Motion model's jacobian wrt noise 
Eigen::MatrixXf Jfw(float theta, float dt);

// Prediction step from the Kalman Filter
void prediction(Eigen::MatrixXf& x_hat, Eigen::MatrixXf& P_hat, Eigen::MatrixXf& F1_k, Eigen::MatrixXf& F2_k, float v, float w, float dt,  Eigen::MatrixXf Q, int scans_num);

// Addnewpose method from the PEKSLAM
void addnewpose(Eigen::MatrixXf& fx_state, Eigen::MatrixXf& x_hat, Eigen::MatrixXf& P_hat, float v, float w, float dt,  Eigen::MatrixXf Q, int scans_num);

// Megthod to find overalpping scans based on the distance form the viewpoint
int OverlappingScans( int *overlapped_scans, Eigen::MatrixXf& x_latest, Eigen::MatrixXf& x_hat, int scans_num, float overlap_threshold = 2);

// Observation equation hx
Eigen::MatrixXf h_x(float x_k, float y_k, float theta_k, float x_x, float y_x, float theta_x);

// Jacobian of the observation equation wrt the recently added state
Eigen::MatrixXf Jh_x_k(Eigen::Vector3f X_k, Eigen::Vector3f X_x);

// Jacobian of the observation equation wrt the viewpoint correspondance
Eigen::MatrixXf Jh_x_x(Eigen::Vector3f X_k, Eigen::Vector3f X_x);

// Build obervation matrix for a given set of correspondances
void ObservationMatrix(Eigen::MatrixXf& x_hat, int current_index, int *overlapped_scans, int overlap_number, int scans_num, Eigen::MatrixXf& H_k, Eigen::MatrixXf& V_k);

// Update step from the Kalman Filter
void update(int current_index, int scan_num, Eigen::MatrixXf& x_hat, Eigen::MatrixXf& P_hat, Eigen::MatrixXf& z_k, Eigen::MatrixXf& h_k, Eigen::MatrixXf& H_k, Eigen::MatrixXf& V_k, Eigen::MatrixXf& R_k);
