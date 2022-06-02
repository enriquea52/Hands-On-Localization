#include <Eigen/Dense>
#include <iostream>

#include <random>

Eigen::Vector3f fx(float x, float y, float theta, float v, float w, float dt)
{


    std::random_device rd{};
    std::mt19937 gen{rd()};
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean

    std::normal_distribution<> W_v{0,0.05};

    std::normal_distribution<> W_w{0,0.04};


    float v_noise = 0; //W_v(gen);

    float w_noise = 0; //W_w(gen);


    Eigen::Vector3f motion_model;
    
    motion_model << (std::cos(theta)*(v + v_noise)*dt) + x,
                    (std::sin(theta)*(v + v_noise)*dt) + y,
                    theta + (w + w_noise)*dt;



    return motion_model;
}

Eigen::Matrix3f Jfx(float theta, float v, float dt)
{
    Eigen::Matrix3f motion_jacobian; 
    motion_jacobian << 1, 0, -(std::sin(theta)*v*dt),
                       0, 1,  (std::cos(theta)*v*dt),
                       0, 0, 1;
    return motion_jacobian;
}

Eigen::MatrixXf Jfw(float theta, float dt)
{
    Eigen::MatrixXf motion_noise_jacobian(3, 2);
    motion_noise_jacobian << std::cos(theta)*dt, 0,
                             std::sin(theta)*dt, 0, 
                             0, dt;

    return motion_noise_jacobian;
}


void prediction(Eigen::MatrixXf& x_hat, Eigen::MatrixXf& P_hat, Eigen::MatrixXf& F1_k, Eigen::MatrixXf& F2_k, float v, float w, float dt,  Eigen::MatrixXf Q, int scans_num)
{
    int state_index_range = 300 - (3*scans_num) - 3;
    // prediction of state's mean
    x_hat.block<3,1>(state_index_range,0) = fx(x_hat(state_index_range,0), x_hat(state_index_range+1,0), x_hat(state_index_range+2,0), v, w, dt);
    
    // // F1_k and F2_k evaluation
    F1_k.block<3,3>(0,0) = Jfx(x_hat(state_index_range+2,0), v, dt);
    F2_k.block<3,2>(0,0) = Jfw(x_hat(state_index_range+2,0), dt);

    int P_index_range = (3*scans_num) + 3;

    Eigen::MatrixXf F1_temp = F1_k.block(0,0,P_index_range,P_index_range);

    Eigen::MatrixXf F1_temp_T = F1_temp.transpose();

    Eigen::MatrixXf F2_temp = F2_k.block(0,0,P_index_range,2);

    Eigen::MatrixXf F2_temp_T = F2_temp.transpose();

    // // prediction of state's covariance
    P_hat.block(0,0,P_index_range,P_index_range) = F1_temp*P_hat.block(0,0,P_index_range,P_index_range)*F1_temp_T + F2_temp*Q*F2_temp_T;
}

void addnewpose(Eigen::MatrixXf& fx_state, Eigen::MatrixXf& x_hat, Eigen::MatrixXf& P_hat, float v, float w, float dt,  Eigen::MatrixXf Q, int scans_num)
{
    std::cout << "Addng a new pose..." << std::endl;
    // Adding new pose
    int state_index_range = 300 - (3*scans_num) - 3;
    

    x_hat.block(state_index_range, 0, 3, 1) = fx_state;


    // // F1_k and F2_k evaluation
    Eigen::MatrixXf F1_k = Eigen::MatrixXf::Zero(3 + 3*scans_num, 3 + 3*(scans_num-1));

    Eigen::MatrixXf F2_k = Eigen::MatrixXf::Zero(3 + 3*scans_num, 2);


    F1_k.block<3,3>(0,0) = Jfx(x_hat(state_index_range+2,0), v, dt);

    F2_k.block<3,2>(0,0) = Jfw(x_hat(state_index_range+2,0), dt);

    Eigen::MatrixXf I;
    I.setIdentity(3, 3);


    for (int i = 1; i < scans_num + 1; i++)
    {
        F1_k.block<3,3>(3*i,3*i - 3)  = I;
    }


    int P_index_range = (3*scans_num) + 3;

    int P_index_range_past = (3*(scans_num-1)) + 3;

    // prediction of state's covariance
    P_hat.block(0,0,P_index_range,P_index_range) = F1_k*P_hat.block(0,0,P_index_range_past,P_index_range_past)*F1_k.transpose() + F2_k*Q*F2_k.transpose();


}


int OverlappingScans( int *overlapped_scans, Eigen::MatrixXf& x_latest, Eigen::MatrixXf& x_hat, int scans_num, float overlap_threshold = 2)
{
    int latest_scan = 300 - (3*(scans_num)) - 3;
    int index = 0;
    int number_of_overlaps = 0;

    for(int i = 0; i < scans_num; i++ )
    {
        index = 300 - (3*i) - 3;

        if ((x_latest.block<2,1>(0,0) - x_hat.block<2,1>(index,0)).norm() < overlap_threshold)
        {

            overlapped_scans[number_of_overlaps] = index;
            number_of_overlaps++;

        }
    }
    return number_of_overlaps;
}


Eigen::MatrixXf h_x(float x_k, float y_k, float theta_k, float x_x, float y_x, float theta_x)
{
    Eigen::MatrixXf h = Eigen::MatrixXf::Zero(3, 1);

    h << x_x*std::cos(-theta_k)-y_x*std::sin(-theta_k)-x_k*std::cos(theta_k)-y_k*std::sin(theta_k),
         x_x*std::sin(-theta_k)+y_x*std::cos(-theta_k)+x_k*std::sin(theta_k)-y_k*std::cos(theta_k),
         -theta_k + theta_x;

    return h;

}

Eigen::MatrixXf Jh_x_k(Eigen::Vector3f X_k, Eigen::Vector3f X_x)
{

    Eigen::MatrixXf Jh(3, 3);

    float x_k, y_k, theta_k, x_x, y_x, theta_x;

    x_k = X_k(0); x_x = X_x(0);

    y_k = X_k(1); y_x = X_x(1);

    theta_k = X_k(2); theta_x = X_x(2);

    Jh << -std::cos(theta_k), -std::sin(theta_k), -x_x*std::sin(theta_k) + y_x*std::cos(theta_k) + x_k*std::sin(theta_k) - y_k*std::cos(theta_k),
           std::sin(theta_k), -std::cos(theta_k), -x_x*std::cos(theta_k) - y_x*std::sin(theta_k) + x_k*std::cos(theta_k) + y_k*std::sin(theta_k),
           0, 0, -1;

    return Jh;

}

Eigen::MatrixXf Jh_x_x(Eigen::Vector3f X_k, Eigen::Vector3f X_x)
{

    Eigen::MatrixXf Jh(3, 3);

    float x_k, y_k, theta_k, x_x, y_x, theta_x;

    x_k = X_k(0); x_x = X_x(0);

    y_k = X_k(1); y_x = X_x(1);

    theta_k = X_k(2); theta_x = X_x(2);

    Jh <<  std::cos(theta_k), std::sin(theta_k), 0,
          -std::sin(theta_k), std::cos(theta_k), 0,
           0, 0, 1;

    return Jh;

}


void ObservationMatrix(Eigen::MatrixXf& x_hat, int current_index, int *overlapped_scans, int overlap_number, int scans_num, Eigen::MatrixXf& H_k, Eigen::MatrixXf& V_k)
{

  Eigen::MatrixXf I;
  I.setIdentity(3, 3);
  int scan_index;
  int temp_index = 0;


    for(int o = 0; o < overlap_number; o++)
    {
        
        scan_index = (300 - overlapped_scans[o] - 3)/3; 

        H_k.block(3*o,0,3,3) = Jh_x_k(x_hat.block(current_index,0,3,1), x_hat.block(overlapped_scans[o],0,3,1));

        temp_index = H_k.cols() - 3 - 3*scan_index;

        H_k.block(3*o, temp_index, 3, 3) = Jh_x_x(x_hat.block(current_index,0,3,1), x_hat.block(overlapped_scans[o],0,3,1));

        V_k.block(3*o,3*o,3,3) = I;
        
    }

    
}

void update(int current_index, int scan_num, Eigen::MatrixXf& x_hat, Eigen::MatrixXf& P_hat, Eigen::MatrixXf& z_k, Eigen::MatrixXf& h_k, Eigen::MatrixXf& H_k, Eigen::MatrixXf& V_k, Eigen::MatrixXf& R_k)
{
  Eigen::MatrixXf I;
  I.setIdentity((3*scan_num) + 3, (3*scan_num) + 3);


  Eigen::MatrixXf P;

  P = P_hat.block(0,0,(3*scan_num) + 3,(3*scan_num) + 3) ;

  Eigen::MatrixXf K = P*H_k.transpose()*( H_k*P*H_k.transpose() + V_k*R_k*V_k.transpose()).inverse();

  x_hat.block(current_index,0,3+scan_num*3,1) = x_hat.block(current_index,0,3+scan_num*3,1) +  K*(z_k - h_k);

  P_hat.block(0,0,(3*scan_num) + 3,(3*scan_num) + 3) = (I - K*H_k)*P;

}