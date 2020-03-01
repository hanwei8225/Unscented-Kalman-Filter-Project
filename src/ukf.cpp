#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // Open NIS data files
  NIS_radar_.open( "../NIS/NIS_radar.txt", std::ios::out );
  NIS_laser_.open( "../NIS/NIS_laser.txt", std::ios::out );

  // Check for errors opening the files
  if( !NIS_radar_.is_open() )
  {
    std::cout << "Error opening NIS_radar.txt" << std::endl;
    exit(1);
  }

  if( !NIS_laser_.is_open() )
  {
    std::cout << "Error opening NIS_laser.txt" << std::endl;
    exit(1);
  }


  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  n_x_ = 5;

  lambda_ = 3 * n_x_;

  n_aug_ = 7;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  {
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));
  }

  time_us_ = 0.0;
  is_initialized_ = false;




}

UKF::~UKF() {
  NIS_radar_.close();
  NIS_laser_.close();
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
 
    if (!is_initialized_)
    {
      time_us_ = meas_package.timestamp_;
      x_ << 1, 1, 1, 1, 0.1;
      P_ << 0.15, 0, 0, 0, 0,
          0, 0.15, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
      if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
      {
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
      {
        double ro = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        double ro_dot = meas_package.raw_measurements_(2);
        double vx = ro_dot * cos(phi);
  	    double vy = ro_dot * sin(phi);
        double v = sqrt(vx * vx + vy * vy);
        x_(0) = ro * cos(phi);
        x_(1) = ro * sin(phi);
        x_(3) = v;
      }

      is_initialized_ = true;
      std::cout << "is_initialized " << is_initialized_ << std::endl;
      return;
    }
    else
    {
      double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
      time_us_ = meas_package.timestamp_;
      if( delta_t > 0.0001 ) Prediction(delta_t);
      std::cout << "Predicted x_: " << x_ << std::endl;
      std::cout << "Predicted P_: " << P_ << std::endl;
      if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
      {
        UpdateLidar(meas_package);
        std::cout << "updated_L x_: " << x_ << std::endl;
        std::cout << "updated_L P_: " << P_ << std::endl;
      }
      else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
      {
        UpdateRadar(meas_package);
        std::cout << "updated_R x_: " << x_ << std::endl;
        std::cout << "updated_R P_: " << P_ << std::endl;
      }
    }
  }


void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */



  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);


  //augmentation use noise,使用噪声Q组成增广矩阵
  MatrixXd Q(2, 2);
  Q << std_a_ * std_a_, 0,
      0, std_yawdd_ * std_yawdd_;
  // P_aug = | p_,0 |
  //         | 0, Q |

  P_aug.fill(0.);  //important
  P_aug.topLeftCorner(n_x_, n_x_) = P_;

  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

  //calculate augmentation sigma point ，计算增广矩阵下的sigma point
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); //7 X 15
  Xsig_aug.col(0) = x_aug;

  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  MatrixXd m_aug = sqrt(lambda_ + n_aug_) * A_aug;

  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + m_aug.col(i);
    Xsig_aug.col(n_aug_ + i + 1) = x_aug - m_aug.col(i);
  }

  //sigma point prediction,Run each augmented sigma points through the process model with noise
  //predict sigma points

  VectorXd a = VectorXd(n_x_);
  VectorXd u = VectorXd(n_x_);
  VectorXd xk = VectorXd(n_x_);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_dot = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    xk = Xsig_aug.col(i).segment(0, n_x_); // 7 => 5
    u[0] = 0.5 * (delta_t * delta_t) * cos(yaw) * nu_a;
    u[1] = 0.5 * (delta_t * delta_t) * sin(yaw) * nu_a;
    u[2] = delta_t * nu_a;
    u[3] = 0.5 * (delta_t * delta_t) * nu_yawdd;
    u[4] = delta_t * nu_yawdd;

    //avoid division by zero
    if (fabs(yaw_dot) > 0.001)
    {
      a(0) = (v / yaw_dot) * (sin(yaw + yaw_dot * delta_t) - sin(yaw));
      a[1] = (v / yaw_dot) * (cos(yaw) - cos(yaw + yaw_dot * delta_t));
      a[2] = 0;
      a[3] = yaw_dot * delta_t;
      a[4] = 0;
    }
    else
    {
      a[0] = v * cos(yaw) * delta_t;
      a[1] = v * sin(yaw) * delta_t;
      a[2] = 0;
      a[3] = yaw_dot * delta_t;
      a[4] = 0;
    }
    //write predicted sigma points into right column
    Xsig_pred_.col(i) = xk + a + u;
  }

  //Predicted_Mean_and_Covariance 预测新的x_和P_｝, Convert Predicted Sigma Points to Mean/Covariance


  //mean
   x_.fill(0.);   // important 
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  //Covariance
  P_.fill(0.); //important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    /*
      When calculating the predicted state covariance matrix,
      in the equation we always need the difference between
      the mean predicted state and a sigma points. The problem
      here is that the state contains an angle.  Subtracting angles 
      is a problem for Kalman filters, because the result might be 
      2π plus a small angle,instead of just a small angle. That’s 
      why I normalize the angle here.

    */
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  //set measurement dimension,px,py
  int n_z = 2;
  // measurement covariance noise matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  //calculate Zsig
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // VectorXd x = Xsig_pred_.col(i);
    // double px = x(0);
    // double py = x(1);

    // Zsig.col(i)(0) = px;
    // Zsig.col(i)(1) = py;
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  //calculate mean predicted measurement
  z_pred.fill(0.); // important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  S.fill(0.); // important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * (z_diff) * (z_diff).transpose();
  }
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);  // important
  // T
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd t_x_diff = (Xsig_pred_.col(i) - x_);
    VectorXd z_diff = Zsig.col(i) - z_pred;
    Tc += weights_(i) * t_x_diff * ((z_diff).transpose());
  }
  MatrixXd K = Tc * (S.inverse());
  //measurement of lidar data
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),
      meas_package.raw_measurements_(1);

  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  NIS_laser = z_diff.transpose()*S.inverse()*z_diff;
  NIS_laser_ << NIS_laser << std::endl;;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // measurement covariance noise matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  //calculate Zsig
  Zsig.fill(0.);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yawd = Xsig_pred_(3,i);

    double ro = sqrt(px * px + py * py);
    double theta = atan2(py, px);
    double ro_dot = (px * cos(yawd) * v + py * (sin(yawd) * v)) / ro;


    Zsig(0,i) = ro;
    Zsig(1,i) = theta;
    Zsig(2,i) = ro_dot;
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }
  
  std::cout << "z_pred R:" << z_pred << std::endl;
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    S += weights_(i) * (z_diff) * (z_diff).transpose();
  }
  S = S + R;
  // std::cout << "S R:" << S << std::endl;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // T
  Tc.fill(0.); //important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd t_x_diff = (Xsig_pred_.col(i) - x_);
    while (t_x_diff(3) > M_PI)
      t_x_diff(3) -= 2. * M_PI;
    while (t_x_diff(3) < -M_PI)
      t_x_diff(3) += 2. * M_PI;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    Tc += weights_(i) * t_x_diff * ((z_diff).transpose());
  }
  // std::cout << "Tc R:" << Tc << std::endl;
  MatrixXd K = Tc * (S.inverse());
  // std::cout << "K R:" << K << std::endl;

  //measurement of radar data
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),
      meas_package.raw_measurements_(1),
      meas_package.raw_measurements_(2);
  // std::cout << "Z measurement:" << z << std::endl;

  VectorXd z_diff = z - z_pred;

  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

    // Uncomment the following to print normalized innovation squared (NIS_radar_),
  // so that it can be plotted and serve as a consistency check on 
  // our choice of process noise values
  NIS_radar = z_diff.transpose()*S.inverse()*z_diff;
  NIS_radar_ << NIS_radar << std::endl;;
}
