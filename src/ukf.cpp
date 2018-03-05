#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.8;//6;//30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;//30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //define spreading parameter
  lambda_ = 3 - n_x_;

  //create sigma point prediction matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create vector for weights_
  weights_ = VectorXd(2 * n_aug_ + 1);

  //set initialization variable
  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
	if (!is_initialized_) {
		/**
		TODO:
		* Initialize the state with the first measurement.
		* Create the covariance matrix.
		* Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		cout << "UKF: " << endl;
		x_ << 1, 1, 0.1, 0.1, 0.1;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			x_(0) = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
			x_(1) = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			x_(0) = meas_package.raw_measurements_[0];
			x_(1) = meas_package.raw_measurements_[1];			
		}
		// Initialize P_ matrix
		P_ <<      1, 0, 0, 0, 0,
				   0, 1, 0, 0, 0,
				   0, 0, 1, 0, 0,
				   0, 0, 0, 1, 0,
				   0, 0, 0, 0, 1;

		//timestamp
		time_us_ = meas_package.timestamp_;
		// done initializing, no need to predict or update
		is_initialized_ = true;
		cout << "Initialized" << endl;
		return;
	}
	//cout << "Processing" << endl;
	//cout << "States: " << endl << x_ << endl;
	//cout << "Covariances: " << endl << P_ << endl;
	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
	time_us_ = meas_package.timestamp_;
	//cout << "Delta_t: " << dt << endl;
	Prediction(dt);
	/*****************************************************************************
	*  Update
	****************************************************************************/
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		UpdateLidar(meas_package);
	}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //cout << "Predicting" << endl;
  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug_.fill(0.0);
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_(5, 5) = std_a_ * std_a_;
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;

  for (int i = 0; i < n_aug_; i++)
  {
	  Xsig_aug_.col(i + 1) = x_aug_ + (sqrt(lambda_ + n_aug_) * A.col(i));
	  Xsig_aug_.col(n_aug_ + i + 1) = x_aug_ - (sqrt(lambda_ + n_aug_) * A.col(i));
  }

  //predict sigma points
  Xsig_pred_.fill(0);
	for (int i = 0; i< 2 * n_aug_ + 1; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug_(0, i);
		double p_y = Xsig_aug_(1, i);
		double v = Xsig_aug_(2, i);
		double yaw = Xsig_aug_(3, i);
		double yawd = Xsig_aug_(4, i);
		double nu_a = Xsig_aug_(5, i);
		double nu_yawdd = Xsig_aug_(6, i);

		//normalize angles
		/*cout << "Iter: " << i << endl;
		cout << "Delta t: " << delta_t << endl;
		cout << "Yaw: " << yaw << endl;*/
		//while (yaw > M_PI) yaw -= 2.*M_PI;
		//while (yaw <-M_PI) yaw += 2.*M_PI;
		//cout << "Normalizing all done 1.." << endl;

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
		}
		else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		//normalize angles
		//cout << "Yaw: " << yaw_p << endl;
		//while (yaw_p > M_PI) yaw_p -= 2.*M_PI;
		//while (yaw_p <-M_PI) yaw_p += 2.*M_PI;
		//cout << "Normalizing all done 2.." << endl;

		//add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		//normalize angles
		//cout << "Yaw: " << yaw_p << endl;
		//while (yaw_p > M_PI) yaw_p -= 2.*M_PI;
		//while (yaw_p <-M_PI) yaw_p += 2.*M_PI;
		//cout << "Normalizing all done 3.." << endl;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}
  //set weights_
  weights_(0) = lambda_/(lambda_+n_aug_);
  
  for(int i=1; i<2*n_aug_+1; i++)
  {
      weights_(i) = 0.5/(n_aug_ + lambda_);
  }

  //predict state mean
  x_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
      x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      
      P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  //cout << "Prediction Complete" << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //cout << "Updating LIDAR" << endl;
  //set measurement dimension, lidar can measure x, and y
  int n_z_ = 2;

  //set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i<2 * n_aug_ + 1; i++) {
	  double weight = 0.5 / (n_aug_ + lambda_);
	  weights(i) = weight;
  }

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  
  //transform sigma points into measurement space
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  double p_x = Xsig_pred_(0, i);
	  double p_y = Xsig_pred_(1, i);
	  double v = Xsig_pred_(2, i);
	  double yaw = Xsig_pred_(3, i);
	  //normalize angles
	  while (yaw > M_PI) yaw -= 2.*M_PI;
	  while (yaw <-M_PI) yaw += 2.*M_PI;

	  double v_x = cos(yaw)*v;
	  double v_y = sin(yaw)*v;

	  Zsig(0, i) = p_x;
	  Zsig(1, i) = p_y;	  	  
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  VectorXd z_diff = Zsig.col(i) - z_pred;

	  S = S + weights(i) * z_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_laspx_*std_laspx_, 0,
	  0, std_laspy_*std_laspy_;
  S = S + R;

  //create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z << meas_package.raw_measurements_[0],   //x in m
	  meas_package.raw_measurements_[1];   //y in m

  //LIDAR NIS
  double lidar_nis_ = ((z - z_pred).transpose()) * S.inverse() * (z - z_pred);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  VectorXd x_diff = Xsig_pred_.col(i) - x_;

	  Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
  //cout << lidar_nis_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //cout << "Updating RADAR" << endl;
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_ = 3;

  //set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i<2 * n_aug_ + 1; i++) {
	  double weight = 0.5 / (n_aug_ + lambda_);
	  weights(i) = weight;
  }

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);

  //transform sigma points into measurement space
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  double p_x = Xsig_pred_(0, i);
	  double p_y = Xsig_pred_(1, i);
	  double v = Xsig_pred_(2, i);
	  double yaw = Xsig_pred_(3, i);
	  //normalize angles
	  while (yaw > M_PI) yaw -= 2.*M_PI;
	  while (yaw <-M_PI) yaw += 2.*M_PI;

	  double v_x = cos(yaw)*v;
	  double v_y = sin(yaw)*v;

	  Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);
	  Zsig(1, i) = atan2(p_y, p_x);
	  Zsig(2, i) = (p_x*v_x + p_y*v_y) / sqrt(p_x*p_x + p_y*p_y);
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  //normalize angles
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
	  S = S + weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_radr_*std_radr_, 0, 0,
	  0, std_radphi_*std_radphi_, 0,
	  0, 0, std_radrd_*std_radrd_;
  S = S + R;

  //create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z << meas_package.raw_measurements_[0],   //rho in m
	   meas_package.raw_measurements_[1],   //phi in rad
	   meas_package.raw_measurements_[2];   //rho_dot in m/s

  //RADAR NIS
  double radar_nis_ = ((z - z_pred).transpose()) * S.inverse() * (z - z_pred);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);

  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  //normalize angles
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	  VectorXd x_diff = Xsig_pred_.col(i) - x_;
	  //normalize angles
	  while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
	  while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;
	  Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  //normalize angles
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
  //update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
  //cout << radar_nis_ << endl;
}
