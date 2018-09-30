#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;  // x is the object state
    P_ = P_in;  // P is the state covariance matrix
    F_ = F_in;  // F is the state transition matrix
    H_ = H_in;  // H is the measurement matrix
    R_ = R_in;  // R is the measurement covariance matrix
    Q_ = Q_in;  // Q is the process covariance matrix
}

void KalmanFilter::Predict() {
	// Predict the state
	x_ = F_ * x_; // the random noise u is factored into P_
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_; //P_ is the uncertainty delta, state covariance
}

void KalmanFilter::Update(const VectorXd &z) {
	//update the state(position) 
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ *Ht + R_; //R is the measurement covariance matrix, user defined
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;
	
	//Update to New state
	x_ = x_ + (K * y); // new state
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_; //new state covariance
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO: update the state by using Extended Kalman Filter equations
	*/
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	
	float rho = sqrt( px*px + py*py);
	float phi = atan2(py, px);
	float rho_dot = fabs(rho) > 0.0001 ? (px*vx + py*vy) / rho : 0; 

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

    VectorXd y = z- z_pred;
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    // M_PI is the pi value define from the Geometry.h lib
    while ( y[1] > M_PI || y[1] < -M_PI ) {
        y[1] = y[1] > M_PI ? y[1] - M_PI: y[1] + M_PI;
    }

    //Update to New state
	x_ = x_ + (K * y); // new state
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_; //new state covariance
}