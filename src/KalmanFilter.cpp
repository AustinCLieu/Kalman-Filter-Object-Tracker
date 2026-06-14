#include "KalmanFilter.h"

// Constructor: Store the four model matrices, and give x_ and P_ acceptable default values
// until the caller supplies real starting values through init()
KalmanFilter::KalmanFilter(const Eigen::MatrixXd &F,
                           const Eigen::MatrixXd &H,
                           const Eigen::MatrixXd &Q,
                           const Eigen::MatrixXd &R)
    : F_(F), H_(H), Q_(Q), R_(R) // Member initializer list
{
    x_ = Eigen::VectorXd::Zero(F_.rows());                // state starts at 0 everything
    P_ = Eigen::MatrixXd::Identify(F_.rows(), F_.rows()); // identify covariance
}

void KalmanFilter::init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0)
{
    x_ = x0;
    P_ = P0;
}

void KalmanFilter::predict()
{
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
    (void)z;
}