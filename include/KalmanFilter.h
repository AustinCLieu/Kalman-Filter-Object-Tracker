#pragma once // ensures header file is included only once during a single compilation

#include <Eigen/Dense> // brings in Eigen's VectorXd and MatrixXd types

// A linear Kalman filter for a constant velocity 2D target
// State vector x = [px, py, vx, vy ] (position and velocity)
// The filter hoplds its current state estimate (x_) and how uncertain it is
// about that estimate (P_, the covariance matrix), and advances both through
// a predict/update cycle
class KalmanFilter
{
public:
    // Constructor: We hand the Kalman filter the four model matrices up front once:
    // F: state transition (motion model)
    // H: measurement matrix (maps state -> 2d x and y, what the sensor sees)
    // Q: process noise covariance (how much we distrust the model)
    // R: measurement noise covariance (how much we distrust the sensor)
    KalmanFilter(const Eigen::MatrixXD &F,
                 const Eigen::MatrixXd &H,
                 const Eigen::MatrixXd &Q,
                 const Eigen::MatrixXd &R);

    // Set the starting state estimate (x0) and starting uncertainty (P0)
    void init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0);

    // Phase 1: project the state and covariance forward using the model
    void predict();

    // Phase 2: correct the prediction using a new measurement z
    void update(const Eigen::VectorXd &z);

    // Read-only access to the current estimate and uncertainty
    // The trailing 'const' promises that these methods don't modify the object
    const Eigen::VectorXd &state() const { return x_; }
    const Eigen::MatrixXd &covariance() const { return P_; }

private:
    Eigen::VectorXd x_; // state estimate (4x1)
    Eigen::MatrixXd P_; // estimate covariance (4x4)
    Eigen::MatrixXd F_; // state transition (4x4)
    Eigen::MatrixXd H_; // measurement matrix (2x4)
    Eigen::MatrixXd Q_; // process noise covariance (4x4)
    Eigen::MatrixXd R_; // measurement noise (2x2)
};