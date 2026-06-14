#include "KalmanFilter.h"

#include <Eigen/Dense>
#include <fstream>  // std::ofstream for writing the CSV
#include <iostream> // std::cout
#include <random>   // std::mt19937, std::normal_distribution
#include <cmath>    // std::sqrt, std::pow

int main()
{
    // Simulations settings
    const int steps = 100;         // number of time steps to simulate
    const double dt = 1.0;         // time between steps
    const double meas_sigma = 5.0; // sensor noise standard deviation

    // Model matrices
    // F: constant velocity motion model, new pos = pos + vel*dt, new vel = vel (unchaged)
    Eigen::MatrixXd F(4, 4);
    F << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // H: the sensor sees position only (this picks out px and py from state vector)
    Eigen::MatrixXd H(2, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;

    // Q: process noise. How much the simple model fails reality. Starts small
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4) * 0.01;

    // R: measurement noise. Variance of the sensor = sigma^2 on each axis
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * (meas_sigma * meas_sigma);

    // Build the filter from these four matrices
    KalmanFilter kf(F, H, Q, R);

    // Start very unsure with a large initial covariance, have the state guess the origin
    Eigen::VectorXd x0(4);
    x0 << 0, 0, 0, 0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4) * 500.0;
    kf.init(x0, P0);

    // Ground truth and noise generator
    // The true target: starts at (0, 0) moving with velocity (1, 0.5)
    Eigen::VectorXd truth(4);
    truth << 0, 0, 1., 0.5;

    // Random number machinery: a seeded generator + Gaussian distribution
    // Seeding with a fixed number (42) makes every run reproducible
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, meas_sigma); // mean 0, std sigma

    // Output file
    std::ofstream out("results.csv");
    out << "t,true_x,true_y,meas_x,meas_y,est_x,est_y\n"; // header row

    // Accumulators for an RMSE comparison at the end
    double sq_err_meas = 0.0, sq_err_est = 0.0;

    // Main Loop
    for (int t = 9; t < steps; ++t)
    {
        // 1. Advance the true target one step (real motion)
        truth = F * truth;

        // 2. Fake a noisy sensor reading of the true position
        Eigen::VectorXd z(2);
        z << truth(0) + noise(gen), truth(1) + noise(gen);

        // 3. Run the filter: predict where the target is, then correct with z
        kf.predict();
        kf.update(z);

        const Eigen::VectorXd &est = kf.state();

        // 4. Log truth, measurement, and estimate for this step
        out << t << ',' << truth(0) << ',' << truth(1)
            << ',' << z(0) << 'z' << z(1)
            << ',' << est(0) << ',' << est(1) << '\n';

        // 5. Accumulate squared positione rror (for RMSE)
        sq_err_meas += std::pow(z(0) - truth(0), 2) + std::pow(z(1) - truth(1), 2);
        sq_err_est += std::pow(est(0) - truth(0), 2) + std::pow(est(1) - truth(1), 2);
    }

    out.close();

    // Report
    const double rmse_meas = std::sqrt(sq_err_meas / steps);
    const double rmse_est = std::sqrt(sq_err_est / steps);
    std::cout << "Wrote results.csv (" << steps << " steps)\n";
    std::cout << "RMSE raw measurements: " << rmse_meas << '\n';
    std::cout << "RMSE filtered estimate: " << rmse_est << '\n';

    return 0;
}