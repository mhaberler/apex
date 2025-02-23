#ifndef BALLOON_EKF_H
#define BALLOON_EKF_H

#include <Eigen/Dense>

class BalloonEKF {
private:
    Eigen::Vector4d x;          // State: [altitude, velocity, acceleration, burner_gain]
    Eigen::Matrix4d P;          // State covariance
    Eigen::Matrix4d Q;          // Process noise covariance
    double R;                   // Measurement noise covariance
    Eigen::Matrix4d F;          // Linearized state transition matrix
    Eigen::Vector4d B;          // Control input effect vector
    Eigen::RowVector4d H;       // Measurement matrix
    double lastTime;            // Last measurement time
    double lastLoudnessDuration; // Last control input for Jacobian
    bool initialized;

public:
    BalloonEKF() {
        x << 0.0, 0.0, 0.0, 0.1; // k = 0.1 m/s^3 per loudness-duration unit
        P.setIdentity();
        P *= 10.0;
        P(3, 3) = 1.0;           // Higher initial uncertainty for k
        Q.setZero();
        Q(0, 0) = 0.01;          // Altitude noise
        Q(1, 1) = 0.1;           // Velocity noise
        Q(2, 2) = 0.5;           // Acceleration noise
        Q(3, 3) = 0.001;         // Small noise for k to allow adaptation
        R = 0.1;
        H << 1.0, 0.0, 0.0, 0.0; // Measure only altitude
        B << 0.0, 0.0, 1.0, 0.0; // Control affects acceleration
        lastTime = -1.0;
        lastLoudnessDuration = 0.0;
        initialized = false;
    }

    void processMeasurement(double time, double altitude, double loudness, double duration) {
        if (!initialized) {
            x(0) = altitude;
            lastTime = time;
            initialized = true;
            return;
        }

        double dt = time - lastTime;
        if (dt <= 0) {
            throw std::runtime_error("Time must increase between measurements");
        }

        double loudnessDuration = loudness * duration;
        predict(dt, loudnessDuration);
        update(altitude);
        lastTime = time;
        lastLoudnessDuration = loudnessDuration;
    }

    void predict(double dt, double loudnessDuration) {
        Eigen::Vector4d x_pred;
        x_pred(0) = x(0) + x(1) * dt + 0.5 * x(2) * dt * dt;
        x_pred(1) = x(1) + x(2) * dt;
        x_pred(2) = x(2) + x(3) * loudnessDuration;
        x_pred(3) = x(3);

        F.setIdentity();
        F(0, 1) = dt;
        F(0, 2) = 0.5 * dt * dt;
        F(1, 2) = dt;
        F(2, 3) = loudnessDuration;

        x = x_pred;
        P = F * P * F.transpose() + Q;
    }

    void update(double altitude) {
        double z = altitude;
        double y = z - H * x; // Scalar residual
        double S = (H * P * H.transpose())(0, 0) + R; // Scalar innovation covariance
        Eigen::Vector4d K = P * H.transpose() / S;    // Kalman gain (4x1)
        x = x + K * y;
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        P = (I - K * H) * P;
    }

    bool isDecelerating(double& timeToZeroSpeed) {
        if (!initialized) {
            return false;
        }

        double v = x(1);
        double a = x(2);

        if (v * a < 0) {
            timeToZeroSpeed = -v / a;
            return true;
        }
        return false;
    }

    double getAltitude() const { return x(0); }
    double getVelocity() const { return x(1); }
    double getAcceleration() const { return x(2); }
    double getBurnerGain() const { return x(3); }
};

#endif // BALLOON_EKF_H