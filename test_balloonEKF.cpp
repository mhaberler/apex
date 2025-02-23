#include <iostream>
#include "BalloonEKF.h"

int main() {
    BalloonEKF ekf;
    double timeToZeroSpeed;

    std::cout << "Timeseries 1:\n";
    // Measurements: time, altitude, burnerOn, burnerDuration
    ekf.processMeasurement(0.0, 100.0, false, 0.0); 
    ekf.processMeasurement(1.0, 104.0, false, 0.0);
    ekf.processMeasurement(2.0, 105.0, false, 0.0);
    ekf.processMeasurement(3.0, 106.0, false, 0.0);


    if (ekf.isDecelerating(timeToZeroSpeed)) {
        std::cout << "Decelerating. Time to zero speed: " << timeToZeroSpeed << " s\n";
    } else {
        std::cout << "Not decelerating.\n";
    }

    std::cout << "State - h: " << ekf.getAltitude()
              << ", v: " << ekf.getVelocity()
              << ", a: " << ekf.getAcceleration() 
              << ", bg: " << ekf.getBurnerGain() << "\n";

    std::cout << "\n\nTimeseries 2:\n";

    ekf.processMeasurement(4.0, 107.0, false, 0.0);
    ekf.processMeasurement(5.0, 109.0, false, 0.0);
    ekf.processMeasurement(6.0, 112.0, false, 0.0);

    if (ekf.isDecelerating(timeToZeroSpeed)) {
        std::cout << "Decelerating. Time to zero speed: " << timeToZeroSpeed << " s\n";
    } else {
        std::cout << "Not decelerating.\n";
    }

    std::cout << "State - h: " << ekf.getAltitude()
              << ", v: " << ekf.getVelocity()
              << ", a: " << ekf.getAcceleration() 
              << ", bg: " << ekf.getBurnerGain() << "\n";

    return 0;
}