#ifndef DRONE_SIMULATOR_H
#define DRONE_SIMULATOR_H

#include "mavlink_handler.h"
#include <thread>
#include <atomic>
#include <mutex>

class DroneSimulator {
public:
    DroneSimulator(const std::string& target_ip, int target_port, int bind_port);
    ~DroneSimulator();

    void startSimulation();
    void stopSimulation();

    // Méthodes pour mettre à jour l'état du drone (ex: depuis une GUI ou un contrôleur)
    void setPosition(double lat, double lon, double alt);
    void setAttitude(double roll, double pitch, double yaw);
    void setVelocity(double vx, double vy, double vz);

private:
    MAVLinkHandler mavlink_handler_;
    std::thread simulation_thread_;
    std::atomic<bool> running_;
    std::mutex data_mutex_;

    // État du drone simulé
    double latitude_deg_;
    double longitude_deg_;
    double altitude_m_;
    double relative_altitude_m_;

    double roll_deg_;
    double pitch_deg_;
    double yaw_deg_; // Heading

    double velocity_x_mps_;
    double velocity_y_mps_;
    double velocity_z_mps_;

    // Simulation loop
    void simulationLoop();
    void updateDroneState();
    void sendTelemetry();
};

#endif // DRONE_SIMULATOR_H