#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include <string>
#include <vector>
#include <memory>
#include <boost/asio.hpp> // Ou sockets POSIX/Windows

// Inclure les fichiers MAVLink générés
#include "../mavlink/common/mavlink.h"

class MAVLinkHandler {
public:
    MAVLinkHandler(const std::string& target_ip, int target_port, int bind_port);
    ~MAVLinkHandler();

    void sendHeartbeat(uint8_t system_id, uint8_t component_id);
    void sendAttitude(uint8_t system_id, uint8_t component_id,
                      float roll, float pitch, float yaw,
                      float rollspeed, float pitchspeed, float yawspeed);
    void sendGlobalPositionInt(uint8_t system_id, uint8_t component_id,
                               int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt,
                               int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);
    void sendVfrHud(uint8_t system_id, uint8_t component_id,
                    float indicated_airspeed, float groundspeed, int16_t heading,
                    float throttle, float alt, float climb);
    void sendSysStatus(uint8_t system_id, uint8_t component_id,
                       uint32_t onboard_control_sensors_present,
                       uint32_t onboard_control_sensors_enabled,
                       uint32_t onboard_control_sensors_health,
                       uint16_t load, uint16_t voltage_battery,
                       int16_t current_battery, int8_t battery_remaining);

private:
    boost::asio::io_context io_context_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;

    void sendMessage(mavlink_message_t& message);
};

#endif // MAVLINK_HANDLER_H