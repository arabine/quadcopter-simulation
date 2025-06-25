#include "mavlink_handler.h"
#include "utils.h"
#include <iostream>

MAVLinkHandler::MAVLinkHandler(const std::string& target_ip, int target_port, int bind_port)
    : socket_(io_context_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), bind_port)),
      remote_endpoint_(boost::asio::ip::address::from_string(target_ip), target_port) {
    std::cout << "MAVLinkHandler initialized. Target: " << target_ip << ":" << target_port
              << ", Local bind port: " << bind_port << std::endl;
}

MAVLinkHandler::~MAVLinkHandler() {
    if (socket_.is_open()) {
        socket_.close();
    }
}

void MAVLinkHandler::sendMessage(mavlink_message_t& message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    try {
        socket_.send_to(boost::asio::buffer(buffer, len), remote_endpoint_);
    } catch (const boost::system::system_error& ex) {
        std::cerr << "Error sending MAVLink message: " << ex.what() << std::endl;
    }
}

void MAVLinkHandler::sendHeartbeat(uint8_t system_id, uint8_t component_id) {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    heartbeat.type = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
    heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED; // Example
    heartbeat.custom_mode = 0; // Depends on your custom modes
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = 3; // MAVLink 2.0

    mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &heartbeat);
    sendMessage(msg);
}

void MAVLinkHandler::sendAttitude(uint8_t system_id, uint8_t component_id,
                                  float roll, float pitch, float yaw,
                                  float rollspeed, float pitchspeed, float yawspeed) {
    mavlink_message_t msg;
    mavlink_attitude_t attitude;

    attitude.time_boot_ms = Utils::getCurrentTimestampMs();
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    attitude.rollspeed = rollspeed;
    attitude.pitchspeed = pitchspeed;
    attitude.yawspeed = yawspeed;

    mavlink_msg_attitude_encode(system_id, component_id, &msg, &attitude);
    sendMessage(msg);
}

void MAVLinkHandler::sendGlobalPositionInt(uint8_t system_id, uint8_t component_id,
                                           int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt,
                                           int16_t vx, int16_t vy, int16_t vz, uint16_t hdg) {
    mavlink_message_t msg;
    mavlink_global_position_int_t global_pos;

    global_pos.time_boot_ms = Utils::getCurrentTimestampMs();
    global_pos.lat = lat;
    global_pos.lon = lon;
    global_pos.alt = alt;
    global_pos.relative_alt = relative_alt;
    global_pos.vx = vx;
    global_pos.vy = vy;
    global_pos.vz = vz;
    global_pos.hdg = hdg;

    mavlink_msg_global_position_int_encode(system_id, component_id, &msg, &global_pos);
    sendMessage(msg);
}

void MAVLinkHandler::sendVfrHud(uint8_t system_id, uint8_t component_id,
                                float indicated_airspeed, float groundspeed, int16_t heading,
                                float throttle, float alt, float climb) {
    mavlink_message_t msg;
    mavlink_vfr_hud_t vfr_hud;

    vfr_hud.airspeed = indicated_airspeed;
    vfr_hud.groundspeed = groundspeed;
    vfr_hud.heading = heading;
    vfr_hud.throttle = throttle;
    vfr_hud.alt = alt;
    vfr_hud.climb = climb;

    mavlink_msg_vfr_hud_encode(system_id, component_id, &msg, &vfr_hud);
    sendMessage(msg);
}

void MAVLinkHandler::sendSysStatus(uint8_t system_id, uint8_t component_id,
                                   uint32_t onboard_control_sensors_present,
                                   uint32_t onboard_control_sensors_enabled,
                                   uint32_t onboard_control_sensors_health,
                                   uint16_t load, uint16_t voltage_battery,
                                   int16_t current_battery, int8_t battery_remaining) {
    mavlink_message_t msg;
    mavlink_sys_status_t sys_status;

    sys_status.onboard_control_sensors_present = onboard_control_sensors_present;
    sys_status.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    sys_status.onboard_control_sensors_health = onboard_control_sensors_health;
    sys_status.load = load;
    sys_status.voltage_battery = voltage_battery;
    sys_status.current_battery = current_battery;
    sys_status.battery_remaining = battery_remaining;

    mavlink_msg_sys_status_encode(system_id, component_id, &msg, &sys_status);
    sendMessage(msg);
}
