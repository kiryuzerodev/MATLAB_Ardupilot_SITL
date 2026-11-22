# SITL Bridge - eavesdrops on ArduPilot and gossips to Simulink
# ArduPilot -> MAVProxy (udp 14551) -> this script -> Simulink (udp 9003)
# Packet: [time_s, armed, mode, nav_pitch_rad, nav_roll_rad, mnvr] as 6 little-endian doubles
# Runs alongside the mission script (tcp 5762), they don't fight over anything
# This is an AI generated code, adding comments where needed. Not cross checking everything as figuring out the exact
# data formatting is not needed to know. Only if it works.
# BUT: Read this in detail for future purposes.
from pymavlink import mavutil
import socket, struct, math

SIMULINK = ('127.0.0.1', 9003)

mav = mavutil.mavlink_connection('udp:127.0.0.1:14551') # This comes from official Ardupilot GitHub - check my documentation for link
print("Bridge: waiting for ArduPilot to say hi...")
mav.wait_heartbeat()
print(f"Bridge: connected to system {mav.target_system}. We're in.")

# Politely ask for NAV_CONTROLLER_OUTPUT (msg 62) at 50 Hz - that's where TECS keeps its pitch demand
mav.mav.command_long_send(mav.target_system, mav.target_component,
                          mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                          62, 20000, 0, 0, 0, 0, 0)
# 62 - the whole structure in GitHub is:

# typedef struct __mavlink_nav_controller_output_t
# {
#  float nav_roll; /*< Current desired roll in degrees*/
#  float nav_pitch; /*< Current desired pitch in degrees*/
#  float alt_error; /*< Current altitude error in meters*/
#  float aspd_error; /*< Current airspeed error in meters/second*/
#  float xtrack_error; /*< Current crosstrack error on x-y plane in meters*/
#  int16_t nav_bearing; /*< Current desired heading in degrees*/
#  int16_t target_bearing; /*< Bearing to current MISSION/target in degrees*/
#  uint16_t wp_dist; /*< Distance to active MISSION in meters*/
# } mavlink_nav_controller_output_t;

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
armed = 0.0
mode = 0.0      # ArduPlane: 0 MANUAL, 5 FBWA, 6 FBWB, 7 CRUISE, 10 AUTO, 11 RTL, 13 TAKEOFF
mnvr = 0.0      # whatever the Lua script shouts via gcs:send_named_float("MNVR", x)

while True:
    msg = mav.recv_match(type=['NAV_CONTROLLER_OUTPUT', 'HEARTBEAT', 'NAMED_VALUE_FLOAT'],
                         blocking=True)
    kind = msg.get_type()

    if kind == 'HEARTBEAT':
        if msg.get_srcSystem() != mav.target_system:
            continue    # not the plane, probably the GCS talking to itself
        armed = 1.0 if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else 0.0
        mode = float(msg.custom_mode)
        continue

    if kind == 'NAMED_VALUE_FLOAT':
        if msg.name.strip('\x00') == 'MNVR':
            mnvr = msg.value
        continue

    # NAV_CONTROLLER_OUTPUT: the good stuff. Comes in degrees, Simulink wants radians
    pkt = struct.pack('<6d', msg._timestamp, armed, mode,
                      math.radians(msg.nav_pitch), math.radians(msg.nav_roll), mnvr)
    sock.sendto(pkt, SIMULINK)