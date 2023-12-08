################################################
################################################
###
###     Test: Take off
###
################################################
################################################

from pymavlink import mavutil
import time
author = '__dienle__'


################################################
################################################
###
###     Connect Raspberry pi Autopilot
###
################################################
################################################

connect_string = "udp:localhost:14551"
baud_rate = 57600

connection = mavutil.mavlink_connection(connect_string, baud_rate)
print("Connected")
connection.wait_heartbeat(timeout=2)
print("Heartbeat from system (system %u component %u)" % 
(connection.target_system, connection.target_component))

# Set global possition for reference
#connection.mav.set_gps_global_origin_send(connection.target_system, int(-37.8*1e7), int(144.911*1e7), int(2.0))

# set mode before arming, then remove bottom set mode
################################################
################################################
###
###     Arm drone
###
################################################
################################################

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, # 1 arm 0 disarm
    0, 0, 0, 0, 0, 0
)
msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print (msg)
print("Armed")
time.sleep(2)


################################################
################################################
###
###     Set Mode
###
################################################
################################################

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, 1,
    4, # set mode (4 is guided)
    0, 0, 0, 0, 0
)
msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print (msg)
print("Mode Set")
time.sleep(2)

################################################
################################################
###
###     Take off and loiter for 10 seconds
###
################################################
################################################



connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 
    2 # altitude (for quad only for fix wing pitch is required)
)
msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print (msg)
print("Take-off")
time.sleep(10)


################################################
################################################
###
###     Return to launch
###
################################################
################################################

connection.mav.command_long_send(
    connection.target_system, 
    connection.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 
    0, 0, 0, 0, 0, 0, 0, 0)

msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print (msg)
print("RETURN TO LAUNCH")
time.sleep(10)


################################################
################################################
###
###     Disarm drone
###
################################################
################################################

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, # 1 arm 0 disarm
    0, 0, 0, 0, 0, 0
)
msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print (msg)
print("Disarmed")
time.sleep(2)



################################################
################################################
###
###     Set Mode
###
################################################
################################################

connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, 1,
    4, # set mode (4 is guided)
    0, 0, 0, 0, 0
)
msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print (msg)
print("mode set")
time.sleep(2)