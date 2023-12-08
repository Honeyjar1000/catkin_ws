################################################
################################################
###
###     Test: Square Route
###
################################################
################################################

from pymavlink import mavutil
import time

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

# ARM
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
print("mode set")
time.sleep(2)

################################################
################################################
###
###     Take off and loiter for 2 seconds
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
time.sleep(2)

################################################
################################################
###
###     MOVE IN SQUARE
###
################################################
################################################


def move(x, y, z):

    param1 = x # X Postion (m) (positive forward or North)
    param2 = y # Y Position (m) (positive right or East)
    param3 = z # Z Position (m) (positive down)
    param4 = 0 
    param5 = 0
    param6 = 0
    param7 = 0 
    param8 = 0 
    param9 = 0 
    param10 = 0 
    param11 = 0 

    connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, 
            connection.target_system, 
            connection.target_component, 
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
            int(0b110111111000), param1, param2, param3, param4, param5, param6, param7, param8, param9, param10, param11))
time.sleep(5)
    
print("Start move 1")
move(1, 0 , 0)
time.sleep(5)
print("End move 1")

print("Start move 2")
move(0, 1 , 0)
time.sleep(5)
print("End move 2")

print("Start move 3")
move(-1, 0 , 0)
time.sleep(5)
print("End move 3")

print("Start move 4")
move(0, -1, 0)
time.sleep(5)
print("End move 4")

################################################
################################################
###
###     RETURN TO LAUNCH
###
################################################
################################################

connection.mav.command_long_send(
    connection.target_system, 
    connection.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 
    0, 0, 0, 0, 0, 0, 0, 0)

msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print(msg)
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
print("mode set")
time.sleep(2)