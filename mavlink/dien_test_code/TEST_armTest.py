"""
arm test (check on mavproxy if drone is armed)
"""
import pymavlink
from pymavlink import mavutil
import time


connect_string = '/dev/ttyUSB0'
baud_rate = 57600


connection = mavutil.mavlink_connection(connect_string, baud_rate)
print("Connected")
connection.wait_heartbeat(timeout=2)
print("Heartbeat from system (system %u component %u)" % 
(connection.target_system, connection.target_component))

# SET MODE TO STABLE
mode = 0
param1 = 1 #(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1)
param2 = mode #Flight Mode Number (FLTMODE1)
param3 = 0 #Empty
param4 = 0 #Empty
param5 = 0 #EmptyMAV_CMD_NAV_GUIDED_ENABLE
param6 = 0 #Empty
param7 = 0 #Empty
connection.mav.command_long_send(connection.target_system, connection.target_component, 
mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, param1, param2, param3, param4, param5, param6, param7)
msg = connection.recv_match(type = 'COMMAND_ACK', blocking=True)
print(msg)
print("System / Component: ", connection.target_system, connection.target_component,)

# ARM
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, # 1 arm 0 disarm
    0, 0, 0, 0, 0, 0
)
print("armed")
time.sleep(1) 
