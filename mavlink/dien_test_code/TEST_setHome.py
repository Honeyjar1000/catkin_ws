################################################
################################################
###
###     Connect Raspberry pi Autopilot
###
################################################
################################################

from pymavlink import mavutil
import time

connect_string = "udp:localhost:14551"
baud_rate = 57600

connection = mavutil.mavlink_connection(connect_string, baud_rate)
print("Connected")
connection.wait_heartbeat(timeout=2)
print("Heartbeat from system (system %u component %u)" % 
(connection.target_system, connection.target_component))

# Set global possition for reference
connection.mav.set_gps_global_origin_send(connection.target_system, int(-37.8*1e7), int(144.911*1e7), int(7.0))

##disarm 
##throttle