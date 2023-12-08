"""
Send MAVLink Commands
"""

__author__ = 'thomasw'
__date__ = '$January 2023$'

if __name__ == "__main__":
    import pymavlink
    from pymavlink import mavutil
    from time import sleep

    #pin_str = "localhost:14550"
    #pin_str = "udpout:ip:14551"
    #pin_str = "udpout:192.168.1.65:14550"
    #pin_str = "udp:localhost:14551"
    pin_str = "/dev/ttyUSB0"
    baurd_rate = 57600
    the_connection = mavutil.mavlink_connection(pin_str, baurd_rate)
    
    sleep(1)
    while True:
        data = the_connection.recv_match()
        #data = the_connection.recv_match()
        if data is not None:
            if str(type(data)) == "<class 'pymavlink.dialects.v20.ardupilotmega.MAVLink_vision_position_estimate_message'>":
            
                x = round(getattr(data, "x"),5)
                y = round(getattr(data, "y"), 5)
                z = round(getattr(data, "z"), 5)
                roll = round(getattr(data, "roll"), 5)
                pitch = round(getattr(data, "pitch"), 5)
                yaw = round(getattr(data, "yaw"), 5)
                s = "x: " + str(x) + "\t| y: " + str(y) + "\t| z: " + str(z) + "\n"
                s += "r: " + str(roll) + "\t| p: " + str(pitch) + "\t| y: " + str(yaw) + "\n\n"
                
                print(s)
            else:
                print(data)