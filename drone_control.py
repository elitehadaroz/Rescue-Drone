# -*- coding: cp1255 -*-
import threading
import dronekit_sitl
import serial
import time
import subprocess
from subprocess import Popen, PIPE
import sys
import os
import signal
import socket
from dronekit import connect, VehicleMode # Import DroneKit-Python5

#this for use simulation in drone 
"""
sitl = dronekit_sitl.start_default()
#connection_string = sitl.connection_string()
"""
"""
def init_mavProxy():
    mavlink_proxy = "mavproxy.py --master='COM4' --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551"
    mavlink_proxyProcess = subprocess.Popen(mavlink_proxy,shell=True, stdout=subprocess.PIPE)
"""   
def inint_person_detection():
    print("start process")
    person_detection = "py object_detection_tutorial_con.py"  # launch the person_detection script using bash
    person_detectionProcess = subprocess.Popen(person_detection,shell=False, stdout=subprocess.PIPE)
    print("waiting for activation")
    time.sleep(10)
    print("ok")
    while True:
        line = person_detectionProcess.stdout.readline().rstrip('\n')
        if not line:
            break
        print(line)


#this if we want to send msg with udp socket
"""
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print ("received message:", data)
"""

def connecting_drone():
    # Connect to the Vehicle.
    print("Connecting to vehicle on: udp:127.0.0.1:14551")
    #connecting to the vehicle by udp- this we can also open the massion planner with the python
    vehicle = connect("127.0.0.1:14551",wait_ready=False,baud=57600)
    #wait_ready is for that all info from drune upload 100%
    vehicle.wait_ready(True, timeout=42)
    print("the vehicle is ready")
    
    #vehicle attributes (state)
    print("Get some vehicle attribute values:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)    # settable

    #Close vehicle object before exiting script
    while True:
        #print(vehicle.location.capabilities)
        #print("the latiiiiiii is:%s"% LocationGlobal)
        #print("theiiiiiiiii lon is:%s"% vehicle.location.global_frame.lon)
        print("the lat is:%s"% vehicle.location.global_frame.lat)
        print("the lon is:%s"% vehicle.location.global_frame.lon)
        time.sleep(4)
"""
def arm_and_takeoff(aTargetAltitude):

    #Arms vehicle and fly to aTargetAltitude.
    

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            vehicle.mode = VehicleMode("AUTO")
            while True:
              print("the lat is:%s"% vehicle.location.global_frame.lon)  
            break
        time.sleep(1)


arm_and_takeoff(10)

vehicle.mode = VehicleMode("AUTO")
"""
if __name__ == "__main__":
    
    person_detection_thread = threading.Thread(name='inint_person_detection', target=inint_person_detection)
    #mavProxy_thread = threading.Thread(name='init_mavProxy', target=init_mavProxy)
    #mavProxy_thread.start()
    #time.sleep(5)
    person_detection_thread.start()
    
    print("t is working")
    #init_mavProxy()
    connecting_drone()
#time.sleep(10)
# Shut down simulator
#sitl.stop()
#print("Completed")
