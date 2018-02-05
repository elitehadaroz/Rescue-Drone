# -*- coding: cp1255 -*-
import threading
import queue
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
import tkinter
import cv2
import numpy
import struct
import pickle
vehicle =None
#this for use simulation in drone 

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

"""
def init_mavProxy():
    mavlink_proxy = "mavproxy.py --master='COM4' --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551"
    mavlink_proxyProcess = subprocess.Popen(mavlink_proxy,shell=True, stdout=subprocess.PIPE)
"""

def inint_person_detection():
    print("start process")
    person_detection = "py person_detection.py"  # launch the person_detection script using bash
    person_detectionProcess = subprocess.Popen(person_detection,shell=True, stdout=subprocess.PIPE)
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
"""
def mavProxy():
    
    mavProxy = 'mavproxy.py --master="COM4" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
    proc = subprocess.Popen(mavProxy,shell=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    print("is in mavproxy")
    while True:
        connect = proc.stdout.readline().split(" ")[0] #if the line = Saved 773 parmeters to mav.prm the mavProxy connect to 3DR.
        print(connect) 
        if(str(connect) == "Saved"):
            print("issssss"+connect) 
            queue.put(connect)
            break
"""
    

    
def connecting_drone():
    # Connect to the Vehicle.
    print "Connecting to vehicle on: udp:127.0.0.1:14551"
    #connecting to the vehicle by udp- this we can also open the massion planner with the python
    vehicle = connect("127.0.0.1:14551",wait_ready=False,baud=57600)
    #wait_ready is for that all info from drune upload 100%
    vehicle.wait_ready(True, timeout=50)
    print("the vehicle is ready")
    
    #vehicle attributes (state)
    print("Get some vehicle attribute values:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)    # settable
    #arm_and_takeoff(vehicle)
    #Close vehicle object before exiting script
    while True:
        #print(vehicle.location.capabilities)
        #print("the latiiiiiii is:%s"% LocationGlobal)
        #print("theiiiiiiiii lon is:%s"% vehicle.location.global_frame.lon)
        print("the lat is:%s"% vehicle.location.global_frame.lat)
        print("the lon is:%s"% vehicle.location.global_frame.lon)
        time.sleep(4)
"""
def arm_and_takeoff(vehicle):

    #Arms vehicle and fly to aTargetAltitude.
    

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    #while not vehicle.is_armable:
    #    print(" Waiting for vehicle to initialise...")
    #    time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    time.sleep(15)
    vehicle.armed   = False

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
    
    #queue = Queue.Queue()
    #mavThread =threading.Thread(name='mavProxyThread',target=mavProxy)
    #mavThread.start()
    #while True:
    #    if(queue.get() == "Saved"):
    #        print(queue.get())
    HOST=''
    PORT=8080
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    print 'Socket created'
    
    print("connect to person")
    person_detection_thread = threading.Thread(name='inint_person_detection', target=inint_person_detection)
    #mavProxy_thread = threading.Thread(name='init_mavProxy', target=init_mavProxy)
    #mavProxy_thread.start()
    #time.sleep(5)
    person_detection_thread.start()
    #time.sleep(20)
    s.bind((HOST,PORT))
    print 'Socket bind complete'
    s.listen(10)
    print 'Socket now listening'
    conn,addr=s.accept()
    

    data = ""
    payload_size = struct.calcsize("L") 
    while True:
        while len(data) < payload_size:
            data += conn.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]
        while len(data) < msg_size:
            data += conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]
        ###

        frame=pickle.loads(frame_data)
        print frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(50) & 0xFF == ord('q'):
          cv2.destroyAllWindows()
          break
    print("t is working\n")
    #init_mavProxy()
    connecting_drone()
    #time.sleep(10)
    # Shut down simulator
    #sitl.stop()
    #print("Completed")
