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
from Tkinter import *
from PIL import Image, ImageTk
import numpy as np

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
############################################# person detection ################################################################
class PersonDetection:
    def __init__(self):
        #self.videoframe = videoframe
        
        print ('Socket created')
        print ('detection process')
        person_detection = "py person_detection.py"  # launch the person_detection script using bash
        self.person_detectionProcess = subprocess.Popen(person_detection,shell=True, stdout=subprocess.PIPE)
        """
        print("now video")
        #video thread
        self.socket_video=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.HOST=''
        self.PORT=8089
        print("conecting to person detection")
        self.socket_video.bind((self.HOST,self.PORT))
        print 'Socket bind complete'
        self.socket_video.listen(10)
        print 'Socket now listening'
        self.conn,self.addr=self.socket_video.accept()
        self.data = ""
        self.payload_size = struct.calcsize("L")
        person_detection_video = threading.Thread(name='person_detection_video',target=self.DetectionVideo(yt))
        person_detection_video.start()
        """
        #msg thread
        #self.DetectionMsg(self.person_detectionProcess)
        self.person_detection_msg = threading.Thread(name='person_detection_msg',target=self.DetectionMsg)
        self.person_detection_msg.start()
        
        print("hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
    def Close_detection(self):
        self.person_detection_msg.join()
        self.person_detectionProcess.kill()
        
    def DetectionMsg(self):
        #time.sleep(20)
        print("ok")
        while True:
            line = self.person_detectionProcess.stdout.readline().rstrip('\n')
            if not line:
                break
            print(line)

    def DetectionVideo(self,yt):
        #print("conecting to person detection")
        #self.socket_video.bind((self.HOST,self.PORT))
        #print 'Socket bind complete'
        #self.socket_video.listen(10)
        #print 'Socket now listening'
        #conn,addr=self.socket_video.accept()
        #data = ""
        #payload_size = struct.calcsize("L")
        """
        while True:
            while len(self.data) < self.payload_size:
                self.data += self.conn.recv(4096)
            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            while len(self.data) < msg_size:
                self.data += self.conn.recv(4096)
            frame_data = self.data[:msg_size]
            self.data = self.data[msg_size:]
            

            frame=pickle.loads(frame_data)
            #cv2.imshow('frame',frame)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            yt.imgtk = imgtk
            yt.configure(image=imgtk)
            yt.after(10, DetectionVideo)
            if cv2.waitKey(50) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break
"""
###############################################################################################################################
################################################ Drone Control ################################################################
"""
class DroneControl:
    def __init(self):
"""
###############################################################################################################################
##################################################### Gui #####################################################################
class Gui:
    def __init__(self,master):      
        master.geometry("950x650")
        master.title("Rescue Drune")

        #section the main frame to rows and columns.
        for x in xrange(5):
            master.grid_columnconfigure(x, weight=1)
        for y in xrange(5):    
            master.grid_rowconfigure(y, weight=1)
            
        #drone_control frames
        drone_control = Frame(master, bg='yellow')
        
        for x in xrange(2):
            drone_control.grid_columnconfigure(x, weight=1)
        for y in xrange(4):    
            drone_control.grid_rowconfigure(y, weight=1)
            
        drone_control.grid(row=0 ,column=1,columnspan=5,sticky=W+N+E+S)
        
        #video_window frames
        self.video_window = Label(master,width=65,height=26,borderwidth=2, relief="groove",bg="gray")
        self.video_window.grid(row=0,column=0,sticky=W+N+E+S,padx=5, pady=5)
        
        #msg_drone frames
        indication_frame = Frame(master,height=200, bg='red')
        for x in xrange(5):
            indication_frame.grid_columnconfigure(x, weight=1)
        for y in xrange(2):    
            indication_frame.grid_rowconfigure(y, weight=1)
        indication_frame.grid(row=1,column=0,columnspan=6,rowspan=5,sticky=W+N+E+S)

        #frame for msg from the drone and software
        monitor_msg=Text(indication_frame,width=30)
        monitor_msg.grid(row=0,column=0,sticky=W+N+S+E)
        scrollbar = Scrollbar(indication_frame)
        scrollbar.grid(row=0,column=1,sticky=W+N+S)
        monitor_msg.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=monitor_msg.yview)
        

        self.isConnect=False
        self.button_connect=Button(drone_control,text="Connect",width=9, height=2, command=lambda:self.Switch_OnOff(master))
        self.button_connect.grid(row=0,column=1,sticky=W+N,pady=4)
       
        self.button_auto=Button(drone_control,text="Auto Search",width=9,height=3)
        self.button_auto.grid(row=1,column=0,columnspan=1,sticky=W+N,padx=4,pady=4)

        self.button_manual=Button(drone_control,text="Manual",width=9,height=3)
        self.button_manual.grid(row=1,column=1,columnspan=1,sticky=W+N,pady=4)
        
        self.button_rtl=Button(drone_control,text="LTR",width=9,height=3)
        self.button_rtl.grid(row=1,column=2,columnspan=1,sticky=W+N,padx=4,pady=4)
        
        """
        self.button_connect=Button(drone_control,text="Connect",width=9, height=2, command=lambda:self.Switch_OnOff(master))
        self.button_connect.place(relx=0.5, rely=0.01,anchor=N)
       
        self.button_auto=Button(drone_control,text="Auto",width=7,height=3)
        self.button_auto.place(relx=0.1, rely=0.3,anchor=W)

        self.button_manual=Button(drone_control,text="Manual",width=7,height=3)
        self.button_manual.place(relx=0.5, rely=0.3,anchor=CENTER)
        
        self.button_rtl=Button(drone_control,text="LTR",width=7,height=3)
        self.button_rtl.place(relx=0.9, rely=0.3,anchor=E)
        """         
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def Switch_OnOff(self,master):
        print("in switcher")
        if self.isConnect is False:
            print("is here")
            self.button_connect.config(text="Disconnect")
            self.isConnect=True
            self.Connect(master)
        else:
            self.button_connect.config(text="Connect")
            self.isConnect=False
            self.Disconnect(master)

    def Connect(self,master):  #connect to the system
        self.p=PersonDetection()
        self.person_detection_video = threading.Thread(name='person_detection_video',target=lambda:self.CamDrone(master))
        self.person_detection_video.start()
        
    def Disconnect(self,master):    #disconnect from button
        print("in disccinecttttttttttttttttttttttttttttt")
        self.p.Close_detection()
        self.isConnect=False
        #time.sleep(1)
        self.video_window.destroy()
        self.video_window = Label(master,width=65,height=26,borderwidth=2, relief="groove",bg="gray")
        self.video_window.grid(row=0,column=0,sticky=W+N+E+S,padx=5, pady=5)
        self.person_detection_video.join()
        self.socket_video.close()
        
    def on_closing(self):   #when the user close the window on X
        if self.isConnect is True:
            self.isConnect=False
            time.sleep(1)
            self.person_detection_video.join()
            root.destroy()
        else:
            root.destroy()
            
    def CamDrone(self,master):
        self.socket_video=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        HOST=''
        PORT=8089
        print("conecting to person detection")
        self.socket_video.bind((HOST,PORT))
        print 'Socket bind complete'
        self.socket_video.listen(10)
        print 'Socket now listening'
        conn,addr=self.socket_video.accept()
        data = ""
        payload_size = struct.calcsize("L")
        #time.sleep(5)
        #self.vedeo=Label(self.video_window)
        #self.vedeo.grid()
        #self.video_window.config(width=500,height=450)
        #self.video_window.destroy()
        #self.video = Label(master,borderwidth=2, relief="groove")
        #self.video.grid(row=0,column=0,sticky=W+N+E+S,padx=5, pady=5)
        openLabel=False
        while self.isConnect:
            while len(data) < payload_size:
                data += conn.recv(4096)
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            while len(data) < msg_size:
                data += conn.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            if openLabel is False:
                self.video_window.config(width=460,height=400)
                openLabel=True
            last_frame=pickle.loads(frame_data)
            last_frame = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)
            #cv2.resize(last_frame, (400,400))
            #cv2.imshow('frame',last_frame)
            img = Image.fromarray(last_frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_window.imgtk = imgtk
            self.video_window.configure(image=imgtk)
            #self.video_window.after(10, self.CamDrone)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

    
    #def camera(self):
        
###############################################################################################################################
##################################################### main ####################################################################
if __name__ == "__main__":
    root =Tk()
    #Person_Detection_Tread = threading.Thread(name='PersonDetection', target=PersonDetection)
    #Person_Detection_Tread.start()
    gui = Gui(root)
    
    
    root.mainloop()
    print("after person detection")
    #queue = Queue.Queue()
    #mavThread =threading.Thread(name='mavProxyThread',target=mavProxy)
    #mavThread.start()
    #while True:
    #    if(queue.get() == "Saved"):
    #        print(queue.get())
    #HOST=''
    #PORT=8089
    #s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    #print 'Socket created'
    
    #print("connect to person")
    #person_detection_thread = threading.Thread(name='inint_person_detection', target=inint_person_detection)
    #mavProxy_thread = threading.Thread(name='init_mavProxy', target=init_mavProxy)
    #mavProxy_thread.start()
    #time.sleep(5)
    #person_detection_thread.start()
    #time.sleep(20)
    """
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
    """
    #init_mavProxy()
    #connecting_drone()
    #time.sleep(10)
    # Shut down simulator
    #sitl.stop()
    #print("Completed")
