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
from dronekit import connect, VehicleMode ,APIException# Import DroneKit-Python
import exceptions
import tkinter
import cv2
import numpy
import struct
import pickle
from Tkinter import *
from PIL import Image, ImageTk
import numpy as np


############################################# person detection ################################################################
class person_detection:
    def __init__(self,drone_vehicle_obj):
        #self.videoframe = videoframe
        self.drone_vehicle=drone_vehicle_obj
        print ('Socket created')
        print ('detection process')
        person_detection = "py person_detection.py"  # launch the person_detection script using bash
        self.person_detection_process = subprocess.Popen(person_detection,shell=True, stdout=subprocess.PIPE)

        #msg thread
        #self.DetectionMsg(self.person_detection_process)
        self.person_detection_msg = threading.Thread(name='person_detection_msg',target=self.detection_msg)
        self.person_detection_msg.start()
        
        print("hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
    def close_detection(self):
        self.person_detection_msg.join()
        self.person_detection_process.kill()
        
    def detection_msg(self):
        while True:
            msg_detection = self.person_detection_process.stdout.readline().rstrip('\n')
            if not msg_detection:
                print "im in breack"
                break
            if msg_detection is not None:
                print(msg_detection+ "he he he")
                self.drone_vehicle.person_detected()
                
    
                

###############################################################################################################################
################################################ Drone Control ################################################################

class droneControl:
    def __init__(self,func_msg):
        self.mavlink_connected=False
        self.show_msg_monitor=func_msg
         
    def mav_proxy_connect(self):
        
        self.mavlink_time_out=False #120s to chance connect mavproxy
        mav_proxy = 'mavproxy.py --master="COM4" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        #droneKitSitl ='127.0.0.1:14549'
        self.mavlink_proc = subprocess.Popen(mav_proxy,shell=True,stdin=PIPE,stdout=subprocess.PIPE)
        
        time.sleep(1)################
        print("is in mavproxy")
        self.msg_from_mavlink = threading.Thread(name='mavlink_msg',target=lambda:self.mavlink_msg()) #start to read msg from mavProxy
        self.msg_from_mavlink.start()
        while self.mavlink_connected is False: # wait to connected
            if self.mavlink_time_out is True: #wait only 120s to connected
                print("in oout")
                break
        
        if self.mavlink_connected is True : #the connected succeeded
            print("is need to connect")
            connecting_drone()
        else:                                           #the connected not succeed 
            self.drone_disconnect()                   #kill the subprocess before exit or connect again
            print("error mavlink connect")
               
    def mavlink_msg(self):
        self.stop_timer=False
        self.time_mavlink_connect = threading.Thread(name='wait_to_connect',target=self.timer_connect_mavproxy) #start timer for 120sec
        self.time_mavlink_connect.start()
        
        while self.mavlink_time_out is False:     #read the msg from mavproxy
            print("close theard mavlink wait")
            connect = self.mavlink_proc.stdout.readline().split(" ")[0] #if the line = Saved 773 parmeters to mav.prm the mavProxy connect to 3DR.
            print(connect)
            if(str(connect) == "Failed"): 
                self.drone_disconnect()                      #kill the subprocess before exit or connect again
                break
            if(str(connect) == "Saved"):        #succeeded
                print(connect)
                print("close theard mavlink wait")
                self.stop_timer=True
                self.mavlink_connected=True 
                break
                 
    def timer_connect_mavproxy(self):    #this function is timer to connect mavProxy,if is not connected after 120s is stop the connected.
        sec=0
        while self.stop_timer is False:
            sec+=1
            if sec == 3:
                self.mavlink_time_out=True
                break
            time.sleep(1)  
        print("time out")

    def drone_disconnect(self):
        self.mavlink_time_out=True                    #reset the timer to connection
        #gui.drone_is_connect=False                    #to reset the option to connect again
        #gui.button_connect.config(text="Connect")   #to reset the name button to connect again
        pid=self.mavlink_proc.pid
        subprocess.Popen('taskkill /F /T /PID %i' % pid,shell=True)
        print("kill the proc")
        
    def auto_mode(self):
        self.arm_and_takeoff(20)
        self.vehicle.mode = VehicleMode("AUTO")
        print"after switch auto mode in drone"
    def arm_and_takeoff(self,target_altitude):  #Arms vehicle and fly to target_altitude.    
        if self.vehicle.armed is False:
            print("Basic pre-arm checks")
            # Don't try to arm until autopilot is ready
            while not self.vehicle.is_armable:
                print(" Waiting for vehicle to initialise...")
                time.sleep(1)

            print("Arming motors")
            # Copter should arm in GUIDED mode
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True

            # Confirm vehicle armed before attempting to take off
            while not self.vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(1)

            print("Taking off!")
            self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

            # Wait until the vehicle reaches a safe height before processing the goto
            #  (otherwise the command after self.vehicle.simple_takeoff will execute
            #   immediately).
            while True:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)
        else:
            print("the copter is armed")
        
    def connecting_drone(self):

        # Connect to the Vehicle.
        print "Connecting to vehicle on: udp:127.0.0.1:14551"
        try:
            #connecting to the vehicle by udp- this we can also open the massion planner with the python
            self.vehicle = connect("127.0.0.1:14551",wait_ready=False,baud=57600)
            #wait_ready is for that all info from drune upload 100%
            self.vehicle.wait_ready(True, timeout=60)
            #print("the vehicle is ready")
        # Bad TCP connection
        except socket.error:
            print 'No server exists!'

        # Bad TTY connection
        except exceptions.OSError as e:
            print 'No serial exists!'
        
        # API Error
        except APIException:
            print 'Timeoutgggggggg!'
            return
        # Other error
        except:
            print 'Some other error!'
        print('helooooo momi')
        #vehicle attributes (state)
        print("Get some vehicle attribute values:")
        print(" GPS: %s" % self.vehicle.gps_0)
        print(" Battery: %s" % self.vehicle.battery)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Mode: %s" % self.vehicle.mode.name)    # settable
        #arm_and_takeoff(vehicle)
        #Close vehicle object before exiting script
        print"end of connected drone"
        """
        while True:
            #print(vehicle.location.capabilities)
            #print("the latiiiiiii is:%s"% LocationGlobal)
            #print("theiiiiiiiii lon is:%s"% vehicle.location.global_frame.lon)
            print("the lat is:%s"% vehicle.location.global_frame.lat)
            print("the lon is:%s"% vehicle.location.global_frame.lon)
            time.sleep(4)
        """
    def connecting_sitl(self):
        drone_kit_sitl ='dronekit-sitl copter --home=31.7965240478516,35.3291511535645,248.839996337891,240'
        self.droneKitSitl_proc = subprocess.Popen(drone_kit_sitl,shell=True,stdin=PIPE,stdout=subprocess.PIPE)
        mavProxy_sitl_theard=threading.Thread(name='start mavProxy sitl',target=self.connection_mavProxy_sitl)
        mavProxy_sitl_theard.start()
        #os.system(droneKitSitl)
        print("after subprocess sitl")
        #time.sleep(3)
        #mavProxy_sitl_theard=threading.Thread(name='start mavProxy sitl',target=self.connection_mavProxy_sitl)
        #mavProxy_sitl_theard.start()
    def connection_mavProxy_sitl(self):
        time.sleep(4)
        mav_proxy_sitl ='mavproxy.py --master=tcp:127.0.0.1:5760 --out=udpout:10.1.49.130:14550 --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551'
        self.mavProxy_sitl_proc = subprocess.Popen(mav_proxy_sitl,shell=True,stdin=PIPE,stdout=subprocess.PIPE)
        #os.system(mavProxy_sitl)
        #sitl_theard_test_msg=threading.Thread(name='sitl msg',target=self.listen)
        #sitl_theard_test_msg.start()
        print("im in mavproxy sitl connection")
        #time.sleep(7)
        self.connecting_drone()
        #self.vehicle = connect(droneKitSitl,heartbeat_timeout=15)
        
    def sitl_disconnect(self):
        print("im in sitl disconeted")
        self.serch_pid_port=subprocess.Popen('netstat -ano | findstr :5760',shell=True,stdin=PIPE,stdout=subprocess.PIPE)
        #sitl_theard_test_msg=threading.Thread(name='sitl msg',target=self.listen)
        #sitl_theard_test_msg.start()
        #time.sleep(1)
        port_pid_task = self.serch_pid_port.stdout.readline().split(" ") #the line that pid of port 5760 is open
        if port_pid_task is not None:
            port_pid=port_pid_task[len(port_pid_task)-1]
            print(port_pid)
            subprocess.Popen('taskkill /F /T /PID ' + port_pid,shell=True)
        
        pid_mavProxy_sitl_proc=self.mavProxy_sitl_proc.pid
        print(pid_mavProxy_sitl_proc)
        subprocess.Popen('taskkill /F /T /PID %i' % pid_mavProxy_sitl_proc,shell=True)
        #pid_mavProxySitl=self.mavProxy_sitl_proc.pid
        #subprocess.Popen('taskkill /F /T /PID %i' % pid_mavProxySitl,shell=True)

    def listen(self):
       while True:     #read the msg from mavproxy
            time.sleep(1)
            line = self.mavProxy_sitl_proc.stdout.readline() #if the line = Saved 773 parmeters to mav.prm the mavProxy connect to 3DR.
            
            print(line)

    def person_detected(self):
        print" i in person detected"
        self.vehicle.mode = VehicleMode("GUIDED")


###############################################################################################################################
##################################################### Gui #####################################################################
class Gui:
    def __init__(self,master):      
        master.geometry("950x650")
        master.title("Rescue Drune")
        self.drone_vehicle=droneControl(self.show_msg_monitor)   
        #section the main frame to rows and columns.
        for x in xrange(5):
            master.grid_columnconfigure(x, weight=1)
        for y in xrange(5):    
            master.grid_rowconfigure(y, weight=1)
            
        #drone_control frames
        drone_control = Frame(master, bg='gray')
        
        for x in xrange(2):
            drone_control.grid_columnconfigure(x, weight=1)
        for y in xrange(4):    
            drone_control.grid_rowconfigure(y, weight=1)
            
        drone_control.grid(row=0 ,column=1,columnspan=5,sticky=W+N+E+S)
        
        #video_window frames
        self.video_window = Label(master,width=65,height=26,borderwidth=2, relief="groove",bg="gray")
        self.video_window.grid(row=0,column=0,sticky=W+N+E+S,padx=5, pady=5)
        
        #msg_drone frames
        indication_frame = Frame(master,height=200, bg='gray')
        for x in xrange(5):
            indication_frame.grid_columnconfigure(x, weight=1)
        for y in xrange(2):    
            indication_frame.grid_rowconfigure(y, weight=1)
        indication_frame.grid(row=1,column=0,columnspan=6,rowspan=5,sticky=W+N+E+S)

        #frame for msg from the drone and software
        self.monitor_msg=Text(indication_frame,width=30,background='black')
        self.monitor_msg.grid(row=0,column=0,sticky=W+N+S+E)
        scrollbar = Scrollbar(indication_frame)
        scrollbar.grid(row=0,column=1,sticky=W+N+S)
        self.monitor_msg.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.monitor_msg.yview)
        self.monitor_msg.tag_configure("success", foreground='#00B400')
        self.monitor_msg.tag_configure("error", foreground='#e60000')
        self.monitor_msg.tag_configure("person", foreground='#00C5CD')
        #monitor_msg.insert(END, 'default text')
        self.drone_is_connect=False #this bool to know if the system connected to the drone and video system
        self.sitl_is_connect=False #this bool to know if the system connected to the simulator
        
        """connect to drone"""
        self.button_connect=Button(drone_control,text="Connect",width=9, height=2,command=lambda:self.switch_on_off(master,'drone'))
        self.button_connect.grid(row=0,column=1,sticky=W+N,pady=4)
        
        """connect to SITL"""
        self.button_connect_sitl=Button(drone_control,text="Connect\nSITL",width=9, height=2, command=lambda:self.switch_on_off(master,'sitl'))
        self.button_connect_sitl.grid(row=0,column=0,sticky=W+N,padx=4,pady=4)

        """AUTO mode"""
        self.button_auto=Button(drone_control,text="Auto Search",width=9,height=3,command=self.send_auto_mode)
        self.button_auto.grid(row=1,column=0,columnspan=1,sticky=W+N,padx=4,pady=4)

        self.button_manual=Button(drone_control,text="Manual",width=9,height=3,command=self.ccc)
        self.button_manual.grid(row=1,column=1,columnspan=1,sticky=W+N,pady=4)
        
        self.button_rtl=Button(drone_control,text="RTL",width=9,height=3)
        self.button_rtl.grid(row=1,column=2,columnspan=1,sticky=W+N,padx=4,pady=4)


        
        
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        #self.show_msg_monitor(">> im innnnln lknkn lknlk nlknrw klfokwmf fwfwwffw aaaaaa bbbbbbbb n","success")
        #self.show_msg_monitor(">> and i in new line",'error')
        #self.show_msg_monitor(">> perso person",'person')
    def ccc(self):
        print("im here sgfsfgsfgsfgsgsfgsgsfsfggsf")
    #this function send to drone move to AUTO mode
    def send_auto_mode(self):
        self.auto = threading.Thread(name='drone connect',target=self.drone_vehicle.auto_mode)
        self.auto.start()

    def switch_on_off(self,master,key):
        if key =='drone' :
            if self.sitl_is_connect is False:
                if self.drone_is_connect is False:
                    print("is here")
                    self.button_connect.config(text="Disconnect")
                    self.drone_is_connect=True
                    self.drone_connect(key,master)
                else:
                    self.button_connect.config(text="Connect")
                    self.drone_is_connect=False
                    #self.video_window.destroy()
                    self.disconnect(key,master)  
            else:
                print("please disconnect from the SITL , and try again")
        else:
            if self.drone_is_connect is False:
                if self.sitl_is_connect is False:
                    self.button_connect_sitl.config(text="Disconnect\nSITL")
                    self.sitl_is_connect=True
                    self.drone_connect(key,master)
                else:
                    self.button_connect_sitl.config(text="ConnectSITL")
                    self.sitl_is_connect=False
                    #self.video_window.destroy()
                    self.disconnect(key,master)
            else:
                print("please disconnect from the drone , and try again")

    def drone_connect(self,key,master):  #connect to the system
        if key =='drone':
            #this apply the mavProxy and after mavProxy succeeded,the mavProxy connect the drone to system in drone_control
            self.connecting_drone_thread = threading.Thread(name='connect_to_drone_thread',target=self.drone_vehicle.mav_proxy_connect)
            self.connecting_drone_thread.start()
            
        else:
            self.connecting_sitl_thread = threading.Thread(name='connect_to_sitl_thread',target=self.drone_vehicle.connecting_sitl())
            self.connecting_sitl_thread.start()
        self.detection_obj=person_detection(self.drone_vehicle)
        self.person_detection_video = threading.Thread(name='person_detection_video',target=lambda:self.cam_drone(master))
        self.person_detection_video.start()
        
    def disconnect(self,key,master):    #disconnect from button
        if key == 'drone':
            self.drone_vehicle.drone_disconnect()
            self.drone_is_connect=False   #change the bool drone to false,is mean that drone now is not connected
            
        elif key == 'sitl':
            self.drone_vehicle.sitl_disconnect()
            self.sitl_is_connect=False    #change the bool sitl to false,is mean that sitl now is not connected
            
        self.detection_obj.close_detection()                        
        self.video_window.destroy()
        self.video_window = Label(master,width=65,height=26,borderwidth=2, relief="groove",bg="gray")
        self.video_window.grid(row=0,column=0,sticky=W+N+E+S,padx=5, pady=5)
        self.socket_video.close()    
        
    def on_closing(self):   #when the user close the window on X
        if self.drone_is_connect is True:
            self.drone_is_connect=False
            self.drone_vehicle.drone_disconnect()
        elif self.sitl_is_connect is True:
            self.sitl_is_connect =False
            self.drone_vehicle.sitl_disconnect()       
        else:
            root.destroy()
            return
        self.detection_obj.close_detection()
        time.sleep(1)
        root.destroy() 
            
            
    def cam_drone(self,master): #master??
        self.socket_video=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket_video.settimeout(15)
        HOST=''
        PORT=8089
        print("conecting to person detection")
        try:
            self.socket_video.bind((HOST,PORT))
            print 'Socket bind complete'
            self.socket_video.listen(10)
            print 'Socket now listening'
            conn,addr=self.socket_video.accept()
            conn.setblocking(1)
        except socket.error, exc:
            print("error,problem video connect: %s" % exc)
            if self.drone_is_connect == True:
                self.disconnect('drone',master)
            elif self.sitl_is_connect == True :
                self.disconnect('sitl',master)
            sys.exit(1)
        
        data = ""
        payload_size = struct.calcsize("L")
        #time.sleep(5)
        #self.vedeo=Label(self.video_window)
        #self.vedeo.grid()
        #self.video_window.config(width=500,height=450)
        #self.video_window.destroy()
        #self.video = Label(master,borderwidth=2, relief="groove")
        #self.video.grid(row=0,column=0,sticky=W+N+E+S,padx=5, pady=5)
        open_label=False
        while self.drone_is_connect or self.sitl_is_connect:
            while len(data) < payload_size:
                data += conn.recv(4096)
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            while len(data) < msg_size:
                data += conn.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            if open_label is False:
                self.video_window.config(width=460,height=400)
                open_label=True
            last_frame=pickle.loads(frame_data)
            last_frame = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(last_frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_window.imgtk = imgtk
            self.video_window.configure(image=imgtk)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

    
    def show_msg_monitor(self,msg,tag):
        self.monitor_msg.config(state='normal')
        self.monitor_msg.insert(END, msg + "\n",tag)
        self.monitor_msg.config(state='disabled')
        
        
     
###############################################################################################################################
##################################################### main ####################################################################
if __name__ == "__main__":
    root =Tk()

    gui = Gui(root)
    
    
    root.mainloop()
    print("after person detection")
    sys.exit()
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
