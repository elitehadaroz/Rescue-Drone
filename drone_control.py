# -*- coding: cp1255 -*-

import exceptions
import pickle
import socket
import struct
import subprocess
import threading
import time
from Tkinter import *
import ttk
from mttkinter import mtTkinter
from subprocess import PIPE
from multiprocessing import Process
import tkMessageBox
import cv2
from PIL import Image, ImageTk
from dronekit import connect, VehicleMode, APIException  # Import DroneKit-Python
from winsound import *


############################################# person detection ##############################################################
class PersonDetection:
    def __init__(self, drone_vehicle_obj,gui_obj):
        self.gui = gui_obj
        self.drone_vehicle = drone_vehicle_obj
        self.listener = True
        print ('Socket created')
        print ('detection process')
        person_detection = "py person_detection.py"  # launch the person_detection script using bash
        self.person_detection_process = subprocess.Popen(person_detection, shell=True, stdout=subprocess.PIPE)

        # msg thread

        self.person_detection_msg = threading.Thread(name='person_detection_msg', target=self.detection_msg)
        self.person_detection_msg.start()

    def close_detection(self):
        self.listener = False
        self.person_detection_msg.join()
        self.person_detection_process.kill()


    def detection_msg(self):
        while self.listener:
            msg_detection = self.person_detection_process.stdout.readline().rstrip('\n')
            if not msg_detection:
                print "im in breack"
                break
            if msg_detection is not None:
                try:
                    self.drone_vehicle.person_detected()
                except:
                    continue


###############################################################################################################################
################################################ Drone Control ################################################################

class DroneControl:
    def __init__(self, gui_obj):
        self.mavlink_connected = False
        self.gui = gui_obj
        self.mavlink_time_out = False
        self.mavlink_proc=None  #this proc start the mavProxy for real drone
        self.mavProxy_sitl_proc=None #this proc start simulator mavProxy mavlink
        self.vehicle=None
        self.stop_timer=None    #this boolean stop_timer is count X second to login option, if is pass X second the system stop trying to connect
        self.drone_connected=False
        self.person_is_detect = False
        self.cam_connect = False
        self.is_armed = False

    def mav_proxy_connect(self):
        self.gui.show_msg_monitor(">> start to connect mavProxy,please wait...", "msg")
        self.mavlink_time_out = False  # 120s to chance connect mavproxy
        self.cam_connect = True
        mav_proxy = 'mavproxy.py --master="COM4" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        # droneKitSitl ='127.0.0.1:14549'
        self.mavlink_proc = subprocess.Popen(mav_proxy, shell=True, stdin=PIPE, stdout=subprocess.PIPE)

        time.sleep(1)  ################
        print("is in mavproxy")
        msg_from_mavlink = threading.Thread(name='mavlink_msg',target=lambda: self.mavlink_msg())  # start to read msg from mavProxy
        msg_from_mavlink.start()
        while self.mavlink_connected is False:  # wait to connected
            if self.mavlink_time_out is True:  # wait only 120s to connected
                break

        if self.mavlink_connected is True:  # the connected succeeded
            print("is need to connect")
            self.gui.show_msg_monitor(">> mavProxy is connected", "success")
            self.connecting_drone()
        else:  # the connected not succeed
            self.cam_connect = False
            self.gui.show_msg_monitor(">> error: problem with connect to mavProxy,(120 sec pass)", "error")
            print("error mavlink connect")

    def mavlink_msg(self):
        self.stop_timer = False
        time_mavlink_connect = threading.Thread(name='wait_to_connect',target=self.timer_connect_mavproxy)  #start timer for 120sec
        time_mavlink_connect.start()

        while self.mavlink_time_out is False:  # read the msg from mavproxy
            print("close thread mavlink wait")
            read_connect = self.mavlink_proc.stdout.readline().split(" ")[
                0]  # if the line = Saved 773 parameters to mav.prm the mavProxy connect to 3DR.
            print(read_connect)
            if str(read_connect) == "Failed":
                #self.drone_disconnect()  # kill the subprocess before exit or connect again
                break
            if str(read_connect) == "Saved":  # succeeded
                print(read_connect)
                print("close theard mavlink wait")
                self.stop_timer = True
                self.mavlink_connected = True
                break

    def timer_connect_mavproxy(self):  # this function is timer to connect mavProxy,if is not connected after 120s is stop the connected.
        sec = 0
        while self.stop_timer is False:
            sec += 1
            if sec == 20:
                self.mavlink_time_out = True
                break
            time.sleep(1)

    def drone_disconnect(self):
        self.gui.show_msg_monitor(">> Disconnects from the drone...", "msg")
        self.mavlink_time_out = True  # reset the timer to connection
        # gui.drone_is_connect=False                    #to reset the option to connect again
        # gui.button_connect.config(text="Connect")   #to reset the name button to connect again
        self.drone_connected = False
        if self.mavlink_proc is not None:
            pid = self.mavlink_proc.pid
            print("mavlink proc kill is",pid)
            subprocess.Popen('taskkill /F /T /PID %i' % pid, shell=True)
            self.gui.show_msg_monitor(">> Drone is disconnected", "msg")

    def auto_mode(self):
        #cmd = self.vehicle.commands
        #cmd.wait_valid()
        self.gui.show_msg_monitor(">> AUTO mode activated", "msg")
        self.arm_and_takeoff(20)
        self.gui.show_msg_monitor(">> The drone begins the mission", "msg")
        self.vehicle.mode = VehicleMode("AUTO")
        print"after switch auto mode in drone"
        print(cmd.count)


    def arm_and_takeoff(self, target_altitude):  # Arms vehicle and fly to target_altitude.
        #self.gui.show_msg_monitor(">> Drone takeoff to " + target_altitude + "meter altitude...", "msg")
        if self.vehicle.armed is False:
            print("Basic pre-arm checks")
            # Don't try to arm until autopilot is ready
            while not self.vehicle.is_armable:
                self.gui.show_msg_monitor(">> Waiting for vehicle to initialise...", "msg")
                time.sleep(1)

            print("Arming motors")
            # Copter should arm in GUIDED mode
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True

            # Confirm vehicle armed before attempting to take off
            while not self.vehicle.armed:
                self.gui.show_msg_monitor(">> Waiting for arming...", "msg")
                time.sleep(1)
            self.is_armed = True
            print("Taking off!")
            self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

            # Wait until the vehicle reaches a safe height before processing the goto
            #  (otherwise the command after self.vehicle.simple_takeoff will execute
            #   immediately).
            while True:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                    self.gui.show_msg_monitor(">> Reached target altitude", "success")
                    break
                time.sleep(1)
        else:
            self.gui.show_msg_monitor(">> the copter is armed", "msg")
            #print("the copter is armed")

    def connecting_drone(self):
        self.gui.show_msg_monitor(">> drone connecting...", "msg")
        # Connect to the Vehicle.
        #print "Connecting to vehicle on: udp:127.0.0.1:14551"
        try:
            # connecting to the vehicle by udp- this we can also open the massion planner with the python
            self.vehicle = connect("127.0.0.1:14551", wait_ready=False, baud=57600)
            # wait_ready is for that all info from drone upload 100%
            self.vehicle.wait_ready(True, timeout=60)
            # print("the vehicle is ready")
        # Bad TCP connection
        except socket.error:
            print 'No server exists!'

        # Bad TTY connection
        except exceptions.OSError as e:
            print 'No serial exists!'

        # API Error
        except APIException:
            print 'the Time is out!'
            return
        # Other error
        except:
            print 'Some other error!'

        # vehicle attributes (state)
        # this line is for test!!!!!!!!!!!!!!!!!!
        print("Get some vehicle attribute values:")
        print(" GPS: %s" % self.vehicle.gps_0)
        print(" Battery: %s" % self.vehicle.battery)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Mode: %s" % self.vehicle.mode.name)  # settable
        # arm_and_takeoff(vehicle)
        # Close vehicle object before exiting script
        self.vehicle.mode = VehicleMode("GUIDED")
        self.drone_connected=True
        print"the drone is connected"
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
        self.cam_connect = True
        drone_kit_sitl = 'dronekit-sitl copter --home=31.7965240478516,35.3291511535645,248.839996337891,240'
        self.dronekit_process = subprocess.Popen(drone_kit_sitl, shell=True, stdin=PIPE, stdout=subprocess.PIPE)
        print("self.dronekit_process",self.dronekit_process.pid)
        mavproxy_sitl_thread = threading.Thread(name='start mavProxy sitl', target=self.connection_mavproxy_sitl)
        mavproxy_sitl_thread.start()

    def connection_mavproxy_sitl(self):
        time.sleep(4)
        mav_proxy_sitl = 'mavproxy.py --master=tcp:127.0.0.1:5760 --out=udpout:10.1.49.130:14550 --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551 '
        self.mavProxy_sitl_proc = subprocess.Popen(mav_proxy_sitl, shell=True, stdin=PIPE, stdout=subprocess.PIPE)
        print("self.dronekit_process", self.mavProxy_sitl_proc.pid)
        print("im in mavproxy sitl connection")
        time.sleep(3)
        self.connecting_drone()


    def sitl_disconnect(self):
        print('im in sitl disconnected')
        self.cam_connect = False
        self.drone_connected = False
        self.dronekit_process.kill()
        search_pid_port = subprocess.Popen('netstat -ano | findstr :5760', shell=True, stdin=PIPE,stdout=subprocess.PIPE)

        port_pid_task = search_pid_port.stdout.readline().split(" ")  # the line that pid of port 5760 is open
        if port_pid_task is not None:
            port_pid = port_pid_task[len(port_pid_task) - 1]
            print(port_pid)
            subprocess.Popen('taskkill /F /T /PID ' + port_pid, shell=True)

        if self.mavProxy_sitl_proc is not None:
            print("kill the self.dronekit_process", self.mavProxy_sitl_proc.pid)
            pid_mavproxy_sitl_proc = self.mavProxy_sitl_proc.pid
            print(pid_mavproxy_sitl_proc)
            subprocess.Popen('taskkill /F /T /PID %i' % pid_mavproxy_sitl_proc, shell=True)



    def person_detected(self):
        if not self.person_is_detect:
            if self.vehicle.mode.name == 'AUTO':
                self.person_is_detect = True
                self.gui.show_msg_user("person detection")

    def info_drone(self):
        info = {'alt': self.vehicle.location.global_relative_frame.alt,
                'ground_speed':self.vehicle.groundspeed,
                'dist': self.vehicle.rangefinder.distance,
                'bat': self.vehicle.battery }
        return info
###############################################################################################################################
##################################################### Gui #####################################################################
###############################################################################################################################

class Gui:
    def __init__(self, master):
        master.geometry("950x650")
        master.title("Rescue Drone")
        self.detection_obj=None #this Obj its person detection class
        self.socket_video=None  #the socket_video assigned to receive video from person_detection file.
        self.message_box_pop = False
        self.message_box = None   #this LabelFrame create when have message from system\person detection etc..
        self.drone_vehicle = DroneControl(self)
        self.drone_control = None
        self.video_window = None
        self.monitor_msg = None

        self.drone_is_connect = False  # this bool to know if the system connected to the drone and video system
        self.sitl_is_connect = False  # this bool to know if the system connected to the simulator

        # create the main frame to rows and columns.
        for x in xrange(5):
            master.grid_rowconfigure(x, weight=1)
        for y in xrange(5):
            master.grid_columnconfigure(y, weight=1)



        # create panel for control drone
        self.create_drone_control_panel(master)
        # create panel for video from drone
        self.create_video_panel(master)
        # msg_drone frames
        self.create_monitor_msg(master)
        # monitor_msg.insert(END, 'default text')

        ########################################################################################

        self.show_msg_monitor(">> Welcome to Rescue Drone software", 'msg')

        master.protocol("WM_DELETE_WINDOW", lambda:self.on_closing(master))
        # self.show_msg_monitor(">> im innnnln lknkn lknlk nlknrw klfokwmf fwfwwffw aaaaaa bbbbbbbb n","success")
        # self.show_msg_monitor(">> and i in new line",'error')
        # self.show_msg_monitor(">> perso person",'person')

    def create_video_panel(self,master):
        self.video_window = Label(master, width=65, height=26, borderwidth=2, relief="groove", bg="gray")
        self.video_window.grid(row=0, column=0, sticky=W + N + E + S, padx=5, pady=5)

    def create_drone_control_panel(self,master):
        # drone_control frames
        self.drone_control = Frame(master, bg='gray')

        for x in xrange(4):
            self.drone_control.grid_rowconfigure(x, weight=1)
        for y in xrange(2):
            self.drone_control.grid_columnconfigure(y, weight=1)


        self.drone_control.grid(row=0, column=1, columnspan=5, sticky=W + N + E + S)

        """button connect to drone"""
        self.button_connect = Button(self.drone_control, text="Connect", width=9, height=2,
                                     command=lambda: self.switch_on_off(master, 'drone'))
        self.button_connect.grid(row=0, column=1, sticky=W + N, pady=4)

        """button connect to SITL"""
        self.button_connect_sitl = Button(self.drone_control, text="Connect\nSITL", width=9, height=2,
                                          command=lambda: self.switch_on_off(master, 'sitl'))
        self.button_connect_sitl.grid(row=0, column=0, sticky=W + N, padx=4, pady=4)

        """button AUTO mode"""
        self.button_auto = Button(self.drone_control, state=DISABLED, text="Auto Search", width=9, height=3,
                                  command=self.send_auto_mode)
        self.button_auto.grid(row=1, column=0, columnspan=1, sticky=W + N, padx=4, pady=4)

        """button MANUAL mode"""
        self.button_manual = Button(self.drone_control, state=DISABLED, text="Manual", width=9, height=3)
        self.button_manual.grid(row=1, column=1, columnspan=1, sticky=W + N, pady=4)

        """button RTL mode"""
        self.button_rtl = Button(self.drone_control, state=DISABLED, text="RTL", width=9, height=3)
        self.button_rtl.grid(row=1, column=2, columnspan=1, sticky=W + N, padx=4, pady=4)

    def create_monitor_msg(self,master):
        indication_frame = Frame(master, height=200, bg='gray')
        for x in xrange(2):
            indication_frame.grid_rowconfigure(x, weight=1)
        for y in xrange(5):
            indication_frame.grid_columnconfigure(y, weight=1)

        indication_frame.grid(row=1, column=0, columnspan=6, rowspan=5, sticky=W + N + E + S)

        # frame for msg from the drone and software
        self.monitor_msg = Text(indication_frame, width=30, background='black')
        self.monitor_msg.grid(row=0, column=0, sticky=W + N + S + E)
        scrollbar = Scrollbar(indication_frame)
        scrollbar.grid(row=0, column=0, sticky=E + N + S)
        self.monitor_msg.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.monitor_msg.yview)
        self.monitor_msg.tag_configure("success", foreground='#00B400')
        self.monitor_msg.tag_configure("error", foreground='#e60000')
        self.monitor_msg.tag_configure("person", foreground='#00C5CD')
        self.monitor_msg.tag_configure("msg", foreground='#ffffff')
        self.create_info_panel(indication_frame)

    def create_info_panel(self,indication_frame):
        self.info = Frame(indication_frame, bg='#282828')
        for x in xrange(10):
            indication_frame.grid_rowconfigure(x, weight=1)
        for y in xrange(5):
            indication_frame.grid_columnconfigure(y, weight=1)

        self.info.grid(row=0, column=1, columnspan=4, sticky=W + N + S + E)

        self.altitude_label = Label(self.info, text="Altitude (m)",font=('Arial', 10), fg="white",bg='#282828')
        self.altitude_label.grid(row=0, column=0,padx=10)
        self.altitude_info = Label(self.info, text="0.00",font=('Arial', 20), fg="#BA55D3", bg='#282828')
        self.altitude_info.grid(row=1,column=0,padx=10)

        self.ground_speed_label = Label(self.info, text="Ground Speed (m/s)", font=('Arial', 10), fg="white", bg='#282828')
        self.ground_speed_label.grid(row=2, column=0, padx=10)
        self.ground_speed_info = Label(self.info, text="0.00", font=('Arial', 20), fg="#00FF00", bg='#282828')
        self.ground_speed_info.grid(row=3, column=0, padx=10)

        self.dist_to_home_label = Label(self.info, text="Dist to Home (m)", font=('Arial', 10), fg="white",bg='#282828')
        self.dist_to_home_label.grid(row=4, column=0, padx=10)
        self.dist_to_home_info = Label(self.info, text="0.00", font=('Arial', 20), fg="#00FFFF", bg='#282828')
        self.dist_to_home_info.grid(row=5, column=0, padx=10)

        self.bat_volt_label = Label(self.info, text="Bat Voltage (V)", font=('Arial', 10), fg="white",bg='#282828')
        self.bat_volt_label.grid(row=6, column=0, padx=10)
        self.bat_volt_info = Label(self.info, text="0.00", font=('Arial', 20), fg="#FFD700", bg='#282828')
        self.bat_volt_info.grid(row=7, column=0, padx=10)

    # this function send to drone move to AUTO mode
    def send_auto_mode(self):
        auto = threading.Thread(name='drone connect', target=self.drone_vehicle.auto_mode)
        auto.start()
        print("gui auto mode end")
    def allow_button(self):
        self.button_auto.config(state=NORMAL)
        self.button_manual.config(state=NORMAL)
        self.button_rtl.config(state=NORMAL)

    def switch_on_off(self, master, key):
        if self.drone_vehicle.is_armed is False:
            if key == 'drone':
                if self.sitl_is_connect is False:
                    if self.drone_is_connect is False:
                        self.button_connect.config(text="Disconnect")
                        self.button_connect_sitl.config(state=DISABLED)
                        self.drone_is_connect = True
                        self.drone_connect(key, master)
                    else:
                        self.button_connect.config(text="Connect")
                        self.button_connect_sitl.config(state=NORMAL)
                        self.drone_is_connect = False
                        disconnect_thread = threading.Thread(name='disconnect from drone',
                                                             target=lambda: self.disconnect(key, master))
                        disconnect_thread.start()
                else:
                    print("please disconnect from the SITL , and try again")
            else:
                if self.drone_is_connect is False:
                    if self.sitl_is_connect is False:
                        self.button_connect_sitl.config(text="Disconnect\nSITL")
                        self.button_connect.config(state=DISABLED)
                        self.sitl_is_connect = True
                        self.drone_connect(key, master)
                    else:
                        self.button_connect_sitl.config(text="ConnectSITL")
                        self.button_connect.config(state=NORMAL)
                        self.sitl_is_connect = False
                        print(key)
                        disconnect_thread = threading.Thread(name='disconnect from sitl',target=lambda:self.disconnect(key,master))
                        disconnect_thread.start()
                else:
                    print("please disconnect from the drone , and try again")
        else:
            self.show_msg_monitor(">> Please land and try again!", 'msg')
    def drone_connect(self, key, master):  # connect to the system
        if key == 'drone':
            # this apply the mavProxy and after mavProxy succeeded,the mavProxy connecting the drone to the system in drone_control
            connecting_drone_thread = threading.Thread(name='connect_to_drone_thread',target=self.drone_vehicle.mav_proxy_connect)
            connecting_drone_thread.start()
        else:
            connecting_sitl_thread = threading.Thread(name='connect_to_sitl_thread',target=self.drone_vehicle.connecting_sitl())
            connecting_sitl_thread.start()

        person_detection_video = threading.Thread(name='cam_drone',target=lambda: self.cam_drone(master,key))
        person_detection_video.start()
        self.allow_button()

    def disconnect(self, key, master):  # disconnect from button
        if key == 'drone':
            self.drone_vehicle.drone_disconnect()
            self.drone_is_connect = False  # change the bool drone to false,is mean that drone now is not connected

        elif key == 'sitl':
            self.drone_vehicle.sitl_disconnect()
            self.sitl_is_connect = False  # change the bool sitl to false,is mean that sitl now is not connected

        if self.message_box:
            self.message_box.destroy()    #if the message box open destroy it.
            self.message_box_pop = False  #reset the bool message box.
        print("close camera")
        self.detection_obj.close_detection()
        self.video_window.destroy()
        self.video_window = Label(master, width=65, height=26, borderwidth=2, relief="groove", bg="gray")
        self.video_window.grid(row=0, column=0, sticky=W + N + E + S, padx=5, pady=5)
        self.socket_video.close()

    def on_closing(self,master):
        if self.drone_vehicle.is_armed is False:
            #if self.drone_vehicle.drone_connected is True:
            if self.drone_is_connect is True :
                print("close drone")
                self.switch_on_off(master,'drone')
                root.destroy()
            elif self.sitl_is_connect is True:
                print("close sitl")
                self.switch_on_off(master,'sitl')
                root.destroy()
            else:
                print("im hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
                root.destroy()
                return
        else:
            print("please disconnect from the drone , and try again")
    def cam_drone(self, master,key):  # master??

        self.detection_obj = PersonDetection(self.drone_vehicle, self)
        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_video.settimeout(15)
        host = ''
        port = 8089
        print("connecting to person detection")
        try:
            self.socket_video.bind((host, port))
            print 'Socket bind complete'
            self.socket_video.listen(15)
            print 'Socket now listening'
            conn, addr = self.socket_video.accept()
            conn.setblocking(1)
        except socket.error, exc:
            print("error,problem video connect: %s" % exc)
            if self.drone_is_connect:
                self.disconnect('drone', master)
            elif self.sitl_is_connect:
                self.disconnect('sitl', master)
            sys.exit(1)

        data = ""
        payload_size = struct.calcsize("L")
        open_label = False
        while self.drone_is_connect or self.sitl_is_connect:
            if self.drone_vehicle.cam_connect: # if the drone is not connected we will not continue to video
                while len(data) < payload_size:
                    data += conn.recv(4096)
                if not data:
                    print("helooooo word")
                    self.socket_video.close()
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("L", packed_msg_size)[0]
                while len(data) < msg_size:
                    data += conn.recv(4096)
                frame_data = data[:msg_size]
                data = data[msg_size:]

                if not open_label:
                    self.video_window.config(width=460, height=400)
                    open_label = True
                last_frame = pickle.loads(frame_data)
                last_frame = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(last_frame)
                imgtk = ImageTk.PhotoImage(image=img)
                self.video_window.imgtk = imgtk
                self.video_window.configure(image=imgtk)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            else:   #if there was no connection to the drone, turn off the video and disconnect from all.
                self.switch_on_off(master,key)
                break
    def show_msg_monitor(self, msg, tag):
        self.monitor_msg.config(state='normal')
        self.monitor_msg.insert(END, msg + "\n", tag)
        self.monitor_msg.config(state='disabled')


    def show_msg_user(self,key):

        if not self.message_box_pop:
            self.message_box_pop = True

            if key == "person detection":
                text_message = "A person has been detected !\n do you want to continue search or send GPS location and RTL"
                yes_button = "send GPS and RTL"
                no_button = "continue to search"

                self.create_message_box(text_message,yes_button,no_button)
                sound_bool = BooleanVar()

                start_alarm = threading.Thread(name="start_the_alarm_person_detect", target=lambda: self.start_alarm(sound_bool))
                start_alarm.start()
                """
                change_color_title_msg = threading.Thread(name="change the title color msg",target=self.change_color)
                change_color_title_msg.start()
                """
                button_sound_on_oof = Checkbutton(self.message_box, text="sound on/off", variable=sound_bool)
                button_sound_on_oof.grid(row=1, column=0)

    def create_message_box(self,text,yes,no):
        self.message_box = LabelFrame(self.drone_control, fg='red', text="!! Message !!", font=("Courier", 15),
                                      labelanchor=N)
        for x in xrange(5):
            self.message_box.grid_columnconfigure(x, weight=1)
        for y in xrange(2):
            self.message_box.grid_rowconfigure(y, weight=1)
        self.message_box.grid(row=2, column=0, columnspan=3, rowspan=5, sticky=W + N + E + S)

        message = Label(self.message_box, text=text,font=10)
        message.grid(row=0, columnspan=5)

        button_yes = Button(self.message_box, text=yes, width=15, height=2,
                           command=lambda: self.user_reply_message('ok'))
        button_yes.grid(row=1, column=1)

        button_no = Button(self.message_box, text=no, width=15, height=2,
                           command=lambda: self.user_reply_message('no'))
        button_no.grid(row=1, column=3)

    def update_parm_drone(self):

        while self.drone_vehicle.drone_connected:
            info =self.drone_vehicle.info_drone()
            self.altitude_info.config(text=info['alt'])
            self.bat_volt_info.config(text=info['ground_speed'])
            self.dist_to_home_info.config(text=info['dist'])
            self.ground_speed_info.config(text=info['bat'])
    #@staticmethod

    """the function call from show_msg_user when detection a person."""
    def start_alarm (self,sound_bool):    #the function call from show_msg_user and only on/off alarm when person detect
        while self.message_box_pop:
            if sound_bool.get() is False:
                print sound_bool.get()
                PlaySound('media\Alarm.wav', SND_FILENAME)
                time.sleep(2)
            else:
                continue
"""
    def change_color(self):

        while self.message_box_pop:
            print "i in change color"
            self.message_box.configure( fg='red' )
            #*.style().configure(self.message_box,fg='white')
            time.sleep(1)
            self.message_box.configure( fg='white' )
"""
###############################################################################################################################
##################################################### main ####################################################################
if __name__ == "__main__":
    root = Tk()

    gui = Gui(root)

    root.mainloop()
    print("after person detection")
    sys.exit()
"""
def get_distance_metres(aLocation1, aLocation2):
    
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
"""
