import threading
import subprocess
from subprocess import PIPE
import socket
import time
from pymavlink import mavutil, mavwp
from dronekit import connect, VehicleMode, APIException ,Command  # Import DroneKit-Python
import exceptions
import math

class DroneControl:
    def __init__(self, gui_obj,repo_obj,setting_obj):
        self.mavlink_connected = False
        self.gui = gui_obj
        self.setting = setting_obj
        self.report = repo_obj
        self.mavlink_time_out = False
        self.mavlink_proc = None  #this proc start the mavProxy for real drone
        self.mavProxy_sitl_proc = None #this proc start simulator mavProxy mavlink
        self.vehicle = None
        self.stop_timer=None    #this boolean stop_timer is count X second to login option, if is pass X second the system stop trying to connect
        self.drone_connected=False
        self.person_is_detect = False
        self.cam_connect = True
        self.command_mission = None
        self.auto_mode_activated = False
        self.__home_loc = None
        self.__person_location = None
        self.__insert_end_mission = False

    #connect with mavproxy and split the data
    def mav_proxy_connect(self):
        self.gui.show_msg_monitor(">> start to connect mavProxy,please wait...", "msg")
        self.mavlink_time_out = False  # 120s to chance connect mavproxy
        #connect to mavproxy to master usb and split to two udp port
        mav_proxy = 'mavproxy.py --master="'+self.setting.get_usb_com()+'" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        #mav_proxy = 'mavproxy.py --master="COM4" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        self.mavlink_proc = subprocess.Popen(mav_proxy, shell=True, stdin=PIPE, stdout=subprocess.PIPE)

        time.sleep(1)

        msg_from_mavlink = threading.Thread(name='mavlink_msg',target=lambda: self.mavlink_msg())  # start to read msg from mavProxy
        msg_from_mavlink.start()
        while self.mavlink_connected is False:  # wait to connected
            if self.mavlink_time_out is True:  # wait only 120s to connected
                break

        if self.mavlink_connected is True:  # the connected succeeded
            self.gui.show_msg_monitor(">> mavProxy is connected", "success")
            self.connecting_drone('drone')
        else:  # the connected not succeed
            self.cam_connect = False
            self.mavlink_connected = False
            self.gui.show_msg_monitor(">> error: problem with connect to mavProxy,(120 sec pass)", "error")

    #start the listener to mavproxy msg if is connect or not.
    def mavlink_msg(self):
        self.stop_timer = False
        time_mavlink_connect = threading.Thread(name='wait_to_connect',target=self.timer_connect_mavproxy)  #start timer for 120sec
        time_mavlink_connect.start()

        while self.mavlink_time_out is False:  # read the msg from mavproxy
            read_connect = self.mavlink_proc.stdout.readline().split(" ")[0]  # if the line = Saved 773 parameters to mav.prm the mavProxy connect to 3DR.
            if str(read_connect) == "Failed":
                break
            if str(read_connect) == "Saved":  # succeeded
                self.stop_timer = True
                self.mavlink_connected = True
                break

    #crete time to connect to mavlink if the time pass the X sec,have a problem
    def timer_connect_mavproxy(self):  # this function is timer to connect mavProxy,if is not connected after 120s is stop the connected.
        sec = 0
        while self.stop_timer is False:
            sec += 1
            if sec == 100:
                self.mavlink_time_out = True
                break
            time.sleep(1)
        print(sec)

    #disconnect from vehicle and close process
    def drone_disconnect(self):
        self.mavlink_connected = False
        if self.vehicle is not None:
            self.vehicle.close()
        self.gui.show_msg_monitor(">> Disconnects from the drone...", "msg")
        self.mavlink_time_out = False  # reset the timer to connection
        self.cam_connect = False
        self.drone_connected = False
        if self.mavlink_proc is not None: #close mavlink process
            pid = self.mavlink_proc.pid
            subprocess.Popen('taskkill /F /T /PID %i' % pid, shell=True)
            self.gui.show_msg_monitor(">> Drone is disconnected", "msg")

    # set GUIDED mode,the drone now in GUIDED mode
    def loiter_mode(self):
        self.vehicle.mode = VehicleMode("LOITER")
        self.gui.show_msg_monitor(">> LOITER mode activated", "msg")
        self.vehicle.flush()

    def manual_mode(self):
        #self.vehicle.mode = VehicleMode("LOITER")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.gui.show_msg_monitor(">> GUIDED mode activated", "msg")
        self.vehicle.flush()

    # set RTL mode,the drone now in RTL mode
    def rtl_mode(self):
        self.auto_mode_activated = False
        self.manual_mode()
        self.gui.show_msg_monitor(">> 6 sec to RTL mode...", "msg")
        self.__home_loc.alt = self.setting.get_altitude()
        self.vehicle.simple_goto(self.__home_loc)
        self.vehicle.flush()
        time.sleep(6)
        self.vehicle.mode = VehicleMode("RTL")
        self.gui.show_msg_monitor(">> RTL mode activated", "msg")
        self.vehicle.flush()

    # set STABILIZE mode,the drone now in STABILIZE mode
    def stabilize_mode(self):
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.gui.show_msg_monitor(">> STABILIZE mode activated", "msg")
        self.vehicle.flush()

    #this function call from key function in GUI class. the function send msg to drone move right move left up down etc.
    def set_velocity_body(self, vx, vy, vz,yaw,command=''):

        if command == 'yaw':
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                0,  # confirmation
                2,  # param 1, yaw in degrees
                0,  # param 2, yaw speed deg/s
                yaw,  # param 3, direction -1 ccw, 1 cw
                1,  # param 4, relative offset 1, absolute angle 0
                0, 0, 0)  # param 5 ~ 7 not used

        else:   # the other command movement drone, pitch, roll,etc.
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # -- BITMASK -> Consider only the velocities
                0, 0, 0,  # -- POSITION
                vx, vy, vz,  # -- VELOCITY
                0, 0, 0,  # -- ACCELERATIONS
                0, yaw)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_gps_and_rtl(self):
        """need to write function that send gps to server!!!!!!"""
        self.gui.get_image = True  # get picture of person detected
        #info =["person location:","lon:",self.__home_loc.lon,"lat:",self.__home_loc.lat]
        #self.report.set_csv_on_report(info)
        self.rtl_mode()


    def send_gps_and_stay(self):
        """need to write function that send gps to server!!!!!!"""
        self.gui.get_image = True  # get picture of person detected
        self.manual_mode()
        self.vehicle.flush()


    #return the person detection location
    def get_person_location(self):
        return self.__person_location

    # after the user insert track on mission planner,this function read the track and save is on command_mission,call from auto_mode function
    def download_mission(self):
        if self.command_mission is None :
            self.command_mission = self.vehicle.commands
            self.command_mission.download()
            self.command_mission.wait_ready()
            while not self.vehicle.home_location:
                self.command_mission = self.vehicle.commands
                self.command_mission.download()
                self.command_mission.wait_ready()
                if self.command_mission.count != 0:
                    self.gui.show_msg_monitor(">> Waiting for home location and download mission...", "msg")
                if not self.vehicle.home_location:
                    time.sleep(1)
            if self.command_mission is not None:
                if self.vehicle.home_location and self.command_mission.count != 0:
                    self.gui.show_msg_monitor(">> Mission download success ", "success")
                    self.gui.show_msg_monitor(">> Home location saved ", "success")
                    self.vehicle.flush()
                    self.command_mission.upload()
                if self.command_mission is not None:
                    if self.command_mission.count == 0:
                        self.command_mission = None

    #the function read waypoint from mission list thet create in setting class,by upload mission file.and now the function upload to drone. call from auto_mode function
    def upload_mission(self):
        self.gui.show_msg_monitor(">> Start to upload mission...", "msg")
        mission_list = self.setting.get_missionlist()
        self.command_mission = self.vehicle.commands
        self.command_mission.clear()
        # Add new mission to vehicle
        for command in mission_list:
            self.command_mission.add(command)
        self.gui.show_msg_monitor(">> Upload mission success", "success")
        self.vehicle.commands.upload()

    #clean all mission from drone
    def clean_missions(self):
        if self.command_mission is not None:
            self.__insert_end_mission = False
            self.command_mission.clear()
            self.vehicle.flush()
            self.command_mission = None
        self.gui.show_msg_monitor(">> Clean mission ", "success")

    #the function switch the drone to AUTO mode.the function call from send_auto_mode in GUI class.
    def auto_mode(self):        #set AUTO mode,the drone now in AUTO mode
        if self.drone_connected is True:
            if self.vehicle.mode.name is not 'AUTO':
                if self.auto_mode_activated is False:
                    if self.command_mission is None: #
                        if self.setting.get_missionlist() is not None:
                            self.upload_mission()
                        else:
                            self.download_mission()
                    if self.command_mission is not None:
                        self.report.set_num_waypoint(self.command_mission.count)
                        self.auto_mode_activated = True
                        self.setting_waypoint_mission() #setting the waypoint according to user request in setting
                        self.arm_and_takeoff(self.setting.get_altitude()) #start takeoff
                        self.gui.show_msg_monitor(">> The drone begins the mission", "msg")
                        self.vehicle.parameters['WPNAV_SPEED'] = self.setting.get_auto_speed()
                        self.vehicle.parameters['RTL_ALT'] = (self.setting.get_altitude()*100)
                        self.report.set_top_speed(self.setting.get_auto_speed())
                        self.vehicle.mode = VehicleMode("AUTO")
                        self.vehicle.flush()
                        while not self.vehicle.mode.name == 'AUTO':
                            time.sleep(1)
                        #self.vehicle.flush()
                        self.read_waypoint_live()

                        self.gui.show_msg_monitor(">> Start landing...", "msg")
                        while self.vehicle.armed:
                            time.sleep(1)

                        self.gui.show_msg_monitor(">> The drone is landed success,end of mission ", "success")
                        self.auto_mode_activated = False
                        self.manual_mode()
                    else:
                        self.gui.show_msg_monitor(">> Please enter mission", "msg")
                else:
                    self.vehicle.mode = VehicleMode("AUTO")
                    self.vehicle.flush()
                    self.gui.show_msg_monitor(">> AUTO mode activated", "msg")
            else:
                self.gui.show_msg_monitor(">> Auto mode already on", "msg")
        else:
            self.gui.show_msg_monitor(">> Please wait until the drone will connect", "msg")

    # the function read all the waypoint and edit values by setting user, and insert rtl mode to end of mission.call from auto_mode function
    def setting_waypoint_mission(self):

        self.command_mission = self.vehicle.commands
        self.command_mission.download()
        self.command_mission.wait_ready()

        if self.__insert_end_mission is False:
            self.__insert_end_mission = True
            missionlist = []
            for cmd in self.command_mission:
                print(cmd)
                cmd.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                cmd.autocontinue = 0
                cmd.z = self.setting.get_altitude() #change the altitude according to the setting that user insert
                missionlist.append(cmd)



            home = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, self.__home_loc.lat,
                       self.__home_loc.lon, self.setting.get_altitude())

            rtl = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                      0, 0, 0, 0, 0, 0, 0, 0)

            missionlist.append(home)  # add new mission,home location waypoint to end of missions
            missionlist.append(rtl)   # add rtl mode to end of mission

            self.command_mission.clear()
            self.vehicle.flush()
            time.sleep(1)
            for cmd in missionlist:
                self.command_mission.add(cmd)
            self.command_mission.upload()
            self.vehicle.flush()
        for cmd in self.command_mission:
            print(cmd)
    #the function read the waypoint and print the waypoint that the drone move it.call from auto_mode function

    def read_waypoint_live(self):
        nextwaypoint = self.vehicle.commands.next
        while nextwaypoint < len(self.vehicle.commands):
            if self.vehicle.commands.next > nextwaypoint:
                display_seq = self.vehicle.commands.next
                point_num = "Moving to waypoint %s" % display_seq
                self.gui.show_msg_monitor(">> " + point_num , "msg")
                nextwaypoint = self.vehicle.commands.next
            time.sleep(1)

    def home_location(self,aLocation, aCurrent=1):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
            0,  # confirmation
            aCurrent,  # param 1: 1 to use current position, 2 to use the entered values.
            0, 0, 0,  # params 2-4
            aLocation.lat,
            aLocation.lon,
            aLocation.alt
        )
        return msg

    # Arms vehicle and up to target_altitude.the function call from this class by auto_mode function
    def arm_and_takeoff(self, target_altitude):
        #self.gui.show_msg_monitor(">> Drone takeoff to " + target_altitude + "meter altitude...", "msg")
        if self.vehicle.armed is False:
            self.report.create_folder()
            self.gui.show_msg_monitor(">> Pre-arm checks", "msg")
            # Don't try to arm until autopilot is ready
            while not self.vehicle.is_armable:
                self.gui.show_msg_monitor(">> Waiting for vehicle to initialise...", "msg")
                time.sleep(1)


            # Copter should arm in GUIDED mode
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            self.vehicle.flush()
            # Confirm vehicle armed before attempting to take off
            self.gui.show_msg_monitor(">> Waiting for arming...", "msg")
            while not self.vehicle.armed:
                time.sleep(1)
            self.gui.show_msg_monitor(">> ARMING MOTORS", "success")
            #self.is_armed = True
            #msg = ">> take off to " + target_altitude + " meter ..."
            #self.gui.show_msg_monitor(str(msg), "msg")
            self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

            # Wait until the vehicle reaches a safe height before processing the AUTO mode
            while True:
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                    self.gui.show_msg_monitor(">> Reached target altitude", "success")
                    self.report.set_max_alt(target_altitude)
                    break
                time.sleep(1)
        else:
            self.gui.show_msg_monitor(">> the drone is armed", "msg")

    #connect to drone and create vehicle obj that this obj send and get msg from the drone.the function call from this calss. or by connecting_sitl ot by mav_proxy_connect
    def connecting_drone(self,key):
        self.gui.show_msg_monitor(">> drone connecting...", "msg")
        try:
            if key == 'sitl':
                self.vehicle = connect('127.0.0.1:14550', wait_ready=True) # Connect to the Vehicle.
                #self.vehicle.wait_ready(True, timeout=60)
            else:
                # connecting to the vehicle by udp- this we can also open the massion planner with the python
                self.vehicle = connect("127.0.0.1:14550", wait_ready=False, baud=57600) # Connect to the Vehicle.
                # wait_ready is for that all info from drone upload 100%
                self.vehicle.wait_ready(True, timeout=70)
        # Bad TCP connection
        except socket.error:
            self.cam_connect = False
            self.gui.show_msg_monitor(">> No server exists! ", 'error')
            return
        # Bad TTY connection
        except exceptions.OSError as e:
            self.cam_connect = False
            self.gui.show_msg_monitor(">> No serial exists! ", 'error')
            return
        # API Error
        except APIException:
            self.cam_connect = False
            self.gui.show_msg_monitor(">> the Time is out!", 'error')
            return
        # Other error
        except:
            self.cam_connect = False
            self.gui.show_msg_monitor(">> Some other error! ,check communication", 'error')
            return

        self.gui.show_msg_monitor(">> GPS: %s" % self.vehicle.gps_0, "msg")
        self.gui.show_msg_monitor(">> Battery: %s" % self.vehicle.battery, "msg")
        self.gui.show_msg_monitor(">> Last Heartbeat: %s" % self.vehicle.last_heartbeat, "msg")
        self.gui.show_msg_monitor(">> System status: %s" % self.vehicle.system_status.state, "msg")
        self.gui.show_msg_monitor(">> Mode: %s" % self.vehicle.mode.name, "msg")
        self.__home_loc = self.vehicle.location.global_relative_frame
        self.vehicle.mode = VehicleMode("GUIDED")
        self.drone_connected = True
        self.report.set_home_location(self.__home_loc)


        self.gui.show_msg_monitor(">> Drone is connected", "success")


    def connecting_sitl(self):
        """"
        print("blaaaaa blaaa blaaaaas")
        self.cam_connect = True
        drone_kit_sitl = 'dronekit-sitl copter --home=31.7965240478516,35.3291511535645,248.839996337891,240'
        self.dronekit_process = subprocess.Popen(drone_kit_sitl, shell=True, stdin=PIPE, stdout=subprocess.PIPE)
        print("self.dronekit_process",self.dronekit_process.pid)
        #mavproxy_sitl_thread = threading.Thread(name='start mavProxy sitl', target=self.connection_mavproxy_sitl)
        #mavproxy_sitl_thread.start()
        #time.sleep(4)
        mav_proxy_sitl = 'mavproxy.py --master=tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551 '
        self.mavProxy_sitl_proc = subprocess.Popen(mav_proxy_sitl, shell=True, stdin=PIPE, stdout=subprocess.PIPE)
        print("self.dronekit_process", self.mavProxy_sitl_proc.pid)
        print("im in mavproxy sitl connection")
        #time.sleep(3)
        """
        self.connecting_drone('sitl')
    """
    def connection_mavproxy_sitl(self):
        time.sleep(4)
        mav_proxy_sitl = 'mavproxy.py --master=tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14551 '
        self.mavProxy_sitl_proc = subprocess.Popen(mav_proxy_sitl, shell=True, stdin=PIPE, stdout=subprocess.PIPE)
        print("self.dronekit_process", self.mavProxy_sitl_proc.pid)
        print("im in mavproxy sitl connection")
        time.sleep(3)
        self.connecting_drone('sitl')
    """

    #disconnect from sitl mode,close process and close communication,the function call from disconnect function in GUI class
    def sitl_disconnect(self):
        self.cam_connect = False
        self.drone_connected = False
        #search_pid_port = subprocess.Popen('netstat -ano | findstr :5760', shell=True, stdin=PIPE,stdout=subprocess.PIPE)

        #port_pid_task = search_pid_port.stdout.readline().split(" ")  # the line that pid of port 5760 is open
        #if port_pid_task is not None:
            #port_pid = port_pid_task[len(port_pid_task) - 1]
            #subprocess.Popen('taskkill /F /T /PID ' + port_pid, shell=True)

        #if self.mavProxy_sitl_proc is not None:
            #pid_mavproxy_sitl_proc = self.mavProxy_sitl_proc.pid
            #subprocess.Popen('taskkill /F /T /PID %i' % pid_mavproxy_sitl_proc, shell=True)


    #when person detection the function switch from auto mode to manual mode and:
    # 1)show msg in monitor that person detection
    # 2)show msg with selection mode "how to continue?"
    # 3)save a photo
    # 4)save locatin and time in report file.
    #the function call from detection_msg function in PersonDetection class
    def person_detected(self):
        if not self.person_is_detect:
            if self.drone_connected and self.vehicle.mode.name == 'AUTO':
                self.gui.show_msg_monitor(">> PERSON DETECTED !!", 'person')
                self.manual_mode()
                self.person_is_detect = True
                self.gui.show_msg_user("person detection")
                self.__person_location = self.vehicle.location.global_relative_frame
                self.report.set_time_detection(time.strftime("%H:%M:%S"))
                self.gui.get_image_function()
                while self.vehicle.groundspeed > 1: # wait the drone stop for more accuracy location
                    time.sleep(1)
                self.__person_location = self.vehicle.location.global_relative_frame

    #need to create distance detection in setting class!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #the function resets the detection option after a specified distance,to alarm again.call from user_reply_message function in GUI class
    def check_alarm_operation(self):
        while self.get_distance_metres(self.__person_location ,self.vehicle.location.global_relative_frame) < 15:
            time.sleep(1)
        self.person_is_detect = False

    #the function read information from drone,call from get_parm_drone function in GUI class
    def get_info_drone(self):
         dist_to_home = self.get_distance_metres(self.__home_loc,self.vehicle.location.global_relative_frame)
         info = {'alt': self.vehicle.location.global_relative_frame.alt,
                'ground_speed':self.vehicle.groundspeed,
                'dist_home': dist_to_home,
                'bat': self.vehicle.battery.voltage,
                'lon':self.vehicle.location.global_relative_frame.lon,
                'lat':self.vehicle.location.global_relative_frame.lat}
         return info

    #check distance between two point (metres)
    def get_distance_metres(self, location1, location2):
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
