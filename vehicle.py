import threading
import subprocess
from subprocess import PIPE
import socket
import time
from pymavlink import mavutil, mavwp
from dronekit import connect, VehicleMode, APIException ,Command  # Import DroneKit-Python
import exceptions
import math
import os

class DroneControl:
    def __init__(self, gui_obj,repo_obj,setting_obj):
        self.mavlink_connected = False
        self.gui = gui_obj
        self.setting = setting_obj
        self.report = repo_obj
        self.mavlink_time_out = False
        self.mavlink_proc=None  #this proc start the mavProxy for real drone
        self.mavProxy_sitl_proc=None #this proc start simulator mavProxy mavlink
        self.vehicle=None
        self.stop_timer=None    #this boolean stop_timer is count X second to login option, if is pass X second the system stop trying to connect
        self.drone_connected=False
        self.person_is_detect = False
        self.cam_connect = True
        #self.is_armed = False
        self.command_mission = None
        self.auto_mode_activated = False
        self.__home_loc = None
        self.__person_location = None

    def mav_proxy_connect(self):
        self.gui.show_msg_monitor(">> start to connect mavProxy,please wait...", "msg")
        self.mavlink_time_out = False  # 120s to chance connect mavproxy
        mav_proxy = 'mavproxy.py --master="'+self.setting.get_usb_com()+'" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        print(mav_proxy)
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
            self.connecting_drone('drone')
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
        if self.vehicle is not None:
            self.vehicle.close()
        self.gui.show_msg_monitor(">> Disconnects from the drone...", "msg")
        self.mavlink_time_out = True  # reset the timer to connection
        self.cam_connect = False
        # gui.drone_is_connect=False                    #to reset the option to connect again
        # gui.button_connect.config(text="Connect")   #to reset the name button to connect again
        self.drone_connected = False
        if self.mavlink_proc is not None:
            pid = self.mavlink_proc.pid
            print("mavlink proc kill is",pid)
            subprocess.Popen('taskkill /F /T /PID %i' % pid, shell=True)
            self.gui.show_msg_monitor(">> Drone is disconnected", "msg")


    def manual_mode(self):      #set GUIDED mode,the drone now in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.gui.show_msg_monitor(">> GUIDED mode activated", "msg")
        self.vehicle.flush()
    def rtl_mode(self):         #set RTL mode,the drone now in RTL mode
        self.manual_mode()
        self.vehicle.simple_goto(self.__home_loc)
        self.vehicle.flush()
        time.sleep(15)
        self.vehicle.mode = VehicleMode("RTL")
        self.gui.show_msg_monitor(">> RTL mode activated", "msg")
        self.vehicle.flush()
    def stabilize_mode(self):
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.gui.show_msg_monitor(">> STABILIZE mode activated", "msg")
        self.vehicle.flush()

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
            # send command to vehicle
            #self.vehicle.send_mavlink(msg)
        else:
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # -- BITMASK -> Consider only the velocities
                0, 0, 0,  # -- POSITION
                vx, vy, vz,  # -- VELOCITY
                0, 0, 0,  # -- ACCELERATIONS
                0, yaw)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_gps_and_rtl(self):

        """need to write function that send gps to server!!!!!!"""
        self.gui.get_image = True  # get picture of person detected
        #info =["person location:","lon:",self.__home_loc.lon,"lat:",self.__home_loc.lat]
        #self.report.set_csv_on_report(info)
        self.drone_vehicle.rtl_mode()
        self.vehicle.flush()
    def send_gps_and_stay(self):

        """need to write function that send gps to server!!!!!!"""
        self.manual_mode()
        self.vehicle.flush()
    def get_person_location(self):
        return self.__person_location


    def download_mission(self):     # after the user insert track on mission planner,this function read the track and save is on command_mission
        if self.command_mission is None :
            self.command_mission = self.vehicle.commands
            self.command_mission.download()
            self.command_mission.wait_ready()
            self.report.set_num_waypoint(self.command_mission.count)
            while not self.vehicle.home_location:
                self.command_mission = self.vehicle.commands
                self.command_mission.download()
                self.command_mission.wait_ready()
                self.report.set_num_waypoint(self.command_mission.count)
                if self.command_mission.count != 0:
                    self.gui.show_msg_monitor(">> Waiting for home location ...", "msg")
                if not self.vehicle.home_location:
                    time.sleep(1)
                    print("here oa read mission")
            if self.command_mission is not None:
                if self.vehicle.home_location and self.command_mission.count != 0:
                    self.gui.show_msg_monitor(">> Home location saved ", "success")
                    self.report.set_home_location(self.__home_loc)
                    self.vehicle.flush()
                    print("hereeeeeeeeeeeeeeeeeeeeeeeee")
                if self.command_mission is not None:
                    if self.command_mission.count == 0:
                        self.command_mission = None

    def upload_mission(self):
        missionlist = self.setting.get_missionlist()
        self.command_mission = self.vehicle.commands
        self.command_mission.clear()
        # Add new mission to vehicle
        for command in missionlist:
            self.command_mission.add(command)
        self.gui.show_msg_monitor(">> Upload mission success", "success")
        self.vehicle.commands.upload()

    def clean_missions(self):
        if self.command_mission is not None:
            self.command_mission.clear()
            self.command_mission = None
        self.gui.show_msg_monitor(">> Clean mission ", "success")
    def auto_mode(self):        #set AUTO mode,the drone now in AUTO mode
        if self.drone_connected is True:
            if self.vehicle.mode.name is not 'AUTO':
                if self.auto_mode_activated is False:
                    if self.command_mission is None:
                        if self.setting.get_missionlist() is not None:
                            self.upload_mission()
                            print("upload mission")
                        else:
                            print("download mission")
                            self.download_mission()
                    if self.command_mission is not None:
                        #missionlist = []
                        #for cmd in self.command_mission:
                            #missionlist.append(cmd)

                        #self.command_mission.clear()

                        #for cmd in missionlist:
                            #self.command_mission.add(cmd)
                        #self.command_mission.upload()

                        self.auto_mode_activated = True
                        self.setting_waypoint_mission() #setting the waypoint according to user request
                        #self.gui.show_msg_monitor(">> AUTO mode activated", "msg")
                        self.arm_and_takeoff(self.setting.get_altitude()) #start takeoff

                        self.gui.show_msg_monitor(">> The drone begins the mission", "msg")
                        self.vehicle.parameters['WPNAV_SPEED'] = self.setting.get_auto_speed()
                        self.report.set_top_speed(self.setting.get_auto_speed())
                        self.vehicle.mode = VehicleMode("AUTO")
                        self.vehicle.flush()
                        self.read_waypoint_live()

                        self.gui.show_msg_monitor(">> Start landing...", "msg")
                        while self.vehicle.armed:
                            time.sleep(1)

                        #self.is_armed = False


                        # Disarm vehicle

                        #while self.vehicle.armed:
                        #    time.sleep(1)

                        self.gui.show_msg_monitor(">> The drone is landed success,end of mission ", "success")
                        self.auto_mode_activated = False
                        self.stabilize_mode()
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

    def setting_waypoint_mission(self):     #the function read all the waypoint and edit values, and insert rtl mode to end of mission
        missionlist = []
        for cmd in self.command_mission:
            print(cmd)
            cmd.z = self.setting.get_altitude() #change the altitude according to the setting that user insert
            missionlist.append(cmd)


        home = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, self.__home_loc.lat,
                       self.__home_loc.lon, 20)

        rtl = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                      0, 0, 0, 0, 0, 0, 0, 0)

        missionlist.append(home)  # add new mission,rtl to end of missions
        missionlist.append(rtl)
        self.command_mission.clear()

        for cmd in missionlist:
            self.command_mission.add(cmd)
        self.command_mission.upload()

    def read_waypoint_live(self):
        nextwaypoint = self.vehicle.commands.next
        while nextwaypoint < len(self.vehicle.commands):
            if self.vehicle.commands.next > nextwaypoint:
                print("in if")
                display_seq = self.vehicle.commands.next
                point_num = "Moving to waypoint %s" % display_seq
                print(self.vehicle.system_status.state)
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

    def arm_and_takeoff(self, target_altitude):  # Arms vehicle and fly to target_altitude.
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

            # Confirm vehicle armed before attempting to take off
            self.gui.show_msg_monitor(">> Waiting for arming...", "msg")
            while not self.vehicle.armed:
                time.sleep(1)
            self.gui.show_msg_monitor(">> ARMING MOTORS", "success")
            #self.is_armed = True
            self.gui.show_msg_monitor(">> take off...", "msg")
            self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

            # Wait until the vehicle reaches a safe height before processing the goto
            #  (otherwise the command after self.vehicle.simple_takeoff will execute
            #   immediately).
            while True:
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                    self.gui.show_msg_monitor(">> Reached target altitude", "success")
                    self.report.set_max_alt(target_altitude)
                    break
                time.sleep(1)
        else:
            self.gui.show_msg_monitor(">> the drone is armed", "msg")

    def connecting_drone(self,key):
        self.gui.show_msg_monitor(">> drone connecting...", "msg")
        # Connect to the Vehicle.
        #print "Connecting to vehicle on: udp:127.0.0.1:14551"
        try:
            if key == 'sitl':
                self.vehicle = connect('127.0.0.1:14550', wait_ready=True)
                #self.vehicle.wait_ready(True, timeout=60)
                print("sitl connect")
            else:
                # connecting to the vehicle by udp- this we can also open the massion planner with the python
                self.vehicle = connect("127.0.0.1:14550", wait_ready=False, baud=57600)
                # wait_ready is for that all info from drone upload 100%
                self.vehicle.wait_ready(True, timeout=60)
           # self.vehicle.heartbeat_timeout(1000)
        # Bad TCP connection
        except socket.error:
            self.cam_connect = False
            print 'No server exists!'
            return
        # Bad TTY connection
        except exceptions.OSError as e:
            self.cam_connect = False
            print 'No serial exists!'
            return

        # API Error
        except APIException:
            self.cam_connect = False
            print 'the Time is out!'
            return
        # Other error
        except:
            self.cam_connect = False
            print 'Some other error!'
            return
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

        self.__home_loc = self.vehicle.location.global_relative_frame

        self.gui.show_msg_monitor(">> Drone is connected", "success")
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

    def sitl_disconnect(self):
        self.cam_connect = False
        self.drone_connected = False
        #self.dronekit_process.kill()
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

        self.gui.show_msg_monitor(">> Sitl is disconnected", "msg")

    def person_detected(self):
        if not self.person_is_detect:
            if self.drone_connected and self.vehicle.mode.name == 'AUTO':
                self.gui.show_msg_monitor(">> PERSON DETECTED !!", 'person')
                self.manual_mode()
                #self.vehicle.mode = VehicleMode("LOITER")
                #self.vehicle.parameters['THR_MID'] = 1500
                #self.stabilize_mode()
                self.person_is_detect = True
                self.gui.show_msg_user("person detection")
                self.__person_location = self.vehicle.location.global_relative_frame
                self.report.set_time_detection(time.strftime("%H:%M:%S"))
                self.gui.get_image_function()
                print("i get a piccccccc")
                while self.vehicle.groundspeed > 1: # wait the drone stop for more accuracy location
                    print("groaund speedd issssss "+ self.vehicle.groundspeed)
                    time.sleep(1)
                self.__person_location = self.vehicle.location.global_relative_frame
                #self.gui.get_image = True  # get picture of person detected



    def check_alarm_operation(self):
        while self.get_distance_metres(self.__person_location ,self.vehicle.location.global_relative_frame) < 15:
            time.sleep(1)
            #print("in here")
            print(self.get_distance_metres(self.__person_location ,self.vehicle.location.global_relative_frame))
        self.person_is_detect = False
        #print("after destanceeeeeeeeeeee" ,self.get_distance_metres(self.__person_location ,self.vehicle.location.global_relative_frame))

    def get_info_drone(self):
         dist_to_home = self.get_distance_metres(self.__home_loc,self.vehicle.location.global_relative_frame)
         info = {'alt': self.vehicle.location.global_relative_frame.alt,
                'ground_speed':self.vehicle.groundspeed,
                'dist_home': dist_to_home,
                'bat': self.vehicle.battery.voltage  }
         return info

    def get_distance_metres(self, location1, location2):
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5