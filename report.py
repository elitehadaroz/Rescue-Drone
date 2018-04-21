
import os
import time
import cv2
import csv

class Report:
    def __init__(self):
        self.path = None
        self.image_num = 0
        self.csv_writer =None
        self.create_folder()
        self.__date = []
        self.__home_location = []
        self.__drone_connect_time = []
        self.__start_mission = []
        self.__end_mission = []
        self.__air_time =[]
        self.__number_waypoint = []
        self.__max_altitude = []
        self.__top_speed = []
        self.__persons_location = []
        self.__num_of_person = 0
        self.__time_detection = ""
        self.create_folder()
        self.set_date()

    def set_date(self):
        self.__date = ["date",time.strftime("%d/%m/%Y")]

    def set_home_location(self,loc):
        self.__home_location = ["home location:","lat:",loc.lat,"lon:",loc.lon]
    def set_drone_connect_time(self,time):
        self.__drone_connect_time = ["time connect",time]

    def set_start_mission(self,start):
        self.__start_mission = ["Start mission",start]

    def set_end_mission(self,end):
        self.__end_mission = ["End mission",end]

    def set_air_time(self,time):
        self.__air_time = ["Air time",time]

    def set_num_waypoint(self,num):
        self.__number_waypoint = ["number of waypoint",num]

    def set_max_alt(self,alt):
        self.__max_altitude = ["Max altitude",alt]

    def set_top_speed(self,speed):
        self.__top_speed = ["Top speed",speed]

    def set_person_loc(self,loc,name):
        person =["person"+self.__num_of_person + "," + name,"lon:",loc.lat,"lon:",loc.lon,"time detection:",self.__time_detection]
        self.__persons_location.append(person)
        self.__num_of_person += 1

    def set_time_detection(self,time):
        self.__time_detection = time

    def create_folder(self): # create folder that hold on report on the mission
        mission_count = 0
        self.path = time.strftime("Missions\%d-%m-%Y No " + str(mission_count))
        while os.path.exists(self.path):
            mission_count += 1
            self.path = time.strftime("Missions\%d-%m-%Y No " + str(mission_count))
        try:
            os.makedirs(self.path)
        except OSError:
            if not os.path.isdir(self.path):
                raise


    def set_csv_on_report(self,info):
        with open(self.path+'\Report mission.csv','ab') as new_file:
            self.csv_writer= csv.writer(new_file)
            self.csv_writer.writerow(info)
        new_file.close()


    def set_image(self,image):
        t = time.strftime("%H-%M-%S")
        new_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.path+'\RD' + str(self.image_num) +"--"+ t +'.png', new_image)
        self.image_num += 1

