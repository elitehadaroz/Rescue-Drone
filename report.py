
import os
import time
import cv2
import csv
from collections import OrderedDict

class Report:
    def __init__(self):
        self.path = None
        self.image_num = 0
        self.csv_writer =None
        self.__data_report = OrderedDict()
        self.__data_report = {'date':'','start mission':'','time connect':'','end mission':'','home location':'',
                              'air time':'','number of waypoint':'','max altitude':'','speed':'','persons location':[]}
        self.__num_of_person = 0
        self.__time_detection = ""


    def set_date(self):

        self.__data_report['date'] = time.strftime("%d/%m/%Y")


    def set_home_location(self,loc):

        self.__data_report['home location'] = ["lat:",loc.lat,"lon:",loc.lon]

    def set_drone_connect_time(self,time_c):

        self.__data_report["time connect"] =  time_c

    def set_start_mission(self,start):

        self.__data_report["start mission"] = start

    def set_end_mission(self,end):

        self.__data_report['end mission'] = end

    def set_air_time(self,time_a):

        self.__data_report['air time'] = time_a

    def set_num_waypoint(self,num):

        self.__data_report['number of waypoint'] =  num

    def set_max_alt(self,alt):

        self.__data_report['max altitude'] = alt

    def set_top_speed(self,speed):

        self.__data_report['speed'] = speed

    def set_person_loc(self,loc,name):
        person =["person"+ str(self.__num_of_person) + "," + name,"lat:",str(loc.lat),"lon:",str(loc.lon),"time detection:",str(self.__time_detection)]
        self.__data_report['persons location'].append(person)
        self.__num_of_person += 1

    def set_time_detection(self,time_de):
        self.__time_detection = time_de

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
        self.set_date()


    def create_report_mission(self): #write to cvs file all the info
        with open(self.path+'\Report mission.csv','ab') as new_file:
            self.csv_writer = csv.writer(new_file)
            self.csv_writer.writerow(['date',self.__data_report['date']])
            self.csv_writer.writerow(['time connect', self.__data_report['time connect']])
            self.csv_writer.writerow(['start mission', self.__data_report['start mission']])
            self.csv_writer.writerow(['end mission', self.__data_report['end mission']])
            self.csv_writer.writerow(['home location', self.__data_report['home location']])
            self.csv_writer.writerow(['air time', self.__data_report['air time']])
            self.csv_writer.writerow(['max altitude', self.__data_report['max altitude']])
            self.csv_writer.writerow(['number of waypoint', self.__data_report['number of waypoint']])
            self.csv_writer.writerow(['speed', self.__data_report['speed']])

            if self.__num_of_person != 0:
                for person in self.__data_report['persons location'] :
                    self.csv_writer.writerow(person)
        new_file.close()


    def set_image(self,image):
        t = time.strftime("%H-%M-%S")
        new_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.path+'\RD' + str(self.image_num) +"--"+ t +'.png', new_image)
        self.image_num += 1

