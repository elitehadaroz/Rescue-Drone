
import os
import time
import cv2
import csv

class Report:
    def __init__(self, ):
        self.path = None
        self.image_num = 0
        self.csv_writer =None
        self.creat_folder()

        with open(self.path+'\Report mission.csv','a') as new_file:
            self.csv_writer= csv.writer(new_file)
        new_file.close()


    def creat_folder(self): # create folder that hold on report on the mission
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

        #self.csv_report()

    def set_csv_report(self):
        with open(self.path+'\Report mission.csv','ab') as new_file:
            self.csv_writer= csv.writer(new_file)

            data = ["name","frt"]
            self.csv_writer.writerow(data)
        new_file.close()


    def set_image(self,image,person=''):
        t = time.strftime("%H-%M-%S")
        new_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.path+'\RD' + str(self.image_num) +"--"+t + person+'.png', new_image)
        self.image_num += 1

        with open(self.path+'\Report mission.csv','ab') as new_file:
            self.csv_writer= csv.writer(new_file)
            data = ["bnn","bbb"]
            self.csv_writer.writerow(data)
        new_file.close()