
from Tkinter import *
import threading
import re
import tkFileDialog
from dronekit import Command
from multiprocessing import Process, Queue

# the class holding values to drone,like speed altitude etc.
class Setting:
    def __init__(self,master,gui):
            self.__gui = gui
            self.__missionlist = None
            self.__setting = {'set altitude(meter)': 7, 'set AUTO speed(c\s)': 200, 'set MANU speed(m\s)': 1,
                              'set num of cell': 6, 'set min volt per cell': 3.65}
            self.__usb_com = "COM4"

            self.start_menu(master)

    # create the nav bar menu
    def start_menu(self,master):

        self.menu_widget = Menu(master)

        #the menu drone setting
        self.settings_drone = Menu(self.menu_widget,tearoff=0)
        self.menu_widget.add_cascade(label="drone setting",command=lambda: self.start_window_by_thread("show setting"))

        #the menu mission settings
        self.setting_mission = Menu(self.menu_widget,tearoff=0)
        self.menu_widget.add_cascade(label="mission settings", menu = self.setting_mission)

        self.setting_mission.add_command(label="upload mission",command=lambda: self.start_window_by_thread("upload mission"))
        self.setting_mission.add_command(label="clean mission",command=self.clean_mission)

        master.config(menu=self.menu_widget)

    def get_altitude(self):
            return self.__setting['set altitude(meter)']
    def get_auto_speed(self):
            return self.__setting['set AUTO speed(c\s)']
    def get_manu_speed(self):
            return self.__setting['set MANU speed(m\s)']
    def get_num_of_cell(self):
            return self.__setting['set num of cell']
    def get_min_v_per_cell(self):
            return self.__setting['set min volt per cell']
    def get_min_voltage(self):
            return self.get_min_v_per_cell()*self.get_num_of_cell() #the minimum voltage that drone can stay in the air.
    def get_missionlist(self):
        if self.__missionlist is not None:
            return self.__missionlist
        else:
            return None
    def get_usb_com(self):
        return self.__usb_com
    def show_setting(self):
            win = Toplevel()
            entries = []
            row = 0
            for key in self.__setting.keys():
                v = IntVar()
                v.set(self.__setting[key])
                lab = Label(win, width=20, text=key + ": ", anchor='w')
                ent = Entry(win, width=7,textvariable=v)

                lab.grid(row = row ,columnspan=2,column = 0,padx=10,sticky = W + E )
                ent.grid(row = row,column = 2,padx=10,pady=5,sticky = E)
                entries.append((key, ent))
                row += 1

            v = IntVar()
            v.set(self.__usb_com)
            lab_usb = Label(win, width=20, text="drone connect usb COM: ", anchor='w')
            usb_com = Entry(win, width=7, textvariable=v)
            lab_usb.grid(row=row,columnspan=2, column=0,padx=10, sticky=W)
            usb_com.grid(row=row, column=2, padx=10, pady=5, sticky=E)
            button_save = Button(win, text='Save', command=lambda: self.save_setting(win,entries,usb_com))
            button_save.grid(row = row+1,column = 1)

    #this function save the values in the self.__setting{}
    def save_setting(self,win,entries,usb_com):
            p = re.compile('\d+(\.\d+)?')
            for entry in entries:
                if p.match(entry[1].get()):
                    self.__setting[entry[0]] = float(entry[1].get())
            self.__usb_com = usb_com.get()
            win.destroy()

    # the read_mission function read a mission from file.waypoint and insert the waypoint in self.__missionlist
    def read_mission(self,q):
            filename = tkFileDialog.askopenfilename(filetypes=[("waypoint mission",'*.waypoints'),('All files','*.*')])
            print ("the filr name is ",filename)
            missionlist = []
            if filename != '':
                with open(filename) as f:
                    for i, line in enumerate(f):
                        if i == 0:
                            if not line.startswith('QGC WPL 110'):
                                self.__gui.show_msg_monitor(">> error: File is not supported WP version", "error")
                                raise Exception('File is not supported WP version')
                        else:
                            linearray = line.split('\t')
                            ln_index = int(linearray[0])
                            ln_currentwp = int(linearray[1])
                            ln_frame = int(linearray[2])
                            ln_command = int(linearray[3])
                            ln_param1 = float(linearray[4])
                            ln_param2 = float(linearray[5])
                            ln_param3 = float(linearray[6])
                            ln_param4 = float(linearray[7])
                            ln_param5 = float(linearray[8])
                            ln_param6 = float(linearray[9])
                            ln_param7 = float(linearray[10])
                            ln_autocontinue = int(linearray[11].strip())
                            cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1,ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                            missionlist.append(cmd)
                    q.put(missionlist)
            else:
                q.put(None)

    def clean_mission(self):
        self.__gui.clean_missions()
        self.__missionlist = None

    #the function start_window_by_thread call from start_menu and open window menu without stock the main window.
    def start_window_by_thread(self,function_name):
            if function_name == 'show setting':
                open_window = threading.Thread(name='open_setting_window', target=self.show_setting)
                open_window.start()
            elif function_name == 'upload mission':
                q = Queue()
                p = Process(target=self.read_mission(q))
                p.start()
                p.join()
                self.__missionlist = q.get()