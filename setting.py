


from Tkinter import *
import threading
import re

class Setting:

        def __init__(self,master):
            self.menu_widget = Menu(master)
            self.__setting = {'set altitude(meter)': 7, 'set speed(c\s)':200, 'set num of cell':6, 'set min volt per cell':3.65}

            self.menu_widget.add_cascade(label="setting", command=lambda: self.start_window_by_thread("show setting"))
            self.menu_widget.add_cascade(label="upload mission", command=lambda: self.start_window_by_thread("upload mission"))
            master.config(menu=self.menu_widget)

        def get_altitude(self):
            return self.__setting['set altitude(meter)']
        def speed(self):
            return self.__setting['set speed(c\s)']
        def get_num_of_cell(self):
            return self.__setting['set num of cell']
        def get_min_v_per_cell(self):
            return self.__setting['set min volt per cell']
        def get_min_voltage(self):
            return self.get_min_v_per_cell()*self.get_num_of_cell() #the minimum voltage that drone can stay in the air.

        def show_setting(self):
            win = Toplevel()
            win.geometry("200x300")
            entries = []
            row = 0
            for key in self.__setting.keys():
                v = v = IntVar()
                v.set(self.__setting[key])
                lab = Label(win, width=15, text=key+": ", anchor='w')
                ent = Entry(win,width=7,textvariable=v)

                lab.grid(row = row ,column = 0,sticky = W)
                ent.grid(row = row,column = 1,sticky = E)
                entries.append((key, ent))
                row += 1

            button_save = Button(win, text='Save', command=lambda: self.save_setting(win,entries))
            button_save.grid(row = row+1)

        def save_setting(self,win,entries):
            p = re.compile('\d+(\.\d+)?')
            for entry in entries:
                if p.match(entry[1].get()):
                    self.__setting[entry[0]] = entry[1].get()

            win.destroy()

        def upload_mission(self):
            # seting= setting.Setting()
            # open_setting_window = threading.Thread(name='open_setting_window', target=self.submenu_callback(master))
            win = Toplevel()
            win.geometry("200x200")
            # display message
            message = "This is the child window"
            Label(win, text=message).pack()
            # quit child window and return to root window
            # the button is optional here, simply use the corner x of the child window
            Button(win, text='OK', command=win.destroy).pack()
        def start_window_by_thread(self,function):
            if function == 'show setting':
                open_window = threading.Thread(name='open_setting_window', target= self.show_setting)
            elif function == 'upload mission':
                open_window = threading.Thread(name='open_upload_mission_window', target=self.upload_mission)
            open_window.start()