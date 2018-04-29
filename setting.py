


from Tkinter import *
import threading

class Setting:

        def __init__(self,master):
            self.menu_widget = Menu(master)

            submenu_widget = Menu(self.menu_widget, tearoff=False)
            submenu_widget.add_command(label="properties", command=lambda: self.start_window_by_thread("properties"))
            #submenu_widget.add_command(label="Submenu Item2", command=self.submenu_callback(master))
            self.menu_widget.add_cascade(label="setting", menu=submenu_widget)
            master.config(menu=self.menu_widget)

        def properties(self):
            # seting= setting.Setting()
            # open_setting_window = threading.Thread(name='open_setting_window', target=self.submenu_callback(master))
            win = Toplevel()
            win.geometry("500x500")
            # display message
            message = "This is the child window"
            Label(win, text=message).pack()
            # quit child window and return to root window
            # the button is optional here, simply use the corner x of the child window
            Button(win, text='OK', command=win.destroy).pack()
        #show_setting_window(self):

        def start_window_by_thread(self,function):
            if function == 'properties':
                open_window = threading.Thread(name='open_setting_window', target= self.properties)
            open_window.start()