#! /usr/bin/python


import multiprocessing
import tkinter as tk

class GuiLog(tk.Frame):
    def __init__(self, master, title, names):
        super().__init__(master)
        self.pack()
        title = tk.Label(self, text=title)
        title.grid(row=0, column=0, columnspan=2)
        self.labels = self.make_list(names)
        self.queue = multiprocessing.Queue()
        self.__update_gui__()

    def make_list(self, names):
        output = {}
        for i, name in enumerate(names, 1):
            lbl_1 = tk.Label(self, text=name)
            lbl_1.grid(row=i, column=0, padx=20, pady=10)
            lbl_2 = tk.Label(self, text="")
            lbl_2.grid(row=i, column=1, padx=10, pady=10)
            output[name] = lbl_2
        return output

    def set_text(self, name, text):
        self.queue.put((name, text))

    def __update_gui__(self):
        while not self.queue.empty():
            name, text = self.queue.get()
            self.labels[name]["text"] = text
            self.update()
        self.after(1, self.__update_gui__)

def init_gui(title, names):
    root = tk.Tk()
    return GuiLog(root, title, names)

def run_gui(gui: GuiLog):
    process = multiprocessing.Process(target=gui.mainloop)
    process.start()
    return process
