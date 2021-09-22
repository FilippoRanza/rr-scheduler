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
        print(f"{name} -> {text}")
        lbl = self.labels[name]
        lbl["text"] = text
        self.update()

def init_gui(title, names):
    root = tk.Tk()
    return GuiLog(root, title, names)

def run_gui(gui: GuiLog):
    process = multiprocessing.Process(target=gui.mainloop)
    process.start()
    return process

