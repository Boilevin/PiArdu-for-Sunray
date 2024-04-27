from tkinter import *
from tkinter.font import Font

root = Tk()
spin = Spinbox(root, from_=0, to=9, 
               font=Font(family='Arial', size=36))
spin.place(x=0, y=0, height=70, width=70)
root.mainloop()