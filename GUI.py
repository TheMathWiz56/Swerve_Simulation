import OccupancyGrid
import numpy as np
import sys
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
import sv_ttk
import os
from PIL import ImageTk, Image
import GenerateOGfromImage

"""
To Do List:
1. edit field image for tool, use an expected color for empty
2. create tool to make occupancy grid from image
    can create occupancy grid with color-zone differentiation
    occupancy grid with not color-zone differentiation (program differentiates)

3. have main gui default to normal field image and occupancy grid
4. get xbox input in program
5. make swerve drive class
6. display robot image on top of field image
7. ability to move robot around on field
8. ability to rotate robot on field
9. edge detection for robot movement
10. auto pathing........

11. only path into correct color zones


"""

"Prints whole array without abbreviating"
np.set_printoptions(threshold=sys.maxsize)

"Tkinter window size and formatting"
root = tk.Tk()
root.geometry("1185x550")
root.title("2848 Swerve Path Planner")
sv_ttk.use_dark_theme()

"Tkinter commands"


def displayOccupancyGrid():
    file = filedialog.askopenfile(mode='r', filetypes=[('Text Document', '*.txt')])
    if file:
        folder_path = os.path.abspath(file.name)
        oc2 = OccupancyGrid.OccupancyGrid(folder_path)
        oc2.displayGrid()


def getWindowSize():
    print(root.winfo_width())
    print(root.winfo_height())


"Initialize all frames for main window"
tFrame = ttk.Frame(root)
tFrame0 = ttk.Frame(tFrame)
tFrame1 = ttk.Frame(tFrame)
mFrame = ttk.Frame(root)
mFrame0 = ttk.Frame(mFrame)
mFrame1 = ttk.Frame(mFrame)
bFrame = ttk.Frame(root)
bFrame0 = ttk.Frame(bFrame)
bFrame1 = ttk.Frame(bFrame)

"pack main frames"
tFrame.pack()
mFrame.pack()
bFrame.pack()

"pack subframes"
tFrame0.pack(side='left')
tFrame1.pack(side='left')
mFrame0.pack(side='left')
mFrame1.pack(side='left')
bFrame0.pack(side='left')
bFrame1.pack(side='left')

"Create and pack field image and canvas"
cSizeMultiplier = 7680 / 3720
cHeight = 375
cWidth = int(cHeight * cSizeMultiplier)
canvas = tk.Canvas(mFrame0, height=cHeight, width=cWidth)
canvas.pack(side='left', anchor='sw', expand=True)

fieldImage = Image.open("Images/Field Image5.png")
fieldImage = fieldImage.resize((cWidth, cHeight))
img = ImageTk.PhotoImage(fieldImage)
canvas.create_image(0, cHeight / 2, anchor="w", image=img)

"Create and pack buttons for tFrame"
pickStart = ttk.Button(tFrame0, text="Pick Start")
pickEnd = ttk.Button(tFrame0, text="Pick End")
showPath = ttk.Button(tFrame0, text="Show Path")
enabledFlash = ttk.Button(tFrame1, text="Enabled")

tbpadx = 100
tbpady = 10
pickStart.pack(side="left", padx=tbpadx, pady=tbpady)
pickEnd.pack(side="left", padx=tbpadx, pady=tbpady)
showPath.pack(side="left", padx=tbpadx, pady=tbpady)
enabledFlash.pack(anchor="e", padx=tbpadx, pady=tbpady)

"Create and pack canvases for joystick graphs"
jcsize = 200
jcanvas0 = tk.Canvas(mFrame0, width=jcsize, height=jcsize)
jcanvas1 = tk.Canvas(mFrame0, width=jcsize, height=jcsize)

jcanvas0.pack(side='top', padx=100, pady=10)
jcanvas1.pack(side='top', padx=100, pady=10)

"Draw axes on Joystick Canvases"
jcanvas0.create_oval(0, 0, 200, 200, fill="white")
jcanvas1.create_oval(0, 0, 200, 200, fill="white")

jcanvas0.create_line(100, 0, 100, 200, dash=(4, 2))
jcanvas0.create_line(0, 100, 200, 100, dash=(4, 2))
jcanvas1.create_line(100, 0, 100, 200, dash=(4, 2))
jcanvas1.create_line(0, 100, 200, 100, dash=(4, 2))

"Create and pack buttons for bFrame"
allianceColor = ttk.Button(bFrame0, text="Alliance Color")
togglePieces = ttk.Button(bFrame0, text="Toggle Pieces")
getScreenSize = ttk.Button(bFrame0, text="Get Size", command=getWindowSize)
timer = ttk.Label(bFrame1, text="Timer")

bbpadx = 150
bbpady = 10
allianceColor.pack(side='left', padx=bbpadx, pady=bbpady, fill='x')
togglePieces.pack(side='left', padx=bbpadx, pady=bbpady, fill='x')
getScreenSize.pack(side='left',pady=bbpady, fill='x')
timer.pack(pady=bbpady, fill='x')

"loop main window"
root.mainloop()
