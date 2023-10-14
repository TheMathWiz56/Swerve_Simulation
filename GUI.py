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
import Joystick

"""
To Do List:
1. edit field image for tool, use an expected color for empty
2. create tool to make occupancy grid from image
    can create occupancy grid with color-zone differentiation
    occupancy grid with not color-zone differentiation (program differentiates)

3. have main gui default to normal field image and occupancy grid
4. get xbox input in program

combine occupancy grid classes into one

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
width = "970"
height = "470"
root = tk.Tk()
root.geometry(width + "x" + height)
root.minsize(int(width), int(height))
root.maxsize(int(width), int(height))
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


def updateJoystick():
    x, y, x1, y1 = joystick1.get_values()
    jcanvas0.moveto(jimg0, int(165 / 2) - 19 + int(x * 65), int(165 / 2) - 18 - int(y * 65))
    jcanvas1.moveto(jimg1, int(165 / 2) - 19 + int(x1 * 65), int(165 / 2) - 18 - int(y1 * 65))
    root.after(20, updateJoystick)


isEnabled = 0


def robotEnable():
    global isEnabled
    isEnabled += 1
    robotEnableFlash()


enabledFlashCounter = 0


def robotEnableFlash():
    global enabledFlashCounter
    if isEnabled % 2 == 1:
        if enabledFlashCounter == 0:
            enabledFlash.config(text="Enabled")
            enabledFlashCounter += 1
        else:
            enabledFlash.config(text="")
            enabledFlashCounter -= 1
        root.after(100, robotEnableFlash)
    else:
        enabledFlash.config(text="Enable")


"Joystick instance"
joystick1 = Joystick.XboxController()
root.after(20, updateJoystick)

"Initialize all frames for main window"
borderwidth = 5
tFrame = ttk.Frame(root, relief='raised', borderwidth=borderwidth)
tFrame0 = ttk.Frame(tFrame)
tFrame1 = ttk.Frame(tFrame)
mFrame = ttk.Frame(root)
mFrame0 = ttk.Frame(mFrame, relief='groove', borderwidth=borderwidth)
mFrame1 = ttk.Frame(mFrame)
bFrame = ttk.Frame(root, relief='raised', borderwidth=borderwidth)
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
mFrame0.configure(width=cWidth, height=cHeight)
canvas = tk.Canvas(mFrame0, height=cHeight, width=cWidth)
canvas.pack(side='left', anchor='sw', expand=True)

fieldImage = Image.open("Images/Field Image5.png")
fieldImage = fieldImage.resize((cWidth, cHeight))
img = ImageTk.PhotoImage(fieldImage)
canvas.create_image(0, cHeight / 2, anchor="w", image=img)

"Create and pack buttons for tFrame"
tbWidth = 20
pickStart = ttk.Button(tFrame0, text="Pick Start", width=tbWidth)
pickEnd = ttk.Button(tFrame0, text="Pick End", width=tbWidth)
showPath = ttk.Button(tFrame0, text="Show Path", width=tbWidth)
enabledFlash = ttk.Button(tFrame1, text="Enable", width=tbWidth, command=robotEnable)

tbpadx = 37
tbpady = 0
pickStart.pack(side="left", padx=tbpadx, pady=tbpady)
pickEnd.pack(side="left", padx=tbpadx, pady=tbpady)
showPath.pack(side="left", padx=tbpadx, pady=tbpady)
enabledFlash.pack(anchor="e")

"Create and pack canvases for joystick graphs"
jcsize = 165
jcanvas0 = tk.Canvas(mFrame1, width=jcsize, height=jcsize)
jcanvas1 = tk.Canvas(mFrame1, width=jcsize, height=jcsize)

jpadx = 5
jpady = 10
jcanvas0.pack(side='top', anchor="w", pady=jpady, padx=jpadx)
jcanvas1.pack(side='top', anchor="w", pady=jpady, padx=jpadx)

"Draw axes on Joystick Canvases"
jcanvas0.create_oval(0, 0, jcsize, jcsize, fill="white")
jcanvas1.create_oval(0, 0, jcsize, jcsize, fill="white")

jcanvas0.create_line(jcsize / 2, 0, jcsize / 2, jcsize, dash=(4, 2))
jcanvas0.create_line(0, jcsize / 2, jcsize, jcsize / 2, dash=(4, 2))
jcanvas1.create_line(jcsize / 2, 0, jcsize / 2, jcsize, dash=(4, 2))
jcanvas1.create_line(0, jcsize / 2, jcsize, jcsize / 2, dash=(4, 2))

"Adding joystick images to Joystick Canvases"
jimgsize = 40

joystickImagge = Image.open("Images/Thumbstick.png")
joystickImagge = joystickImagge.resize((jimgsize, jimgsize))
jimgpng = ImageTk.PhotoImage(joystickImagge)
jimg0 = jcanvas0.create_image(jcsize / 2, jcsize / 2, image=jimgpng)
jimg1 = jcanvas1.create_image(jcsize / 2, jcsize / 2, image=jimgpng)

"Create and pack buttons for bFrame"
allianceColor = ttk.Button(bFrame0, text="Alliance Color", width=tbWidth)
togglePieces = ttk.Button(bFrame0, text="Toggle Pieces", width=tbWidth)
timer = ttk.Label(bFrame1, text="Timer", width=tbWidth)

bbpadx = 107
bbpady = 0
allianceColor.pack(side='left', padx=bbpadx, pady=bbpady, fill='x')
togglePieces.pack(side='left', padx=bbpadx, pady=bbpady, fill='x')
timer.pack(pady=bbpady, fill='x')

"Creating Menubar"
menubar = tk.Menu()
fieldM = tk.Menu(menubar, tearoff=False)
robotM = tk.Menu(menubar, tearoff=False)
driveM = tk.Menu(menubar, tearoff=False)
physicalSettingsM = tk.Menu(menubar, tearoff=False)
kinematicsM = tk.Menu(menubar, tearoff=False)
dimensionsM = tk.Menu(menubar, tearoff=False)
simulationM = tk.Menu(menubar, tearoff=False)

menubar.add_cascade(label='Field', menu=fieldM)
fieldM.add_cascade(label="Update Field Image")
fieldM.add_cascade(label="Update Occupancy Grid")
fieldM.add_cascade(label="Display Occupancy Grid")
fieldM.add_cascade(label="Set Default")

menubar.add_cascade(label='Robot', menu=robotM)
robotM.add_cascade(label="Drive", menu=driveM)
driveM.add_cascade(label="Tank")
driveM.add_cascade(label="Swerve")
robotM.add_cascade(label="Physical Settings", menu=physicalSettingsM)
physicalSettingsM.add_cascade(label="Kinematics", menu=kinematicsM)
physicalSettingsM.add_cascade(label="Dimensions", menu=dimensionsM)
kinematicsM.add_cascade(label="MAX Velocity")
kinematicsM.add_cascade(label="MAX Acceleration")
kinematicsM.add_cascade(label="MAX Jerk")
dimensionsM.add_cascade(label="Edit Width")
dimensionsM.add_cascade(label="Edit Length")

menubar.add_cascade(label="Simulation", menu=simulationM)
simulationM.add_cascade(label="Set % error")
simulationM.add_cascade(label="Set PID")

menubar.add_cascade(label="Print Window Size", command=getWindowSize)

"Display of menu bar in the app"
root.config(menu=menubar)

"loop main window"
root.mainloop()
