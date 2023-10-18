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

create graphic for what robot looks like, ie where modules are and which is length and which is width


5. make swerve drive class
6. display robot image on top of field image
7. ability to move robot around on field
8. ability to rotate robot on field
9. edge detection for robot movement
10. auto pathing........

11. only path into correct color zones

button for only manual control screen, just field and robot with maybe joysticks


"""

"Prints whole array without abbreviating"
np.set_printoptions(threshold=sys.maxsize)

"Tkinter window size and formatting"
width = "970"
height = "870"
root = tk.Tk()
root.geometry(width + "x" + height)
root.minsize(int(width), int(height))
root.maxsize(int(width), int(height))
root.title("2848 Swerve Path Planner")
sv_ttk.use_dark_theme()

"Tkinter commands"

OCG = OccupancyGrid.OccupancyGrid("Occupancy Grids/Occupancy Grid3.txt")
OCGarr = np.array(OCG.getOGGrid())
"all if ft/s, ft/s^2, and ft/s^3 respectively, jerk may not be needed"
"Will need to take angular velocity, acceleration, and jerk into account"
vMax = 0
aMax = 0
jMax = 0

rWidth = 0
rLength = 0

parameters = [vMax, aMax, jMax, rWidth, rLength]
print(OCGarr.shape)


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
    if isEnabled % 2 == 1:
        robotEnableFlash()
        updateRobotPose()


enabledFlashCounter = 0
deltaT = 10


def robotEnableFlash():
    global enabledFlashCounter
    if enabledFlashCounter == 0:
        enabledFlash.config(text="Enabled")
        enabledFlashCounter += 1
    else:
        enabledFlash.config(text="")
        enabledFlashCounter -= 1
    if isEnabled % 2 == 1:
        root.after(100, robotEnableFlash)
    else:
        enabledFlash.config(text="Enable")


def updateRobotPose():
    global robotx, roboty, robotw, tkrobotImage, crobotImage
    x, y, x1, y1 = joystick1.get_values()
    "will need to find PythagC, cap at maxV and then find theta from og x and y"
    temprobotx = robotx
    temproboty = roboty
    temprobotw = robotw

    temprobotx += (x * deltaT / 1000 * 500)
    temproboty += (y * deltaT / 1000 * 500)
    temprobotw += x1 * deltaT / 1000 * 1000 * -1

    xyMatrix = np.array([temprobotx, temproboty])
    rMatrix = np.array([[1, 0], [0, -1]])
    canvasOutput = np.matmul(xyMatrix, rMatrix)

    xsize, ysize = OCGarr.shape
    OCGxscaler = xsize / cWidth
    OCGyscaler = ysize / cHeight

    if checkRobotPoseUpdate(int(canvasOutput[0] * OCGxscaler), int(ysize - canvasOutput[1] * OCGyscaler - 1),
                            temprobotw, 56):
        canvas.delete(crobotImage)
        robotImageR = robotImage.rotate(robotw, expand=True)
        tkrobotImage = ImageTk.PhotoImage(robotImageR)
        crobotImage = canvas.create_image(canvasOutput[0], canvasOutput[1], image=tkrobotImage)
        robotx = temprobotx
        roboty = temproboty
        robotw = temprobotw
    print(checkRobotPoseUpdate(int(canvasOutput[0] * OCGxscaler), int(ysize - canvasOutput[1] * OCGyscaler - 1),
                               temprobotw, 56))

    if isEnabled % 2 == 1:
        root.after(deltaT, updateRobotPose)


def checkRobotPoseUpdate(xpos, ypos, wdeg, radius):
    print(xpos, ypos)
    robotVerticesArr = getRobotVertices(xpos, ypos, wdeg, radius)
    try:
        print(OCGarr[xpos][ypos])
    except:
        print("out of bounds")
    """for x in range(xpos - radius, xpos + radius):
        for y in range(ypos - radius, ypos + radius):
            if np.hypot(x - xpos, y - ypos) <= radius:
                try:
                    if OCGarr[x][y] == 100 or x <= 0 or y <= 0:
                        return False
                except:
                    return False"""
    degpv = wdeg + 45
    times180 = int(abs(degpv) / 180)
    print(degpv, times180)
    if times180 > 0:
        if wdeg > 0:
            if times180 % 2 == 0:
                degpv = degpv % 360
            else:
                degpv = -180 + degpv % 180
        else:
            if times180 % 2 == 1:
                degpv = degpv % 360
            else:
                degpv = -180 + degpv % 180
    print("wdeg: " + str(degpv))

    v1 = robotVerticesArr[0:2]
    v2 = robotVerticesArr[2:4]
    v3 = robotVerticesArr[4:6]
    v4 = robotVerticesArr[6:8]

    "top, right, left, down"
    if -45 < degpv < 45:
        if not (rectangularRobotPoseCheck(v3, v4, v2, v1)):
            return False
    elif -135 < degpv < -45:
        if not (rectangularRobotPoseCheck(v2, v3, v1, v4)):
            return False
    elif 45 < degpv < 135:
        if not (rectangularRobotPoseCheck(v4, v1, v3, v2)):
            return False
    elif 135 < degpv or degpv < -135:
        if not (rectangularRobotPoseCheck(v1, v2, v4, v3)):
            return False

    return True


def getRobotVertices(xpos, ypos, wdeg, radius):
    wdeg = -wdeg + 45
    v1theta = -1 * wdeg + 90
    v2theta = -1 * wdeg
    v3theta = -1 * wdeg - 90
    v4theta = -1 * wdeg - 180

    v1x = radius * np.cos(v1theta * np.pi / 180)
    v1y = radius * np.sin(v1theta * np.pi / 180)
    v2x = radius * np.cos(v2theta * np.pi / 180)
    v2y = radius * np.sin(v2theta * np.pi / 180)
    v3x = radius * np.cos(v3theta * np.pi / 180)
    v3y = radius * np.sin(v3theta * np.pi / 180)
    v4x = radius * np.cos(v4theta * np.pi / 180)
    v4y = radius * np.sin(v4theta * np.pi / 180)

    return [int(xpos + v1x), int(ypos + v1y), int(xpos + v2x), int(ypos + v2y), int(xpos + v3x), int(ypos + v3y),
            int(xpos + v4x), int(ypos + v4y)]


def getLineEquation(v0, v1):
    x0, y0 = v0
    x1, y1 = v1
    try:
        slope = (y1 - y0) / (x1 - x0)
        intercept = y0 - slope * x0

        return slope, intercept
    except:
        print("division by 0")


def rectangularRobotPoseCheck(vT, vR, vL, vD):
    for x in range(vT[0], vD[0]):
        if x < vL[0]:
            if x < vR[0]:
                if not (checkBetweenLines(vL, vT, vT, vR, x)):
                    return False
            else:
                if not (checkBetweenLines(vL, vT, vR, vD, x)):
                    return False
        else:
            if x < vR[0]:
                if not (checkBetweenLines(vL, vD, vT, vR, x)):
                    return False
            else:
                if not (checkBetweenLines(vL, vD, vR, vD, x)):
                    return False
    return True


def checkBetweenLines(v1, v2, v3, v4, x):
    line1 = getLineEquation(v1, v2)
    line2 = getLineEquation(v3, v4)
    for y in range(int(line1[0] * x + line1[1]), int(line2[0] * x + line2[1])):
        try:
            if OCGarr[x][y] == 100 or x <= 0 or y <= 0:
                return False
        except:
            return False

    return True


def updateParameterScreen(i, parameterName):
    window = tk.Toplevel(root)
    window.title = "Update Parameter"
    window.geometry("250x150")
    sv_ttk.use_dark_theme()
    parameterNameLbl = ttk.Label(window, text=parameterName)
    parameterNameLbl.pack()
    parameterEntry = ttk.Entry(window, takefocus=True)
    parameterEntry.pack()
    parameterButton = ttk.Button(window, text="Update Parameter",
                                 command=lambda: updateParameter(i, parameterEntry.get(), window))
    parameterButton.pack()
    window.mainloop()


def updateParameter(i, value, window):
    parameters[i] = value
    print(parameters[i])
    window.destroy()


def checkPoseKinematics(x, y, x1):
    "returns coerced updated x y and w according to kinematics data"
    global robotx, roboty, robotw
    temprobotx = robotx
    temproboty = roboty
    temprobotw = robotw

    temprobotx += (x * deltaT / 1000 * 500)
    temproboty += (y * deltaT / 1000 * 500)
    temprobotw += x1 * deltaT / 1000 * 1000 * -1


def rangeCoerce(cmin, cmax, cinput):
    if cinput > cmax:
        return cmax
    elif cinput < cmin:
        return cmin
    return cinput


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

"Create and pack field&robot image and canvas"
cSizeMultiplier = 7680 / 3720
cWidth = 375
cHeight = int(cWidth * cSizeMultiplier)
mFrame0.configure(width=cWidth, height=cHeight)
canvas = tk.Canvas(mFrame0, height=cHeight, width=cWidth)
canvas.pack(side='left', anchor='sw', expand=True, padx=200)

fieldImage = Image.open("Images/Field Image5.png")
fieldImage = fieldImage.resize((cHeight, cWidth))
fieldImage = fieldImage.rotate(90, expand=True)
img = ImageTk.PhotoImage(fieldImage)
canvas.create_image(0, cHeight / 2, anchor="w", image=img)

robotImage = Image.open("Images/RobotImageBlue.png")
robotImage = robotImage.resize((40, 40))
tkrobotImage = ImageTk.PhotoImage(robotImage)
crobotImage = canvas.create_image(cWidth / 2, cHeight / 2, image=tkrobotImage)

robotx = cWidth / 2
roboty = -1 * cHeight / 2
robotw = 0  # in degrees

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
jcanvas0lbl = ttk.Label(mFrame1, text="Left Joystick")
jcanvas1lbl = ttk.Label(mFrame1, text="Right Joystick")

jpadx = 5
jpady = 10
jcanvas0lbl.pack(side='top', anchor="center", pady=jpady, padx=jpadx)
jcanvas0.pack(side='top', anchor="w", pady=jpady, padx=jpadx)
jcanvas1.pack(side='top', anchor="w", pady=jpady, padx=jpadx)
jcanvas1lbl.pack(side='top', anchor="center", pady=jpady, padx=jpadx)

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
kinematicsM.add_cascade(label="MAX Velocity", command=lambda: updateParameterScreen(0, "Max Velocity"))
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
