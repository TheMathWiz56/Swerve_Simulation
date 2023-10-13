from PIL import Image
from tkinter import filedialog
import numpy as np
import os

"Defines a OGfromImage class instance"
"Has 3 RGBA values for specifying empty colors"
"Either takes a filepath or uses file explorer to path to an image"
"Generate Occupancy Grid for given image provide empty colors"


class OGfromImage:
    def __init__(self, file_name="", rgb1=None, rgb2=None, rgb3=None):
        self.oGGrid = None
        if rgb1 is None:
            rgb1 = [255, 255, 0, 247]
        if rgb2 is None:
            rgb2 = [255, 0, 255, 247]
        if rgb3 is None:
            rgb3 = [0, 255, 0, 247]
        self.rgb1 = rgb1
        self.rgb2 = rgb2
        self.rgb3 = rgb3
        self.file_name = file_name

        if self.file_name == "":
            self.file = filedialog.askopenfile(mode='r', filetypes=[('image files', '*.png')])
            if self.file:
                self.folder_path = os.path.abspath(self.file.name)
                self.field_image = Image.open(self.folder_path)
        else:
            self.field_image = Image.open(self.file_name)

        self.createOccupancyGrid()

    "Takes the image, converts into an nparray and then loops through and applies RGBA - OG conversion"
    def createOccupancyGrid(self):
        imageGrid = np.array(self.field_image.getdata())

        "Array Order: [y][x][RGBA data]"
        imageGrid = imageGrid.reshape((self.field_image.size[1], self.field_image.size[0], 4))

        self.oGGrid = np.empty(shape=(self.field_image.size[1], self.field_image.size[0]))

        i = 0
        for row in imageGrid:
            j = 0
            for col in row:
                self.oGGrid[i][j] = self.convertRGBtoOG(col)
                j += 1
            i += 1

        self.writeToTextFile()

    "used to set which colors are empty space"
    def setRGBConversion(self, rgb1, rgb2, rgb3):
        self.rgb1 = rgb1
        self.rgb2 = rgb2
        self.rgb3 = rgb3

    "Compares input array [R,G,B,A] against empty color arrays and returns Occupancy"
    def convertRGBtoOG(self, rgbInput):
        if (rgbInput == self.rgb1).all():
            return 1
        elif (rgbInput == self.rgb2).all():
            return 2
        elif (rgbInput == self.rgb3).all():
            return 0
        else:
            return 100

    "Saves generate Occupancy Grid text to a text file"
    def writeToTextFile(self):
        location = filedialog.asksaveasfilename(
            filetypes=[("txt file", ".txt")],
            defaultextension=".txt",
            initialfile="Occupancy Grid.txt",
            confirmoverwrite=True, )

        if location != "":
            f = open(location, "w")
        else:
            f = open("Occupancy Grids/Occupancy Grid.txt", "w")

        for row in self.oGGrid:
            for col in row:
                tempString = "%3s" % int(col)
                f.write(tempString + " ")
            f.write("\n")

    "Displays the Occupancy Grid as an image for easy visualization"
    def displayOGGrid(self):
        Image.fromarray(self.oGGrid).show()

    "Returns the Occupancy Grid Array"
    def getOGGrid(self):
        return self.oGGrid
