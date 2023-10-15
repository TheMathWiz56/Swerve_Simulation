import numpy as np
from PIL import Image

"Defined with a file location where occupancy grid text file is stored"
"Reads information from OG text file and inserts into a growing list"
"Once all lines have been read, converts into an nparray and reshapes itself"


class OccupancyGrid:
    def __init__(self, file_path):
        self.file_path = file_path
        self.occupancy_list = []

        with open(self.file_path) as f:
            self.text_input = f.readlines()
        self.colN = 0
        for i in self.text_input:
            self.tempString = ""
            for j in i:
                if j != '\t' and j != '\n' and j != " ":
                    self.tempString += j
                else:
                    if self.tempString != "":
                        self.occupancy_list.append(int(self.tempString))
                        self.colN += 1
                        self.tempString = ""
        self.occupancy_grid = np.array(self.occupancy_list)
        self.occupancy_grid = self.occupancy_grid.reshape(len(self.text_input), int(self.colN / len(self.text_input)))

    "Outputs occupancy grid to terminal"
    def printOG(self):
        for row in self.occupancy_grid:
            print(row)

    "Displays the Occupancy Grid as an image for easy visualization"
    def displayGrid(self):
        Image.fromarray(self.occupancy_grid).show()

    def getOGGrid(self):
        return self.occupancy_grid
