import numpy as np
import cv2
import math
import os.path

# This is a start for the map program
prompt = '> '
print("What is the name of your floor plan you want to convert to a ROS map:") 
file_name = input(prompt)
print("You will need to choose the x coordinates horizontal with respect to each other")
print("Double Click the first x point to scale")

# Read in the image
image = cv2.imread(file_name)

# Some variables
ix, iy = -1, -1
x1 = [0, 0, 0, 0]
y1 = [0, 0, 0, 0]
font = cv2.FONT_HERSHEY_SIMPLEX

# Mouse callback function
def draw_point(event, x, y, flags, param):
    global ix, iy, x1, y1, sx, sy, image

    if event == cv2.EVENT_LBUTTONDBLCLK:
        ix, iy = x, y
        print(ix, iy)

        # Draw cross lines
        image[iy, ix] = (0, 0, 255)
        cv2.line(image, (ix + 2, iy), (ix + 10, iy), (0, 0, 255), 1)
        cv2.line(image, (ix - 2, iy), (ix - 10, iy), (0, 0, 255), 1)
        cv2.line(image, (ix, iy + 2), (ix, iy + 10), (0, 0, 255), 1)
        cv2.line(image, (ix, iy - 2), (ix, iy - 10), (0, 0, 255), 1)

        if x1[0] == 0:
            x1[0] = ix
            y1[0] = iy
            print('Double click a second x point')   
        elif x1[1] == 0:
            x1[1] = ix
            y1[1] = iy
            print("What is the x distance in meters between the 2 points?") 
            deltax = float(input(prompt))
            dx = math.sqrt((x1[1] - x1[0])**2 + (y1[1] - y1[0])**2) * 0.05
            sx = deltax / dx
            print("You will need to choose the y coordinates vertical with respect to each other")
            print('Double Click a y point')
        elif x1[2] == 0:
            x1[2] = ix
            y1[2] = iy
            print('Double click a second y point')
        else:
            print("What is the y distance in meters between the 2 points?") 
            deltay = float(input(prompt))
            x1[3] = ix
            y1[3] = iy    
            dy = math.sqrt((x1[3] - x1[2])**2 + (y1[3] - y1[2])**2) * 0.05
            sy = deltay / dy 
            print(sx, sy)
            res = cv2.resize(image, None, fx=sx, fy=sy, interpolation=cv2.INTER_CUBIC)
            cv2.imwrite("KEC_BuildingCorrected.pgm", res)
            cv2.imshow("Image2", res)

            print("What is the name of the new map?") 
            mapName = input(prompt)

            print("Where is the desired location of the map and yaml file?") 
            print("NOTE: if this program is not run on the TurtleBot, Please input the file location of where the map should be saved on TurtleBot. The file will be saved at that location on this computer. Please then transfer the files to TurtleBot.") 
            mapLocation = input(prompt)

            completeFileNameMap = os.path.join(mapLocation, mapName + ".pgm")
            completeFileNameYaml = os.path.join(mapLocation, mapName + ".yaml")

            with open(completeFileNameYaml, "w") as yaml:
                yaml.write(f"image: {mapLocation}/{mapName}.pgm\n")
                yaml.write("resolution: 0.050000\n")
                yaml.write("origin: [-1.0, -1.0, 0.000000]\n")
                yaml.write("negate: 0\n")
                yaml.write("occupied_thresh: 0.65\n")
                yaml.write("free_thresh: 0.196\n")

            cv2.imwrite(completeFileNameMap, res)
            exit()

# Set up window and mouse callback
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('image', draw_point)

# Loop until Esc is pressed
while True:
    cv2.imshow('image', image)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):
        print('Done')

cv2.destroyAllWindows()
