import cv2
import numpy as np

#blue
lower_start_color = 90
upper_start_color = 130
start_final_color = [255,0,0]

lower_dest_color = 45
upper_dest_color = 80
dest_final_color = [0,255,0]

lower_white = np.array([0,0,100])
upper_white = np.array([255,40,255])

##
## Change based on minimum arm movement
##

grid_size = 50

def toGrid(image):
    hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(hsv_image,lower_white,upper_white)
    output_image = np.zeros(image.shape)
    output_image[np.where(white_mask==255)] = [255,255,255]

    ##
    ## Change this to make a bitmap of obstacles instead of an image
    ##
    ## This was for testing - logic needs to be edited to include border
    ## Will probably need to add start/dest points and radii arguments 
    ## so the start/dest aren't included as obstacles
    ##

    grid_image = image.copy()
    grid_image[:][:][:] = 255

    for i in range(0,image.shape[0]-grid_size,grid_size):
        for j in range(0,image.shape[1]-grid_size,grid_size):
            grid_clear = True
            for x in range(i,i+grid_size):
                if(grid_clear == False):
                    break
                for y in range(j,j+grid_size):
                    if(np.all(output_image[x][y] == 0)):
                        grid_clear = False
                        break
            if(not grid_clear):
                for x in range(i,i+grid_size):
                    for y in range(j,j+grid_size):
                        grid_image[x][y] = [0,0,0]
    cv2.imwrite("grid_output.jpg",grid_image)               






cv2.destroyAllWindows()