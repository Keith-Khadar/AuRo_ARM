import cv2
import numpy as np
from .detectCircles import detectCircles

lower_black = np.array([0,0,0])
upper_black = np.array([180, 255, 100])

lower_white = np.array([0,0,100])
upper_white = np.array([255,40,255])

## Change based on minimum arm movement
grid_size = 25

''' Converts post-homography image to map.
    Obstacles(1) and free space(0) are just a bit map.
    Start and End points are marked as 3 and 4 using detectCircles()
''' 
def toBitmapGrid(image, start_center, end_center, start_radius, dest_radius):
    
    if start_center is None or end_center is None:
        print("Error: Could not detect start and end markers")
        return None

    cv2.circle(image,start_center,int(start_radius) + 5,(255,255,255),-1)
    cv2.circle(image,end_center,int(dest_radius) + 5,(255,255,255),-1)
    
    start_center_x, start_center_y = start_center
    end_center_x, end_center_y = end_center

    start_grid_x = start_center_x//grid_size
    start_grid_y = start_center_y//grid_size
    end_grid_x = end_center_x//grid_size
    end_grid_y = end_center_y//grid_size
    
    print(f"start center: ({start_center_x}, {start_center_y})")
    print(f"end center: ({end_center_x}, {end_center_y})")

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
    obstacle_mask = cv2.inRange(hsv_image, lower_black, upper_black)
    #obstacle_mask = cv2.bitwise_not(white_mask)
    #obstacle_mask = cv2.inRange(grey_image, 0, 35)

    rows = image.shape[0]//grid_size
    cols = image.shape[1]//grid_size
    bitmask = np.zeros((rows,cols), dtype=np.uint8)

    for i in range(rows):
        for j in range(cols):
            grid_cell = obstacle_mask[i*grid_size: (i+1) * grid_size,
                                      j*grid_size: (j+1) * grid_size]
            
            if np.any(grid_cell == 255):
                bitmask[i, j] = 1
    
    bitmask[start_grid_y, start_grid_x] = 2
    bitmask[end_grid_y, end_grid_x] = 3

    return bitmask, (start_grid_x, start_grid_y), (end_grid_x, end_grid_y)

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
