import cv2
import numpy as np


#blue
lower_start_color = 100
upper_start_color = 130
start_final_color = [255,0,0]

#green
lower_dest_color = 20
upper_dest_color = 100
dest_final_color = [0,255,0]


grid_size = 50




def circleDetector(image):
    """ Simple OpenCV function for circle detection
        - detects edges, applies threshold to binary image space 
        - then find object countours 
        - then returns the center of the detected circle
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # get gray image
    img_edges = cv2.Canny(gray,  50, 190, 3) # detect edges
    #cv2.imshow('img_edges', img_edges)
    ret, img_thresh = cv2.threshold(img_edges, 254, 255, cv2.THRESH_BINARY) # convert to binary images
    #cv2.imshow('img_thresh', img_thresh)
    contours, _ = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # get object contours

    # draw contour (see opencv tutorials)
    min_radius, max_radius = 15, 150
    centers=np.array([])
    #radii = np.array([])
    radii = 0;
    for c in contours:
        (x, y), radius = cv2.minEnclosingCircle(c)
        radius = int(radius)
#        if (radius > min_radius) and (radius < max_radius): # Take only the valid circle(s)
#            if(len(centers) == 0):
#                centers = np.array([[x,y]])
#            else:
#                centers = np.append(centers,[[x, y]],axis=0)
#            radii = np.append(radii,[radius])
        if(radius > min_radius) and (radius < max_radius) and radius > radii:
            centers = np.array([int(x),int(y)])
            radii = radius

    #cv2.imshow('contours', img_thresh)
    return centers,radii


def detectCircles(image):
    hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_start = np.array([lower_start_color,150,150])
    upper_start = np.array([upper_start_color,255,255])

    lower_dest = np.array([lower_dest_color, 150,0])
    upper_dest = np.array([upper_dest_color,255,255])

    start_mask = cv2.inRange(hsv_image, lower_start, upper_start)
    dest_mask = cv2.inRange(hsv_image, lower_dest, upper_dest)

    lower_white = np.array([0,0,100])
    upper_white = np.array([255,40,255])
    white_mask = cv2.inRange(hsv_image,lower_white,upper_white)


    output_image = image.copy()
    output_image[:,:,:] = 255
    output_image[np.where(start_mask==255)] = start_final_color
    cv2.imwrite("blue_only.jpg",output_image)
    centers,radii = circleDetector(output_image)
    if(len(centers) == 0):
        return image, [], [], 0, 0
    #centers = (int(np.mean(centers[:,0])),int(np.mean(centers[:,1])))

    output_image2 = image.copy()
    output_image2[:,:,:] = 255
    output_image2[np.where(dest_mask==255)] = dest_final_color
    cv2.imwrite("green_only.jpg",output_image2)
    centers2,radii2 = circleDetector(output_image2)
    if(len(centers2) == 0):
        return image, [], [], 0, 0
    #centers2 = (int(np.mean(centers2[:,0])),int(np.mean(centers2[:,1])))

    #output_image[np.where(dest_mask==255)] = dest_final_color
    #cv2.circle(output_image,centers,int(max(radii)),(0,0,255),10)
    #cv2.circle(output_image,centers2,int(max(radii2)),(0,0,255),10)

    #return centers, (int(max(radii)), int(max(radii2)))

    #output_image[np.where(dest_mask==255)] = dest_final_color
    cv2.circle(image,centers,int(radii),(0,0,255),5)
    cv2.circle(image,centers2,int(radii2),(0,0,255),5)
    return image, centers, centers2, radii, radii2

    #cv2.imwrite("circle_output.jpg",output_image)
