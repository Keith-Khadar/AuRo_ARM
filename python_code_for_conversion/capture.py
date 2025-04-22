import cv2

from detectCircles import detectCircles
from grid import toGrid

# Open the default camera
cam = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

start_center_prev = []
dest_center_prev = []
count = 0

while True:
    ret, frame = cam.read()

    ##
    ## Add homography preprocessing here
    ##

    # Find start and dest points
    frame, start_center, dest_center, start_radius, dest_radius = detectCircles(frame)

    #Display image for debugging
    cv2.imshow('Camera', frame)

    # This code checks for green and blue circles that havent moved for twenty frames
    # Which will indicate user is done drawing and arm should start moving
    if(len(start_center_prev) > 0 and len(dest_center_prev) > 0):
        if(len(start_center) > 0 and len(dest_center)>0):
            if(abs(start_center_prev[0]-start_center[0]) <= 10 and abs(start_center_prev[1]-start_center[1]) <= 10 and 
               abs(dest_center_prev[0]-dest_center[0]) <= 10 and abs(dest_center_prev[1]-dest_center[1]) <= 10):
                count = count + 1
                print(count)
                if(count > 20):


                    ## Convert image to bitmap/path - edit to add return value
                    toGrid(frame)

                    ## Add arm movement/publications here

                    ## Add some sort of waiting condition to tell the arm to reset and get ready to draw again

            else:
                count = 0
                print("reset")
                print(abs(start_center_prev[0]-start_center[0]), abs(start_center_prev[1]-start_center[1]),
                      abs(dest_center_prev[0]-dest_center[0]),abs(dest_center_prev[1]-dest_center[1]))
            start_center_prev = start_center
            dest_center_prev = dest_center

    if(len(start_center) > 0 and len(dest_center) > 0):
        start_center_prev = start_center
        dest_center_prev = dest_center


    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
#out.release()
cv2.destroyAllWindows()
