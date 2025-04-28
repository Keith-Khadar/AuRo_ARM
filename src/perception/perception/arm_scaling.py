import cv2
import numpy as np
import matplotlib.pyplot as plt
from .pathing import pathing_dfs, pathing_bfs
from .grid import toBitmapGrid

RES_MULT = 90
ARUCO_SZ = 0.984252 # 25mm
PAGE_MARGIN = 0.39

def homography(image):    
    # Predefined based on 8.5 x 11 printer paper. Units in inches. Top left corners of ArUco Markers on vertical sheet.
    height = 11 * RES_MULT
    width = 8.5 * RES_MULT
    pts_world = np.array([
        [0 + PAGE_MARGIN + ARUCO_SZ, 0 + PAGE_MARGIN + ARUCO_SZ],               # Top left (0)
        [width - (PAGE_MARGIN + ARUCO_SZ), PAGE_MARGIN + ARUCO_SZ],             # Top right (1)
        [0 + (PAGE_MARGIN + ARUCO_SZ), height - (PAGE_MARGIN + ARUCO_SZ)],      # Bottom left (2)
        [width - (ARUCO_SZ + PAGE_MARGIN), height - (PAGE_MARGIN + ARUCO_SZ)]   # Bottom right (3)
    ], dtype=np.float32)


    # Obtain coordinates using ArUco marker detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    aruco_dic = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    corners, ids, rejects = cv2.aruco.detectMarkers(gray, aruco_dic)
    if (len(corners) > 3):
        print("ArUco Markers detected")
        image_aruco = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
        #cv2.imshow("ArUco Markers in Image", image_aruco)
        for i in range(len(corners)):
            #print(corners[i][0], '\n')
            pass
    else:
        print("ERROR: Failed to detect markers")
        return image

    pts_image = np.array([
        corners[3][0][2],
        corners[1][0][3], 
        corners[2][0][1],
        corners[0][0][0]
    ], dtype=np.float32)

    # Perform homography
    H, retVal = cv2.findHomography(pts_image, pts_world)
    image_scaled = cv2.warpPerspective(image, H, ((int)(width - (2*PAGE_MARGIN+ARUCO_SZ)), (int)(height - (2*PAGE_MARGIN+ARUCO_SZ))))
    return image_scaled
    
if __name__ == '__main__' : 
    image = cv2.imread("data/sample2.jpg")
    result = mazeHomography(image)
    #cv2.imshow("Source", image)
    #cv2.imshow("Scaled", result)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # (Optional): See input image at bitmap resolution
    #grid.toGrid(image_scaled)
    
    # Testing grid with start and end
    image2 = cv2.imread("data/point_test.jpg")
    image2_bitmap, image2_start, image2_end = toBitmapGrid(image2)
    np.savetxt('bitmap.txt', image2_bitmap, fmt='%d')
    
    image2_dfs_bitmap, image2_dfs_dir = pathing_dfs(image2_bitmap, image2_start, image2_end)
    np.savetxt('dfs_bitmap.txt', image2_dfs_bitmap, fmt='%d')

    image2_bfs_bitmap, image2_bfs_dir = pathing_bfs(image2_bitmap, image2_start, image2_end)
    np.savetxt('bfs_bitmap.txt', image2_bfs_bitmap, fmt='%d')    
