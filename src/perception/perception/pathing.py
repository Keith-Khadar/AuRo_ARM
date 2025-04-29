import numpy as np
from collections import deque

''' Performs depth first search (DFS) algorithm using input bitmap post image processing.
    Returns bitmap as well as directions for the arm to mimic.
    Directions are formatted in (dy,dx) where movement is by one grid square.
    Pathing is shown on bitmap as 4s
''' 
def pathing_dfs(bitmap, start, end):
    bitmap_dfs = bitmap.copy()
    rows, cols = bitmap.shape
    visited = np.zeros((rows, cols), dtype = bool)
    
    # Map for pathing later on
    parent = {} 

    paths = {
        (-1, 0),
        (1,0),
        (0,-1),
        (0, 1)
    }

    # In the beginning only start is initialized
    stack = [start]
    visited[start] = True

    while stack:
        x,y = stack.pop()
        
        # End reached
        if (x,y) == end:
            break
        
        # Else investigate other paths
        for (dy, dx) in paths:
            new_y = y + dy
            new_x = x + dx
            # Check boundary conditions
            if 0 <= new_y < rows and 0 <= new_x < cols:
                if not visited[new_y, new_x] and bitmap_dfs[new_y, new_x] != 1:
                    visited[new_y, new_x] = True
                    parent[(new_x, new_y)] = (x, y)
                    stack.append((new_x, new_y))

    # Get the actual path
    if end not in parent:
        print("DFS: Not actual path found\n")
        return bitmap_dfs, []
    
    path = []
    directions = []
    position = end

    while position != start:
        path.append(position)

        prev = parent[position]
        path_to_x = position[0] - prev[0]
        path_to_y = position[1] - prev[1]
        directions.append((path_to_y, path_to_x))
        
        position = prev
    
    path.append(start)
    path.reverse()
    directions.reverse()

    # Draw path on bitmap
    for (x,y) in path:
        bitmap_dfs[y,x] = 4

    return bitmap_dfs, directions

''' Performs breadth first search (BFS) algorithm using input bitmap post image processing.
    Returns bitmap as well as directions for the arm to mimic.
    Directions are formatted in (dy,dx) where movement is by one grid square.
    Pathing is shown on bitmap as 4s
''' 
def pathing_bfs(bitmap, start, end):
    rows, cols = bitmap.shape
    visited = np.zeros((rows, cols), dtype = bool)
    bitmap_bfs = bitmap.copy()

    # Map for pathing later on
    parent = {} 

    paths = {
        (-1, 0),
        (1,0),
        (0,-1),
        (0, 1)
    }

    # In the beginning only start is initialized
    queue = deque()
    queue.append((start))
    visited[start] = True

    while queue:
        x,y = queue.popleft()

        # End reached
        if (x,y) == end:
            break
        
        # Else investigate other paths
        for (dy, dx) in paths:
            new_y = y + dy
            new_x = x + dx
            # Check boundary conditions
            if 0 <= new_y < rows and 0 <= new_x < cols:
                if not visited[new_y, new_x] and bitmap_bfs[new_y, new_x] != 1:
                    visited[new_y, new_x] = True
                    parent[(new_x, new_y)] = (x, y)
                    queue.append((new_x, new_y))

    # Get the actual path
    if end not in parent:
        print("BFS: Not actual path found\n")
        return bitmap_bfs, []
    
    path = []
    directions = []
    position = end

    while position != start:
        path.append(position)

        prev = parent[position]
        path_to_x = position[0] - prev[0]
        path_to_y = position[1] - prev[1]
        directions.append((path_to_y, path_to_x))
        
        position = prev
    
    path.append(start)
    path.reverse()
    directions.reverse()

    # Draw path on bitmap
    for (x,y) in path:
        bitmap_bfs[y,x] = 4

    return bitmap_bfs, directions

map = [[0,0,0],[0,0,0],[0,1,0],[0,0,0],[0,0,0]]

pathing_bfs(map, (0,0), (2,4))