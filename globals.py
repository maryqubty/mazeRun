#Shared Globals:
mazeUrl, mazePcd, originalMaze, depth_axis, wall_points= None, None, None, None, None

def setUrl(url):
    global mazeUrl
    mazeUrl=url

def set_Pcd(pcd):
    global mazePcd
    mazePcd= pcd

def set_Original_Maze(maze):
    global originalMaze
    originalMaze= maze

def set_depth_axis(axis):
    global depth_axis
    depth_axis= axis

def set_wall_points(points):
    global wall_points
    wall_points= points
