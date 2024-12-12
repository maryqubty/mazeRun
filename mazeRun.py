import open3d as o3d 
import numpy as np
import tkinter as tk
from tkinter import Button
from tkinter import filedialog


#Main Code
# Prompt user to select a PLY file
file_path = filedialog.askopenfilename(
    title="Select a PLY File", 
    filetypes=[("PLY Files", "*.ply")]
)
if not file_path:
    print("No file selected. Exiting.")
    exit()
# Load the selected PLY file
mazeUrl = file_path
# Load the PLY file as a mesh
originalMaze = o3d.io.read_triangle_mesh(mazeUrl)
# Visualize the mesh
originalMaze.compute_vertex_normals()

# Sample point clouds from the mesh
mazePcd= originalMaze.sample_points_uniformly(number_of_points=50000)

#Now we're going to identify walls and open areas (roads) in a the maze based on depth differences
#Identify depth axis based on variance - for each different maze model:
point_coords = np.asarray(mazePcd.points)
axis_variances = np.var(point_coords, axis=0)
depth_axis = np.argmin(axis_variances)  # Axis with least variance is the depth
print("Depth axis:", depth_axis)
# Get depth values
mazeDepth = np.asarray(mazePcd.points)[:, depth_axis] #depth_axis is the depth axis
#save the sorted depth values in an output file - sorted
np.savetxt("sorted_depth_values.txt", np.sort(mazeDepth), delimiter=",")

# Analyze depth distribution to dynamically adjust thresholds
depth_values = np.asarray(mazePcd.points)[:, depth_axis]
min_depth, max_depth = np.min(depth_values), np.max(depth_values)

#Instead of fixed percentiles, analyze the depth distribution dynamically: Use clustering with KMeans to segment depth dynamically
from sklearn.cluster import KMeans

depth_values_reshaped = depth_values.reshape(-1, 1)
kmeans = KMeans(n_clusters=3, random_state=0).fit(depth_values_reshaped)
cluster_centers = sorted(kmeans.cluster_centers_.flatten())
road_depth_threshold, wall_depth_threshold, ground_depth_threshold = cluster_centers

print("Dynamically calculated thresholds:")
print("Road:", road_depth_threshold)
print("Wall:", wall_depth_threshold)
print("Ground:", ground_depth_threshold)

# Map categories to ensure correct visualization
road_points = point_coords[depth_values <= road_depth_threshold]
wall_points = point_coords[
    (depth_values > road_depth_threshold) & (depth_values <= wall_depth_threshold)
]
ground_points = point_coords[depth_values > wall_depth_threshold]


# Create a tkinter window
window = tk.Tk()
window.title("Maze Segmentation")
# Set the size of the window (width x height)
window.geometry("400x300")  # Example size, adjust as needed

# Function to visualize wall points
def visualize_walls():
    if len(wall_points) == 0:
        print("No wall points to display.")
    else:
        wall_pcd = o3d.geometry.PointCloud()
        wall_pcd.points = o3d.utility.Vector3dVector(wall_points)
        o3d.visualization.draw_geometries([wall_pcd], window_name="Wall Points")

# Function to visualize road points
def visualize_roads():
    if len(road_points) == 0:
        print("No road points to display.")
    else:
        road_pcd = o3d.geometry.PointCloud()
        road_pcd.points = o3d.utility.Vector3dVector(road_points)
        o3d.visualization.draw_geometries([road_pcd], window_name="Road Points")

# Function to visualize ground points
def visualize_ground():
    if len(ground_points) == 0:
        print("No ground points to display.")
    else:
        ground_pcd = o3d.geometry.PointCloud()
        ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
        o3d.visualization.draw_geometries([ground_pcd], window_name="Ground Points")

# Add buttons to the window
btn_walls = Button(window, text="Show Walls", command=visualize_walls)
btn_walls.pack(pady=20)  # Adjust padding as needed

btn_roads = Button(window, text="Show Roads", command=visualize_roads)
btn_roads.pack(pady=20)  # Adjust padding as needed

btn_ground = Button(window, text="Show Ground", command=visualize_ground)
btn_ground.pack(pady=20)  # Adjust padding as needed

# Run the tkinter main loop
window.mainloop()

'''
# Segment based on depth threshold
#road trip threshold is based on the lowest depth value of the maze
road_depth_threshold = np.min(mazeDepth)+ 3
print("road depth threshold: ", road_depth_threshold)
#wall depth threshold is based on the average depth value of the maze
wall_depth_threshold = np.mean(mazeDepth)
print("wall depth threshold: ", wall_depth_threshold)
#ground threshold is based on the highest depth value of the maze
ground_depth_threshold = np.max(mazeDepth) 
print("ground depth threshold: ", ground_depth_threshold)

walls = np.where((mazeDepth > wall_depth_threshold) & (mazeDepth < ground_depth_threshold))[0]
roads = np.where(mazeDepth <= road_depth_threshold)[0]
# Create point clouds for walls and roads
walls_pcd = o3d.geometry.PointCloud()
walls_pcd.points = o3d.utility.Vector3dVector(np.asarray(mazePcd.points)[walls, :])
#this next command doesnt work, i # for now
#walls_pcd.paint_uniform_color([1, 0, 0])  # Red for walls

roads_pcd = o3d.geometry.PointCloud()
roads_pcd.points = o3d.utility.Vector3dVector(np.asarray(mazePcd.points)[roads, :])
#this next command doesnt work, i # for now
#roads_pcd.paint_uniform_color([0, 0, 1])  # Blue for roads

# Print the points of maze_pcd to determine which axis is the depth axis
walls_points = np.asarray(walls_pcd.points)
sorted_points = walls_points[np.argsort(walls_points[:, 1])]


# Visualize the segmented point clouds of roads
#o3d.visualization.draw_geometries([roads_pcd], window_name="points cloud Roads")
# visualizing the roads mesh
#roadsMesh = convert_to_mesh(roads_pcd)
#o3d.visualization.draw_geometries([roadsMesh], window_name="mesh of roads")
# Visualize the segmented point clouds of walls
#o3d.visualization.draw_geometries([walls_pcd], window_name="points cloud Walls")
# visualizing the walls mesh
#wallsMesh = convert_to_mesh(walls_pcd)
#o3d.visualization.draw_geometries([wallsMesh], window_name="mesh of walls")


#now after preparing the point clouds and the meshes of the walls and roads, we can add buttons for more clear visualization:
# Create a tkinter window
window = tk.Tk()
window.title("Maze Segmentation")
# Set the size of the window (width x height)
window.geometry("400x300")  # Example size, adjust as needed

#create buttons for the walls and roads
#button for maze mesh
####maze_button = Button(window, text="Maze Mesh", command=lambda: o3d.visualization.draw_geometries([originalMaze], window_name="Maze Mesh"))
####maze_button.pack()
#button for maze point cloud
maze_pcd_button = Button(window, text="Maze Point Cloud", command=lambda: o3d.visualization.draw_geometries([mazePcd], window_name="Maze Point Cloud"))
maze_pcd_button.pack()
#button for roads point cloud
roads_button = Button(window, text="Roads Point Cloud", command=lambda: o3d.visualization.draw_geometries([roads_pcd], window_name="Roads Point Cloud"))
roads_button.pack()
#button for roads mesh
####roads_mesh_button = Button(window, text="Roads Mesh", command=lambda: o3d.visualization.draw_geometries([roadsMesh], window_name="Roads Mesh"))
####roads_mesh_button.pack()
#button for walls point cloud
walls_button = Button(window, text="Walls Point Cloud", command=lambda: o3d.visualization.draw_geometries([walls_pcd], window_name="Walls Point Cloud"))
walls_button.pack()
#button for walls mesh
####walls_mesh_button = Button(window, text="Walls Mesh", command=lambda: o3d.visualization.draw_geometries([wallsMesh], window_name="Walls Mesh"))
####walls_mesh_button.pack()
# Run the tkinter main loop
window.mainloop()
'''
#end of the code for now