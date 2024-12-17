import open3d as o3d
import numpy as np
import tkinter as tk
from tkinter import Button
from file_loader import select_model
import globals
from runProccess import run_walls_proccess
from sklearn.neighbors import NearestNeighbors


# Function to visualize road points
def visualize_roads():
    #points= globals.walls_points
    url= globals.mazeUrl
    if(url is None):
        print("No model was selected!")
        return
    #if reached here, then we do have a model and pcd
    walls_points= globals.wall_points
    if len(walls_points) == 0:
        print("No walls points to display.")
    else:
        walls_pcd = o3d.geometry.PointCloud()
        walls_pcd.points = o3d.utility.Vector3dVector(walls_points)
        o3d.visualization.draw_geometries([walls_pcd], window_name="walls Points")

def run_proccess():
    run_walls_proccess()

def create_gui():
    # Create a tkinter window
    window = tk.Tk()
    window.title("Maze Segmentation")
    # Set the size of the window (width x height)
    window.geometry("400x300")  # Example size, adjust as needed

    btn_s = Button(window, text="Select a Model", command=select_model)
    btn_s.pack(pady=20)  # Adjust padding as needed

    btn_model = Button(window, text="Proccess model", command=run_proccess)
    btn_model.pack(pady=20)  # Adjust padding as needed

    # Add buttons to the window
    btn_walls = Button(window, text="Show cloud-points of walls", command=visualize_roads)
    btn_walls.pack(pady=20)  # Adjust padding as needed

    # Becky's Update
    # Grid Visualization Button
    btn_grid = Button(window, text="Show 3D Grid Visualization", command=process_and_visualize_maze)
    btn_grid.pack(pady=20)

    # Run the tkinter main loop
    window.mainloop()








# # /////////////////////////////Becky's Update/////////////////////////////////

def classify_points_based_on_density_and_depth(pcd, radius=0.2, density_threshold=50, depth_threshold=-0.5):
    """
    Classifies points into walls and roads based on local point density and depth values.

    Parameters:
    - pcd: Open3D point cloud object.
    - radius: Distance radius to consider nearby points.
    - density_threshold: Number of points in the neighborhood to classify as a wall.
    - depth_threshold: Z-axis depth value threshold to assist in classification.

    Returns:
    - walls_points: Points classified as walls.
    - roads_points: Points classified as roads.
    """
    points = np.asarray(pcd.points)  # Convert point cloud to numpy array
    nbrs = NearestNeighbors(radius=radius)  # Use nearest neighbors search for density calculation
    nbrs.fit(points)

    wall_points = []
    road_points = []

    # Check the density and depth for each point
    for i, point in enumerate(points):
        # Find the neighbors within the specified radius
        indices = nbrs.radius_neighbors([point], radius)[1][0]

        # Check based on depth (Z-axis)
        if point[2] < depth_threshold:
            # If depth is less than threshold, classify as wall
            wall_points.append(point)
        elif len(indices) > density_threshold:
            # If point has more nearby points, classify as a wall (high density)
            wall_points.append(point)
        else:
            # Otherwise, classify as road (low density)
            road_points.append(point)

    print(f"Identified {len(wall_points)} wall points, {len(road_points)} road points.")
    return np.array(wall_points), np.array(road_points)


def visualize_road_wall_classification(walls_points, roads_points):
    """
    Visualizes walls and roads using Open3D with distinct colors.
    """
    if len(walls_points) == 0:
        print("No walls points to display.")
        return

    # Create Open3D point clouds for visualization
    walls_pcd = o3d.geometry.PointCloud()
    walls_pcd.points = o3d.utility.Vector3dVector(walls_points)
    walls_pcd.paint_uniform_color([1, 0, 0])  # Red color for walls

    roads_pcd = o3d.geometry.PointCloud()
    roads_pcd.points = o3d.utility.Vector3dVector(roads_points)
    roads_pcd.paint_uniform_color([0, 1, 0])  # Green color for roads

    # Visualize walls (red) vs roads (green)
    o3d.visualization.draw_geometries([walls_pcd, roads_pcd], window_name="Walls (Red) vs Roads (Green)")


def process_and_visualize_maze():
    """
    Main function to classify and visualize walls and roads.
    """
    maze_pcd = globals.mazePcd  # Make sure this is properly loaded and accessible

    if maze_pcd is None:
        print("No point cloud loaded! Please select and process a model first.")
        return

    print(f"Loaded point cloud with {len(maze_pcd.points)} points.")

    # Step 1: Classify points into walls and roads based on density and depth
    walls_points, roads_points = classify_points_based_on_density_and_depth(maze_pcd)

    if len(walls_points) == 0:
        print("No walls points were classified.")
        return

    # Step 2: Visualize the classification results
    visualize_road_wall_classification(walls_points, roads_points)
