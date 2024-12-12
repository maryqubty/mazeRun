import open3d as o3d 
import tkinter as tk
from tkinter import Button
from file_loader import select_model
import globals
from runProccess import run_walls_proccess

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

    btn_roads = Button(window, text="Select a Model", command=select_model)
    btn_roads.pack(pady=20)  # Adjust padding as needed

    btn_roads = Button(window, text="Proccess model", command=run_proccess)
    btn_roads.pack(pady=20)  # Adjust padding as needed

    # Add buttons to the window
    btn_roads = Button(window, text="Show cloud-points of walls", command=visualize_roads)
    btn_roads.pack(pady=20)  # Adjust padding as needed


    # Run the tkinter main loop
    window.mainloop()
