import open3d as o3d
from tkinter import filedialog
import globals

def select_model():
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
    globals.setUrl(mazeUrl)
    # Load the PLY file as a mesh
    originalMaze = o3d.io.read_triangle_mesh(mazeUrl)
    globals.set_Original_Maze(originalMaze)
    # Visualize the mesh
    originalMaze.compute_vertex_normals()

    # Sample point clouds from the mesh
    mazePcd= originalMaze.sample_points_uniformly(number_of_points=50000)
    globals.set_Pcd(mazePcd)
