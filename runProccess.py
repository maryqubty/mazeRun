import globals
import depth_analysis

def run_walls_proccess():
    mazePcd= globals.mazePcd
    depth_axis, point_coords, depth_values= depth_analysis.identify_depth_axis(mazePcd)
    walls_depth_threshold= depth_analysis.analyze_depth_distribution(mazePcd, depth_axis)
    walls_points= depth_analysis.map_walls_points(depth_axis, walls_depth_threshold, point_coords, depth_values)
    #update the global variables for further uses:
    globals.set_depth_axis(depth_axis)
    globals.set_wall_points(walls_points)
    return walls_points



# /////////////////// Updated Process Function /////////////////////
# def run_walls_proccess():
    """
    Main function to identify walls from the 3D point cloud, generate an occupancy grid,
    and update global variables accordingly.
    """
    mazePcd = globals.mazePcd
    if mazePcd is None:
        print("Error: No point cloud data (mazePcd) found in globals. Exiting process.")
        return None

    # Step 1: Identify the depth axis and analyze points
    depth_axis, point_coords, _ = depth_analysis.identify_depth_axis(mazePcd)

    # Step 2: Downsample to reduce the number of points
    print("Downsampling wall points for efficiency...")
    downsampled_points = downsample_points(point_coords, voxel_size=0.1)

    # Recalculate depth values for the downsampled points
    downsampled_depth_values = downsampled_points[:, depth_axis]

    # Step 3: Analyze depth distribution and map wall points
    walls_depth_threshold = depth_analysis.analyze_depth_distribution(mazePcd, depth_axis)
    walls_points = depth_analysis.map_walls_points(
        depth_axis, walls_depth_threshold, downsampled_points, downsampled_depth_values
    )

    if walls_points.shape[0] == 0:
        print("Warning: No wall points detected. Exiting process.")
        return None

    # Step 4: Generate the grid and update global variables
    try:
        grid = generate_grid_from_points(walls_points, grid_resolution=0.1)
        globals.set_grid(grid)

        # Update other global variables
        globals.set_depth_axis(depth_axis)
        globals.set_wall_points(walls_points)
    except Exception as e:
        print(f"Error generating occupancy grid: {e}")
        return None

    print("Walls process completed successfully.")
    return walls_points


# ///////////////// Updated Function //////////////////////////////
# def generate_grid_from_points(wall_points, grid_resolution=0.1):
    """
    Generate a 3D occupancy grid representation from wall points.

    Parameters:
    - wall_points (ndarray): Nx3 array of wall point coordinates.
    - grid_resolution (float): Size of each grid cell.

    Returns:
    - grid (ndarray): 3D occupancy grid.
    """
    if wall_points is None or wall_points.shape[0] == 0:
        raise ValueError("Input wall_points is empty. Cannot generate a grid.")

    # Step 1: Determine bounds of the grid
    min_coords = np.min(wall_points, axis=0)
    max_coords = np.max(wall_points, axis=0)

    # Step 2: Define grid size
    grid_size = np.ceil((max_coords - min_coords) / grid_resolution).astype(int)
    print(f"Grid size determined: {grid_size}")

    if np.any(grid_size <= 0):
        raise ValueError("Grid dimensions must be positive. Check grid_resolution and input wall points.")

    # Step 3: Initialize an empty grid
    grid = np.zeros(grid_size, dtype=np.int8)

    # Step 4: Map wall points to grid indices safely
    try:
        indices = ((wall_points - min_coords) / grid_resolution).astype(int)

        # Sanity check for valid indices
        valid_mask = np.all((indices >= 0) & (indices < grid_size), axis=1)
        valid_indices = indices[valid_mask]

        for idx in valid_indices:
            grid[idx[0], idx[1], idx[2]] = 1  # Mark wall cells
    except Exception as e:
        raise RuntimeError(f"Failed to map points to the grid: {e}")

    print("3D occupancy grid generation completed.")
    return grid
