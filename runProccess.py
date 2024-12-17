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
