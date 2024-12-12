import numpy as np
from sklearn.cluster import KMeans
import globals

def identify_depth_axis(mazePcd):
    #Now we're going to identify walls and open areas (roads) in a the maze based on depth differences
    #Identify depth axis based on variance - for each different maze model:
    point_coords = np.asarray(mazePcd.points)
    axis_variances = np.var(point_coords, axis=0)
    depth_axis = np.argmin(axis_variances)  # Axis with least variance is the depth
    print("Depth axis:", depth_axis)
    # Analyze depth distribution to dynamically adjust thresholds
    depth_values = np.asarray(mazePcd.points)[:, depth_axis]
    return depth_axis, point_coords, depth_values


def analyze_depth_distribution(mazePcd, depth_axis):
    # Analyze depth distribution to dynamically adjust thresholds
    depth_values = np.asarray(mazePcd.points)[:, depth_axis]
    min_depth, max_depth = np.min(depth_values), np.max(depth_values)

    #Instead of fixed percentiles, analyzing the depth distribution dynamically: we're using clustering with KMeans to segment depth dynamically
    depth_values_reshaped = depth_values.reshape(-1, 1)
    kmeans = KMeans(n_clusters=3, random_state=0).fit(depth_values_reshaped)
    cluster_centers = sorted(kmeans.cluster_centers_.flatten())
    wall_depth_threshold, road_depth_threshold, ground_depth_threshold = cluster_centers

    print("Dynamically calculated thresholds:")
    print("Wall:", wall_depth_threshold)
    print("Road:", road_depth_threshold)
    print("Ground:", ground_depth_threshold)

    return wall_depth_threshold



def map_walls_points(depth_axis, walls_depth_threshold, point_coords, depth_values):
    '''
    Axis Selection:
    Axis 1 (e.g., Y in many systems): Depth values increase from bottom to top.
    Axis 2 (e.g., Z in many systems): Depth values may increase in the opposite direction (top to bottom or front to back).
    Axis 3 : Left-To-Right.
    '''
    #If the axis direction is inverted, invert the depth values before clustering:
    if depth_axis == 2:  # Assuming Axis 2 needs to be inverted
        depth_values = -depth_values
    # Map categories to ensure correct visualization
    walls_points = point_coords[depth_values < walls_depth_threshold]
    return walls_points
