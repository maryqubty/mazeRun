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