# Auxiliary code for simulation of UAV trajectories

from matplotlib.path import Path
import numpy as np

def uav_Marker(angle: float) -> Path:
    """
    Outputs a UAV Maker as a Matplotlib path rotated an angle "angle" [degrees]. It looks as the head of and arrow
    """

    # Global scale parameter for the marker
    scale = 1

    # 2D Points to define each of the marker vertices
    points = np.transpose(np.array(
        [
            [0, 0], #center
            [scale * 0.65, - scale * 0.5], #lower right 
            [0, scale], #up
            [ - scale * 0.65, - scale * 0.5], #lower left  

        ]
    ))

    # Generic 2D Rotation Matrix
    mat_rot =  np.array(
        [
            [np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))],
            [-np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle))],
        ]
    )

    # Rotate points
    rot_points = np.transpose(np.matmul(mat_rot, points))

    # Define the Path with rotated points and the correct codes
    return Path(rot_points,[
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO, 
        Path.LINETO, 
    ])

def get_Path_Points(pos_i: np.ndarray, pos_f: np.ndarray, uav_speed: float, dt: float) -> np.ndarray:
    """
    Divides the a segment defined by two points pos_f and pos_i based on UAV speed and sim speed. Outputs the corresponding coordinates
    as an Nx3 or Nx2 array
    """
    dx = uav_speed * dt # Spatial step that the UAV takes in one time unit

    delta_pos = pos_f - pos_i
    d = np.linalg.norm(delta_pos)

    # If the initial and final point are too close, just return the final point
    if d<= dx:
        return np.array([pos_f])

    # Get the number of point as the maximum number of dx length that fit within d
    n_points = int(np.floor(np.linalg.norm(delta_pos) / dx))

    # Return an array with all the points corresponding to taking n_points step of length dx along the segment
    # DOES NOT contain the initial point

    #print(np.array([pos_i + (i+1) * dx*delta_pos/d for i in range(n_points)]))
    #print(np.array([pos_f]))

    return np.concatenate((np.array([pos_i + (i+1) * dx*delta_pos/d for i in range(n_points)]), np.array([pos_f])))


