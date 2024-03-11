# This file contains several functions related to coordinates and their representation

import numpy as np
import requests
import utm
from urllib.request import urlopen
from io import BytesIO
from PIL import Image
import matplotlib.pyplot as plt
import time
import copy
from modules import dubins as DB

wait_time = 1  # Wait time in between API Request. This is too much brute force

def angle_to_x(v:np.ndarray):
    """
    Computes the angle (o to 2pi) that the vector v forms with x. Clockwise is positive
    """

    alpha = np.arccos(v[0] / np.linalg.norm(v))

    if v[1] < 0:
        alpha = 2 * np.pi - alpha

    return alpha

# ------------------------- Coordinates transformations ---------------------------------------

def latlon2utm(latlon: np.ndarray) -> tuple[np.ndarray, tuple]:
    """
    latitud-longitud to UTM coordinates conversion. It is based on the 'utm' module and it only
    manages shape details.
    Works with Nx2 or Nx3 arrays. If height is included, it is left unchanged.

    It is assumed that all points are within the same UTM Zone

    Inputs:
        - "lalton" contains the point or points of numerical positions with latlon coordinates
        
    Outputs:
        - An array with the same format as the input containing the UTM coordinates within the corresponding Zone
        - "UTM_Zone" is a tuple (Zone_Number: int, Zone_Letter: str)

    """

    # Check latlon shape to determine if it is a single point or more
    if latlon.shape == (2,):  # 1 point without height
                                                       # lat, lon
        E, N, Z_number, Z_letter = utm.from_latlon(latlon[0], latlon[1])
        return np.array([E, N]), (Z_number, Z_letter)
    
    elif latlon.shape == (3,): # 1 point with height
        E, N, Z_number, Z_letter = utm.from_latlon(latlon[0], latlon[1])
        return np.array([E, N, latlon[2]]), (Z_number, Z_letter)
    
    # Several points
    else: 
        E, N, Z_number, Z_letter = utm.from_latlon(latlon[:,0], latlon[:,1])
        if len(latlon[0]) > 2: # several points with heights
            return np.transpose(np.vstack((E, N, latlon[:,2]))), (Z_number, Z_letter)
        
        else: # several points without heights
            return np.transpose(np.vstack((E, N))), (Z_number, Z_letter)

def utm2latlon(utmcoord: np.ndarray, UTM_Zone: tuple) -> np.ndarray:
    """
    UTM to latitud-longitud coordinates conversion. It is based on the 'utm' module.
    Works with Nx2 or Nx3 arrays. If height is included, it is left unchanged.

    Inputs:
        - "utmcoord" contains the point or points of numerical positions within a UTM Zone
        - "UTM_Zone" is a tuple (Zone_Number: int, Zone_Letter: str)

    Outputs:
        - An array with the same format as the input containing the latitude-longitud conversion.
    """

    # Sanity checks?

    zone_number = UTM_Zone[0]
    zone_letter = UTM_Zone[1]

    # Check shape to determine if it is a single point or more
    if utmcoord.shape == (2,): # 1 point without height
        lat, lon = utm.to_latlon(utmcoord[0], utmcoord[1], zone_number, zone_letter)
        return np.array([lat, lon])
    elif utmcoord.shape == (3,): # 1 point with height
        lat, lon = utm.to_latlon(utmcoord[0], utmcoord[1], zone_number, zone_letter)
        return np.array([lat, lon, utmcoord[2]])
    
    # Several points
    else:
        lat, lon = utm.to_latlon(utmcoord[:,0], utmcoord[:,1], zone_number, zone_letter)
        if len(utmcoord[0]) > 2:  # several points with heights
            return np.transpose(np.vstack((lat, lon, utmcoord[:,2])))
        
        else: # several points without heights
            return np.transpose(np.vstack((lat, lon)))

def latlon2mercator(lat_deg: float, lon_deg:float, zoom:int) -> np.ndarray:
  """
  Transformation a single point from latitude-longitud (degrees) to Web mercator with an specific zoom level
  It outputs the x-y coordinates as an 2D vector.
  """
  lat_rad = np.radians(lat_deg)
  n = 2.0 ** zoom
  xtile = int((lon_deg + 180.0) / 360.0 * n)
  ytile = int((1.0 - np.log(np.tan(lat_rad) + (1 / np.cos(lat_rad))) / np.pi) / 2.0 * n)
  return np.array([xtile, ytile])

def mercator2latlon(xtile: int, ytile:int, zoom:int) -> np.ndarray:
  """
  Transformation a single point from Web Mercator to Latitude-Longitude (degrees) with an specific zoom level
  It outputs the latitude-longitude coordinates as an 2D vector.
  """
  n = 2.0 ** zoom
  lon_deg = xtile / n * 360.0 - 180.0
  lat_rad = np.arctan(np.sinh(np.pi * (1 - 2 * ytile / n)))
  #lat_rad = 2*np.arctan(np.exp(np.pi * (1 - 2 * ytile / n)))-np.pi/2 # It is the same
  lat_deg = np.degrees(lat_rad)
  return np.array([lat_deg, lon_deg])

# ------------------------- General auxiliary functions ---------------------------------------

def getBoundingBox(coordinates: np.ndarray) -> tuple[float, float, float, float]:
    """
    Computes the 2D bounding box for a list of 2D coordinates. It gives
    (xmin, xmax, ymin, ymax)
    """

    xmin = min(coordinates[:,0])
    xmax = max(coordinates[:,0])
    ymin = min(coordinates[:,1])
    ymax = max(coordinates[:,1])

    return xmin, xmax, ymin, ymax

def update_Height_Online(coords: np.ndarray):
    """
    Function to update a point's or list of points height using the online opentopodata.org API
    It takes an Nx3 array and updates the third component of each row.
    It assumes that the points are given by latitude-longitud.

    It does not output the array, it updates it inplace.
    """
    time.sleep(wait_time)
    url = 'https://api.opentopodata.org/v1/eudem25m?locations='

    # Check shape to determine if it is a single point or more
    # If it 2D, leave it as it is
    if coords.shape == (3,):
        # If it is a point 3D, then update the height

        # request url
        request = url + f"{coords[0]}"+','+f"{coords[1]}"+'|'
        # get response from the api
        #print(request)
        response = requests.get(request, timeout=10.0)
        
        # if the response is positive, update, if not, don't
        if 200 == response.status_code and "error" not in response.json():
            coords[2] = response.json()["results"][0]["elevation"]
        else: print("Height Online Update failed!")

    # Several points
    else:

        # If they are 2D, leave them as they are
        if coords.shape[1] == 3:
            
            # Sends the entire array points in one single request to speed things up
            towers_string = ""
            for col in coords:
                towers_string = towers_string+f"{col[0]}"+','+f"{col[1]}"+'|'

            # request url
            request = url + towers_string
            #print(request)
            response = requests.get(request, timeout=10.0)

            # if the response is positive, store heights in "elevation"
            elevations = np.zeros(len(coords))
            if 200 == response.status_code and "error" not in response.json():
                for k in range(len(coords)):
                    elevations[k] = response.json()["results"][k]["elevation"]
            else: print("Height Online Update failed!")

            # and update the original array
            coords[:,2] = elevations
            

    return None

def update_UTM_Height_Online(coords: np.ndarray, utmZone: tuple):
    """
    Function to update a point's or list of points height using the online opentopodata.org API
    It takes an Nx3 array and updates the third component of each row.
    It assumes that the points are given in UTM Coordinates.

    It does not output the array, it updates it inplace.
    """
    time.sleep(wait_time)

    url = 'https://api.opentopodata.org/v1/eudem25m?locations='

    coordslatlot = utm2latlon(coords, utmZone)

    # Check shape to determine if it is a single point or more
    # If it 2D, leave it as it is
    if coords.shape == (3,):
        # If it is a point 3D, then update the height

        # request url
        request = url + f"{coordslatlot[0]}"+','+f"{coordslatlot[1]}"+'|'
        # get response from the api
        #print(request)
        response = requests.get(request, timeout=10.0)
        
        # if the response is positive, update, if not, don't
        if 200 == response.status_code and "error" not in response.json():
            coords[2] = response.json()["results"][0]["elevation"]

        else: print("Height Online Update failed!")

    # Several points
    else:

        # If they are 2D, leave them as they are
        if coords.shape[1] == 3:
            
            # Sends the entire array points in one single request to speed things up
            towers_string = ""
            for col in coordslatlot:
                towers_string = towers_string+f"{col[0]}"+','+f"{col[1]}"+'|'

            # request url
            request = url + towers_string
            #print(request)
            response = requests.get(request, timeout=10.0)

            # if the response is positive, store heights in "elevation"
            elevations = np.zeros(len(coords))
            if 200 == response.status_code and "error" not in response.json():
                for k in range(len(coords)):
                    elevations[k] = response.json()["results"][k]["elevation"]
            else: print("Height Online Update failed!")

            # and update the original array
            coords[:,2] = elevations
            
    print(response.json())
    return None

def get_Path_Online_Elevations(point1_UTM: np.ndarray, point2_UTM: np.ndarray, utmZone: tuple, distance: float) -> list:
    """
    It uses the opentopodata API to sample the heights of a path given by point1 and point2. distance is the maximum distance
    between to sampling points.
    """
    time.sleep(wait_time)

    url = 'https://api.opentopodata.org/v1/eudem25m?locations='

    n_points = int(np.floor(np.linalg.norm(point2_UTM-point1_UTM) / distance) + 2) # Intermediate points + start and ending

    p1 = utm2latlon(point1_UTM, utmZone)
    p2 = utm2latlon(point2_UTM, utmZone)

    # request url
    request = url + f"{p1[0]}"+','+f"{p1[1]}"+'|'+f"{p2[0]}"+','+f"{p2[1]}"+f"&samples={n_points}"
    # get response from the api
    #print(request)
    response = requests.get(request, timeout=10.0)
    print(request)
    print(response.json())
    path = []
    # if the response is positive, update, if not, don't
    if 200 == response.status_code and "error" not in response.json():
        for point in response.json()["results"]:
            latlonz = np.array([point["location"]["lat"], point["location"]["lng"], point["elevation"]])
            path.append(latlon2utm(latlonz)[0])

            print("API Response: ", response)

    else: print("Height Online Update failed!")
    return path

def update_Height(coords: np.ndarray, height: float):
    for point in coords:
        point[2] += height

def compute_Parallel_Trajectory(current_pos: np.ndarray, segment: tuple, offset: float) -> tuple[tuple, np.ndarray, np.ndarray]:
    """
    Given the current position and the a pair of points (as 2D or 3D vectors), it compute the waypoints for the closest parallel inspection trajectory at horizontal offset
    distance. Height is ignored in the entire function.

    Outputs:
        - Tuple of the two waypoints as 2D vectors.
        - n_dir: Normalized vector in the direction of the output trajectory
        - n_perp: Normalized vector perpendicular to n_dir that points towards the original segment.
    """

    # Get heights out
    s1 = segment[0][:2]
    s2 = segment[1][:2]

    # If the two points are the same, then ???
    if (s1 == s2).all():
        print("ComputeTrajectories: huh?")
        #raise Exception("ComputeTrajectories: huh?")

    # Compute direction as a normalized vector
    n_dir = s2 - s1
    n_dir = n_dir / np.linalg.norm(n_dir)

    # and one of the two perpendicular vectors
    n_perp = np.array([n_dir[1], -n_dir[0]])

    # The two possible points to start the inspection
    p1 = s1 + offset * n_perp
    p2 = s1 - offset * n_perp

    # Check which one is closest and then output in accordance
    if np.linalg.norm(p1-current_pos) <= np.linalg.norm(p2-current_pos):
        p3 = s2 + offset * n_perp
        # n_perp goes from the tower to the previuos point
        return (p1, p3), n_dir, - n_perp
    else: 
        p3 = s2 - offset * n_perp
        return (p2, p3), n_dir, n_perp
    
def compute_Orbital_Trajectory(current_pos: np.ndarray, point: np.ndarray, next_point: np.ndarray, distance: float, n_points: int) -> tuple[list, np.ndarray, list]:
    """
    Given the current position and a point (as a 2D or 3D vector), it compute the waypoints for a circular inspection trajectory at a horizontal distance
    distance. Height is ignored in the entire function.

    Outputs:
        - List of the n_points waypoints as 2D vectors.
        - n_dir: Normalized vector in the direction of the output trajectory.
    """

    # Get heights out
    p1 = current_pos[:2]
    p2 = point[:2]
    p3 = next_point[:2]

    # If the two points are the same, then ???
    if (p1 == p2).all():
        print("ComputeTrajectories: huh?")
        #raise Exception("ComputeTrajectories: huh?")

    # Compute direction as a normalized vector
    n_dir = p2 - p1
    n_dir = n_dir / np.linalg.norm(n_dir)

    phi = np.arccos(-n_dir[0])
    if 0 < n_dir[1]: phi = - phi

    p4 = [np.cos((n_points - 1) / n_points * 2 * np.pi + phi), np.sin((n_points - 1) / n_points* 2 * np.pi + phi)]
    p5 = [np.cos((n_points - 1) / n_points * 2 * np.pi + phi), -np.sin((n_points - 1) / n_points* 2 * np.pi + phi)]

    d4 = np.linalg.norm(p3-p4)
    d5 = np.linalg.norm(p3-p5)

    if d4 < d5: sign = +1
    else: sign = -1

    orbit = [p2 + distance * np.array([np.cos(i / n_points * 2 * np.pi + phi), sign * np.sin(i / n_points * 2 * np.pi + phi)]) for i in range(n_points)]

    v_dirs = [p2 - 0.5*(orbit[i]+orbit[i+1])  for i in range(n_points-1)]
    v_dirs.append(p2 - 0.5*(orbit[0]+orbit[-1]))

    k = 0
    for vec in v_dirs:
        v_dirs[k]= vec / np.linalg.norm(vec)
        k += 1

    return orbit, n_dir, v_dirs

def get_Safe_Dubins_3D_Path(p1: np.ndarray, n1a: np.ndarray, p2: np.ndarray, n2a:np.ndarray,
                             min_turning_radius:float, g_max:float, **kwargs) -> np.ndarray:
    """
    Use Dubins paths to find a valid path to transition from the 3D-point p1 in the 2D-direction
    n1a to p2 in direction n2a. It checks for maximum height change and min turning radius 
    
    Outputs a NumPy array with the path and a boolean that is true if the maximum gradient is needed.
    """

    threshold = 25

    # Normalize direction in case they are not
    n1 = n1a / np.linalg.norm(n1a)
    n2 = n2a / np.linalg.norm(n2a)

    try:
        step_size = kwargs["step_size"]
    except:
        step_size = 1

    # Let's first try a regular dubins path with linear interpolation of height
    points2D, _, _, _  = DB.plan_dubins_path(p1[:2], n1, p2[:2], n2, min_turning_radius, step_size = step_size)

    # Check gradient
    d = compute_Path_Length(points2D) # The dubins function return lengths, but meh.
    
    dh_1 = p2[2]-p1[2]
    g = (dh_1) / d

    n_points = len(points2D[:,0])

    max_gradQ = np.abs(g) > g_max

    print("gmax:", g_max)
    print("g:", g)

    if not max_gradQ: # Safe

        heights = p1[2] + dh_1 * (np.array((range(n_points))) + 1) / n_points

        print("hey")
        
    else:             # Too step

        dh_2 = g_max*d * np.sign(p2[2]- p1[2])
        heights = p1[2] + dh_2 * (np.array((range(n_points))) + 1) / n_points

        if dh_2 > threshold:

            # If its is way too step, add an spiral
            h_diff = p2[2] - heights[-1]
            n_rev = int(np.ceil( h_diff / ( g_max * 2 * np.pi * min_turning_radius)))

            points3D = get_3D_Spiral_Path(np.append(points2D[-1], heights[-1]), n2, min_turning_radius, 
                                           True, n_rev, h_diff, step_size = step_size)

            points2D = np.concatenate((points2D, points3D[:,:2]))
            heights = np.concatenate((heights, points3D[:,2]))  

    return np.column_stack((points2D, heights)), max_gradQ

def compute_Path_Length(points:np.ndarray)->float:
    """
    Computes the aproximate length a discretized path given by points
    """
    length = 0
    for i in range(len(points[:,0])-1):
        length = length + np.linalg.norm(points[i]-points[i+1])

    return length

def get_Safe_Turn(p1: np.ndarray, n1: np.ndarray, p2:np.ndarray, n2:np.ndarray, R:float):

    # Check that n1 and n2 are normalized

    n1 = n1 / np.linalg.norm(n1)
    n2 = n2 / np.linalg.norm(n2)

    # Compute the angle:
    alpha = np.arccos(-np.dot(n1, n2))  # Take into account the direction of n2

    # Get the intersection S of the two lines:
    mu = np.linalg.solve(np.column_stack((n1,n2)), p2 - p1)
    s = p1 + mu[0] * n1

    # Get the distance to the initial tangent point and the center
    # of the circle
    x = R / np.tan(alpha / 2)
    d = np.sqrt(x*x + R*R)

    # Get such points
    t1 = s + x * n1
    c = s + d * (n1-n2) / np.linalg.norm(n1-n2)


    # Compute the circle vector that needs to be rotated
    phi = np.pi + alpha
    path = []

    n_points = 50
    for i in range(n_points + 1):
        path.append(rotation_2D(t1, c, -phi * i / n_points))

    path = np.array(path)

    return path

def get_Safe_Loiter_Alignment(p: np.ndarray, n: np.ndarray, p_loiter:np.ndarray,
                               loiter_radius:float, turning_radius:float, **kwargs):

    # https://observablehq.com/@donghaoren/common-tangent-lines-for-two-circles

    try:
        steps = kwargs["step_size"]
    except:
        steps = 0.5

    # Define the rotation circle center. First check which is closer
    c1 = p + turning_radius * np.array([n[1], -n[0]])
    c2 = p - turning_radius * np.array([n[1], -n[0]])

    if np.linalg.norm(c1-p_loiter) < np.linalg.norm(c2-p_loiter): 
        c = c1
        sign = +1
    else: 
        c = c2
        sign = -1

    list_all = find_Tangents_to_2Circles(c, turning_radius, p_loiter, loiter_radius)

    alphas = []
    for pair in list_all:

        alpha1 = angle_to_x(- sign * np.array([n[1], -n[0]]))
        alpha2 = angle_to_x(pair[0] - c)

        print("p", pair[0], "a1", alpha1, "a2", alpha2)

        dalpha = alpha2 - alpha1

        if -1 == sign:
            if alpha2 < alpha1:
                phi = 2 * np.pi + dalpha
            else:
                phi = dalpha
        else:
            if alpha2 < alpha1:
                phi = -dalpha
            else:
                phi = 2 * np.pi - dalpha

        alphas.append(phi)

    print(sign)
    print(alphas)

    index_min = min(range(len(alphas)), key=alphas.__getitem__)
    alpha = alphas[index_min]

    print(alpha)


    path = []

    n_points = int(np.floor(2 * np.pi * turning_radius / steps))
    for i in range(n_points + 1):
        path.append(rotation_2D(p, c, sign * alpha * i / n_points))

    path = np.array(path)
    
    #clkwiseQ = check_CLKWISE_or_CCLKWISE(p, c, t)
    return path, sign == 1, list_all

def check_CLKWISE_or_CCLKWISE(starting_point: np.ndarray, circle_center: np.ndarray, tangent_point:np.ndarray) -> bool:
    """
    (BROKEN)
    Uses the vectorial product to check whether the next circular turn needs to be done clockwise or counterclockwise
    """
    r = tangent_point - circle_center
    app_dir = tangent_point - starting_point

    # third component of the vectorial product
    z = r[0] * app_dir[1] - r[1] * app_dir[0]

    if z > 0 : return False
    else : return True

def find_Tangents_to_2Circles(p1:np.ndarray, r1: float, p2:np.ndarray, r2:float) -> list:
    """
    Finds all possible tangent point for two circles with straight lines. It outputs a list of tuples each
    of which contains the tangent points of the each line with the circles.  
    """

    # From here below, it is unused code.

    # Define all intermediate parameters:
    dplus = np.linalg.norm(p1-p2)**2 - (r1+r2)**2
    dminus = np.linalg.norm(p1-p2)**2 - (r1-r2)**2
    d1 = r1 * ( np.linalg.norm(p2)**2 - np.dot(p1, p2) )
    d2 = r2 * ( np.linalg.norm(p2)**2 - np.dot(p1, p2) )
    q = p1[0] * p2[1] - p1[1] * p2[0]

    # Different cases:

    if 0 > dplus and 0 > dminus:
        # No tangent lines
        # One circle is smaller than the other and totally inside the other
        return []
    
    if 0 > dplus and 0 == dminus:
        # 1 tangent lines
        # One circle is smaller than the other is inside the other and has a common point

        # Just one point
        return [((r2 * p1 + r1 * p2) / (r1+r2),)]
    
    if 0 > dplus and 0 < dminus:
        # 2 tangents lines
        # The two circles overlap and therefore no internal tangent lines

        # Only externals
        # Intersection

        
        if r1 == r2: # Intersection is at infinity, compute another way
            
            n = p2-p1
            n = n / np.linalg.norm(n)
            n_perp = np.array([n[1], -n[0]])

            t1p = p1 + r1 * n_perp
            t1m = p1 - r1 * n_perp
            t2p = p2 + r2 * n_perp
            t2m = p2 - r2 * n_perp 
    
        else: # intersection point is bound
            s = ( r1 * p2 - r2 * p1 ) / (r1 - r2)

            dp1s = np.linalg.norm(s-p1)
            n = (p1-s) / dp1s

            alpha = np.arcsin(r1 / dp1s)
            x1 = np.sqrt(dp1s**2-r1**2)
            x2 = np.sqrt(np.linalg.norm(s-p2)**2-r2**2)

            t1p = s + x1 * rotation_2D(n, [0,0], alpha)
            t1m = s + x1 * rotation_2D(n, [0,0], -alpha)
            t2p = s + x2 * rotation_2D(n, [0,0], alpha)
            t2m = s + x2 * rotation_2D(n, [0,0], -alpha)

        return [(t1p, t2p), (t1m, t2m)]
    
    if 0 == dplus and 0 < dminus:
        # 3 tangents lines 
        # The two circles share a common point

        # External lines
        if r1 == r2: # Intersection is at infinity, compute another way
            
            n = p2-p1
            n = n / np.linalg.norm(n)
            n_perp = np.array([n[1], -n[0]])

            t1p = p1 + r1 * n_perp
            t1m = p1 - r1 * n_perp
            t2p = p2 + r2 * n_perp
            t2m = p2 - r2 * n_perp
    
        else: # intersection point is bound
            s = ( r1 * p2 - r2 * p1 ) / (r1 - r2)

            dp1s = np.linalg.norm(s-p1)
            n = (p1-s) / dp1s

            alpha = np.arcsin(r1 / dp1s)
            x1 = np.sqrt(dp1s**2-r1**2)
            x2 = np.sqrt(np.linalg.norm(s-p2)**2-r2**2)

            t1p = s + x1 * rotation_2D(n, [0,0], alpha)
            t1m = s + x1 * rotation_2D(n, [0,0], -alpha)
            t2p = s + x2 * rotation_2D(n, [0,0], alpha)
            t2m = s + x2 * rotation_2D(n, [0,0], -alpha) 

        # Add the only internal one

        return [(t1p, t2p), (t1m, t2m), ((r2 * p1 + r1 * p2) / (r1+r2), )]

    else:
        # 4 tangents lines

        # External lines
        if r1 == r2: # Intersection is at infinity, compute another way
            
            n = p2-p1
            n = n / np.linalg.norm(n)
            n_perp = np.array([n[1], -n[0]])

            t1p = p1 + r1 * n_perp
            t1m = p1 - r1 * n_perp
            t2p = p2 + r2 * n_perp
            t2m = p2 - r2 * n_perp
    
        else: # intersection point is bound
            s = ( r1 * p2 - r2 * p1 ) / (r1 - r2)

            dp1s = np.linalg.norm(s-p1)
            n = (p1-s) / dp1s

            alpha = np.arcsin(r1 / dp1s)
            x1 = np.sqrt(dp1s**2-r1**2)
            x2 = np.sqrt(np.linalg.norm(s-p2)**2-r2**2)

            t1p = s + x1 * rotation_2D(n, [0,0], alpha)
            t1m = s + x1 * rotation_2D(n, [0,0], -alpha)
            t2p = s + x2 * rotation_2D(n, [0,0], alpha)
            t2m = s + x2 * rotation_2D(n, [0,0], -alpha) 

        # Internals

        dp1p2 = np.linalg.norm(p2-p1)
        n = (p2-p1) / dp1p2   
        d1 = r1 / (r1+r2) * dp1p2
        d2 = r2 / (r1+r2) * dp1p2

        alpha1 = np.arccos(r1 / d1)
        alpha2 = np.arccos(r2 / d2)
            
        t1p_i = p1 + r1 * rotation_2D(n, [0,0], alpha1)
        t1m_i = p1 + r1 * rotation_2D(n, [0,0], -alpha1)
        t2p_i = p2 + r2 * rotation_2D(-n, [0,0], alpha2)
        t2m_i = p2 + r2 * rotation_2D(-n, [0,0], -alpha2) 

        return [(t1p, t2p), (t1m, t2m), (t1p_i, t2p_i), (t1m_i, t2m_i)]

def get_3D_Spiral_Path(init_point: np.ndarray, n: np.ndarray, radius: float, clkwiseQ: bool,
                        n_rev: float, dheight: float, **kwargs) -> np.ndarray:
    """
    Creates an spiral trajectory based on certain parameters. The initial point is not included in the output
    """
    try:
        steps = kwargs["step_size"]
    except:
        steps = 1

    n_perp = np.array([n[1], -n[0]]) / np.linalg.norm(n)

    sign = (-1)**(1+clkwiseQ)

    points = []

    n_points = int(np.floor(2 * np.pi * radius* n_rev / steps))
    for i in range(n_points):
        point = rotation_2D(init_point[:2], init_point[:2] + sign * radius * n_perp, sign * n_rev * 2*np.pi *(i+1)/n_points)
        height = init_point[2] + dheight * (i+1) / n_points
        points.append(np.append(point, height))

    points = np.array(points)

    return points

def safe_Height_Change(p1: np.ndarray, p2: np.ndarray, g_max: float):

    # Check whether the height difference between p1 and p2 is too much
    d = np.linalg.norm(p1[:2]-p2[:2])
    g = p2[2]-p1[2] / d

    # if the difference if too much, change the altitude the maximum allowed
    # by g_max

    if np.abs(g) > g_max:
        points = [p1, np.append(p2[:2], g * d + p2[2])]
    else:
        points = [p1, p2]

    return points

def rotation_2D(point: np.ndarray, rot_point: np.ndarray, angle: float) -> np.ndarray:
    """
    Rotates the 2D "point" and "angle" through the "rot_point"
    """

    R = np.array([[np.cos(angle) , np.sin(angle)]
                 ,[-np.sin(angle), np.cos(angle)]])
    
    return copy.deepcopy(np.matmul(R, (point-rot_point))+rot_point)
    
def get_Path_Points(pos_i: np.ndarray, pos_f: np.ndarray, dx: float) -> np.ndarray:
    """
    Divides the a segment defined by two points pos_f and pos_i based onto n_points. Outputs the corresponding coordinates
    as an Nx3 or Nx2 array
    """

    delta_pos = pos_f - pos_i
    d = np.linalg.norm(delta_pos)

    # If the initial and final point are too close, just return the final point
    if d <= dx:
        return np.array([pos_f])

    # Get the number of point as the maximum number of dx length that fit within d
    n_points = int(np.floor(np.linalg.norm(delta_pos) / dx))

    # Return an array with all the points corresponding to taking n_points step of length dx along the segment
    # DOES NOT contain the initial point

    #print(np.array([pos_i + (i+1) * dx*delta_pos/d for i in range(n_points)]))
    #print(np.array([pos_f]))

    return np.concatenate((np.array([pos_i + (i+1) * dx*delta_pos/d for i in range(n_points)]), np.array([pos_f])))


# -------------------------  Plotting functions ---------------------------------------

def get_Image_Cluster(lat_deg:float, lon_deg:float, delta_lat:float, delta_long:float, zoom:int) -> tuple[Image.Image, np.ndarray, np.ndarray]:
    """
    Compute a satellital image from as a tile cluster obtained through Google Maps' API using latitude-longitude coordinates.
    
        - lat_deg and lon_deg are the aprox Latitud and Longitud for the top left corner of the top left corner tile of the cluster
          given in degrees.
        - delta_lat and delta_long are the coordinates width and height offset for the bottom right tile
          given in degrees.
        - zoom level. Each zoom level gives a 2-fold increase in the number of necessary tiles to compute for the same area. If it is too high,
          the API won't respond as consequece.

    Outputs:
        - Image cluster as a PIL.Image.Image
        - Latitude-longitud for the center of the top left tile as an array
        - Latitude-longitud for the center of the bottom right tile as an array

    For a fixed zoom, each tile of the map has some Mercator coordinates. The higher the zoom, the smaller the
    area a tile represents and the closer its top left corner will be to the aprox. coordinates give. Those tiles are 
    always 256x256px each and can be combined to obtain an entire image. For each higher zoom level, more tiles are needed
    to compile the entire image of the same area. 

    References:
        - https://en.wikipedia.org/wiki/Web_Mercator_projection
        - https://en.wikipedia.org/wiki/Mercator_projection
    
    """

    # Define the base URL for the API
    smurl = r"https://mt.google.com/vt/lyrs=s&x={0}&y={1}&z={2}"

    # Define the two exact Mercator points that enclose the area
    # In Web Mercator, y is the opossite direction as in latlon
    xmin, ymin = latlon2mercator(lat_deg, lon_deg, zoom)
    xmax, ymax = latlon2mercator(lat_deg - delta_lat, lon_deg + delta_long, zoom)
    
    # Get the image as a cluster of 256x256px tiles
    Cluster = Image.new('RGB',((xmax-xmin+1)*256-1,(ymax-ymin+1)*256-1), color = 'white') 
    for xtile in range(xmin, xmax+1):
        failedQ = False
        for ytile in range(ymin,  ymax+1):
            try:
                imgurl=smurl.format(xtile, ytile, zoom)
                #print("Opening: " + imgurl)
                imgstr = urlopen(imgurl).read()
                tile = Image.open(BytesIO(imgstr))
                Cluster.paste(tile, box=((xtile-xmin)*256 ,  (ytile-ymin)*255))
                print("Opened: " + imgurl)
            except:
                print("Failed at: " + imgurl)
                failedQ = True
                break
        
        # If any tile fails to download, stop
        if failedQ:
            break

    # Return tile cluster as an image and the latlon coord of the two bounding tiles
    return Cluster, mercator2latlon(xmin, ymin, zoom), mercator2latlon(xmax, ymax, zoom)

def plot_Satellital_Map(axes: plt.Axes, image: np.ndarray, latlon1: np.ndarray, latlon2: np.ndarray, zoom:int):
    """
    Auxiliary function that projects a satellital image to its coordinates on a already existing matplotlib canvas
    It takes into account the discrete nature of the web mercator/image tiles that the image represents
    """

    # Compute degrees of latlon per pixel of the image
    deg_p_px_X = np.abs(latlon2[1]-latlon1[1])/(image.shape[0]-255)
    deg_p_px_Y = np.abs(latlon2[0]-latlon1[0])/(image.shape[1]-255)

    # Assign latlon coordinates to top left and bottom right pixels
    # taking into account some Mercator nuissances
    latlon_e1 = np.asarray([latlon1[0], latlon1[1]])
    latlon_e2 = np.asarray([latlon2[0]-255*deg_p_px_Y, latlon2[1]+255*deg_p_px_X])

    # to UTM. Assuming everything is within the same UTM Zone
    utm1, _ = latlon2utm(latlon_e1)
    utm2, _ = latlon2utm(latlon_e2)

    # Plot image projected into the UTM coordinates
    axes.imshow(image, extent = [utm1[0], utm2[0], utm2[1], utm1[1]], alpha = 0.7)

    return None

