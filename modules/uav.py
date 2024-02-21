# UAV and UAV Team Classes
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from yaml import load, Loader
import copy

from modules import waypoints as WP, coordinates as CO, towers as TW, bases as BA

class UAV_Battery():
    def __init__(self, Type: str = " ", Cap: float = 0.0, Cells: int = 0, VpC: float = 0.0):
        self.type = Type       
        self.capacity = Cap 
        self.cells = Cells
        self.volts_per_cell = VpC

    def reset(self):
        self.type = " "       
        self.capacity = 0.0        # [mAh]
        self.cells = 0              
        self.volts_per_cell = 0.0  # [V]

    def capacity_Joules(self):
        return self.capacity * 3.6 * self.volts_per_cell * self.cells

class UAV():
    def __init__(self, Name: str = "M300", 
                 ID: str = "U", 
                 Mass: float = 7.2, 
                 rotors_number: int = 4, 
                 rotor_blades:int = 2, 
                 rotor_radius: float = 0.2667,
                 blade_chord: float = 0.0247, 
                 lift_coefficient: float = 1.01, 
                 drag_coefficient: float = 0.01,
                 Kappa: float = 1.15, 
                 Eta: float = 0.72, 
                 Kmu: float = 4.65, 
                 flat_area: float = 0.35,
                 Camera: bool = False,
                 Battery: UAV_Battery = UAV_Battery()):
        """
        A representation of a UAV. It contains all the necessary parameters 
        """
        
        # Verifying values might be needed. Add verification if issues are common
        self.__name = Name
        self.__id = ID
        self.__mass = Mass                          # total mass                  [kg]
        self.__number_of_Rotors = rotors_number     # number of rotors            [-]
        self.__rotor_Blades = rotor_blades          # number of rotor blades      [-]
        self.__rotor_Radius = rotor_radius          # rotor radius                [m]
        self.__blade_Chord = blade_chord            # blade chord (x=0.7*R)       [m]

        self.__lift_Coefficient = lift_coefficient  # sectional lift coefficient  [-]
        self.__drag_Coefficient = drag_coefficient  # sectional drag coefficient  [-]  
        self.__kappa = Kappa                        # induced power factor        [-]
        self.__eta = Eta                            # energy efficiency           [-]
        self.__K_mu = Kmu                           # P0 numerical constant       [-]
        self.__flat_Area = flat_area                # equivalent flat-plate area  [m^2]
        self.__camera = Camera                      # If it needs camera actions  [bool]

        self.__battery = Battery

        self.route = []
        self.routeAbstract = []
        self.routeUTM = []
        self.routeModes = []
        self.waypoints = WP.Waypoints()
        self.missionSettings = {'Base': 'B1', 'Nav. speed': 15, 'Insp. speed': 7, 'Landing Mode': 'Auto',
                                 "Insp. height": 5, "Insp. horizontal offset": 0, "Cam. angle": 90, "Tower distance": 5}
        self.extra_parameters = {}
        self.sim_trajectory = None
        self.sim_angles = None
        self.sim_plot = None
        self.arrow_refs = [None]
        self.sim_continueQ = False

    # ------------------------ Setting parameter functions ---------------------------
        
    def load_from_Model(self, model: str, id:str):

        # It might be a good not load the entire database each time
        f = open("./files/devices.yaml", "r")
        data = load(f, Loader=Loader)[model]
        f.close()

        
        self.__name             = model
        self.__id               = id
        self.__mass             = data['mass']
        self.__number_of_Rotors = data['number_of_rotors']
        self.__rotor_Blades     = data['rotor_blades']
        self.__rotor_Radius     = data['rotor_radius']
        self.__blade_Chord      = data['blade_chord']
        self.__lift_Coefficient = data['lift_coefficient']
        self.__drag_Coefficient = data['draft_coefficient']
        self.__kappa            = data['induced_power_factor']
        self.__eta              = data['energy_efficiency']
        self.__K_mu             = data['P0_numerical_constant']
        self.__flat_Area        = data['equivalent_flat_plate_area']
        self.__camera           = True
        self.__battery          = UAV_Battery("Cellular",
                                                data['battery']['capacity'],
                                                data['battery']['number_of_cells'],
                                                data['battery']['voltage_per_cell'],
                                                )

        return None
        

    # ----------------------------- Getting parameter functions -------------------------------
        
    def get_ID(self) -> str:
        """
        Outputs the ID of the UAV as a str.
        """
        return self.__id
    
    def get_Name(self) -> str:
        return self.__name
    
    def get_Battery(self) -> UAV_Battery:
        return self.__battery
    
    def get_UAV_Parameters(self) -> tuple:
        """
        Outputs the ID of the UAV as a str.
        """

        out = { "name": copy.deepcopy(self.__name),            
                "id": copy.deepcopy(self.__id),                
                "mass": copy.deepcopy(self.__mass),          
                "number_of_Rotors": copy.deepcopy(self.__number_of_Rotors),
                "rotor_Blades": copy.deepcopy(self.__rotor_Blades),       
                "rotor_Radius": copy.deepcopy(self.__rotor_Radius),     
                "blade_Chord": copy.deepcopy(self.__blade_Chord),       
                "lift_Coefficient": copy.deepcopy(self.__lift_Coefficient),   
                "drag_Coefficient": copy.deepcopy(self.__drag_Coefficient), 
                "kappa": copy.deepcopy(self.__kappa),              
                "eta": copy.deepcopy(self.__eta),              
                "K_mu": copy.deepcopy(self.__K_mu),              
                "flat_Area": copy.deepcopy(self.__flat_Area),          
                "camera": copy.deepcopy(self.__camera),             
                "battery": copy.deepcopy(self.__battery) }         

        return out
        
    # Add units
    def print(self):
        """
        Prints the data of the UAV in a human-readable way.
        """

        print("UAV: ", self.__name, " with ID: ", self.__id)
        print("|--> Mass: ", self.__mass)
        print("|--> Number of rotors: ", self.__number_of_Rotors)
        print("|--> Blades per rotor: ", self.__rotor_Blades)
        print("|--> Rotor radius: ", self.__rotor_Radius)
        print("|--> Blade chord: ", self.__blade_Chord)
        print("|--> Lift Coefficient: ", self.__lift_Coefficient)
        print("|--> Drag Coefficient: ", self.__drag_Coefficient) 
        print("|--> kappa: ", self.__kappa)
        print("|--> eta: ", self.__eta)
        print("|--> K_mu: ", self.__K_mu)
        print("|--> Effective flat area: ", self.__flat_Area)
        print("|--> CameraQ: ", self.__camera)
        print("\--> Battery:")
        print("        |--> Type: ", self.__battery.type)
        print("        |--> Capacity: ", self.__battery.capacity)
        print("        |--> Number of cells: ", self.__battery.cells)
        print("        \--> Volts per cell: ", self.__battery.volts_per_cell)

    # ----------------------------- Plotting functions -------------------------------

    def plot_Route(self, axes: plt.Axes, colorp):
        """
        Plots the current routes for this UAV into existing Matplotlib axes.
        """


        if self.arrow_refs[0] != None:
            for k in range(len(self.arrow_refs)):
                self.arrow_refs[k].remove()

        self.arrow_refs = len(self.route)*[None]

        k = 0
        for edge in self.routeUTM: # edge is an ordered tuple

            UTM0 = edge[0]
            UTM1 = edge[1]

            self.arrow_refs[k] = axes.arrow(UTM0[0], UTM0[1], UTM1[0]-UTM0[0], UTM1[1]-UTM0[1], color = colorp,
                       head_width=12, head_length=12, length_includes_head = True, zorder = 15)
            
            k += 1

    # ----------------------------- General functions -------------------------------

    def compute_Waypoints(self, mode: int, towers: TW.Towers, bases: BA.Bases):
        """
        Computes the corresponding waypoints and actions for the UAV depending on the use case (mode) and
        mission settings

        Modes:
            - "0": Photo and video of segments
            - "1": Photo and video of points
        """

        # Reset waypoints first
        self.waypoints.reset()

        dH = 20 # Security height offset for navigation
        fH = 5 # first height First and last height.

        # Tower height, this should be a external parameter
        if "Tower Height" in self.extra_parameters:
            tH = self.extra_parameters["Tower Height"]
        else:
            tH = 5 

        utmZone = towers.get_UTM_Zone()

        # Precompute the coordinates into a dict. THIS COULD SIMNPLIFY STUFF IF I SAVED DIRECTLY INTO THE UI
        coords_dict = towers.get_DictCoordinates()
        coords_dict[self.missionSettings["Base"]] = bases.get_Base(self.missionSettings["Base"]).get_Coordinates()
        bH = coords_dict[self.missionSettings["Base"]][2]

        match mode:

            case 0:
                
                if len(self.route) == 1:
                    return None

                # Precompute points parallel to the power lines without heights
                preMoves = []
                preNdirs = []
                preVdirs = []
                point = self.routeUTM[0][0][:2] # Base
                for move in self.routeUTM[1:-1]: 

                    pmove, n_dir, v_dir = CO.compute_Parallel_Trajectory(point, move, self.missionSettings["Insp. horizontal offset"])
                    preMoves.append(pmove)
                    preNdirs.append(n_dir)
                    preVdirs.append(v_dir)

                    point = pmove[1]

                #print(" moves : ", preMoves)
                #print(" Ndirs : ", preNdirs) # preNdirs seems to be fine
                print(" Vdirs : ", preVdirs) # preNdirs seems to be fine
                    

                # Gimbal is fixed beforehand
                gimbal = - float(self.missionSettings["Cam. angle"])
                
                # First waypoint is at base but at "fH" height. Start video and point
                # the camera to the next point. Always at Z = fH

                point = np.append(self.routeUTM[0][0][:2], fH)

                n_dir = preMoves[0][0]-self.routeUTM[0][0][:2]
                n_dir = n_dir / np.linalg.norm(n_dir)

                yaw = np.rad2deg(np.arccos(n_dir[1]))
                if n_dir[0] < 0: yaw = -yaw

                actions = {"video_start": 0, "gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")

                # The next point is at the same place but at Z = fH + dH
                point = np.append(self.routeUTM[0][0][:2], tH + dH)
                actions = {"gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")

                # -----
                ipoints = CO.get_Path_Points(self.routeUTM[0][0], preMoves[0][0], 200)
                CO.update_UTM_Height_Online(ipoints, utmZone)
                CO.update_Height(ipoints, tH + dH - bH)
                    
                if len(ipoints) > 1:
                    for ipoint in ipoints[:-1]:
                        self.waypoints.add_Waypoint(ipoint, actions, "Navigation")
                # -----

                btH = coords_dict[self.route[0][1]][2] - bH # base of tower with respect to the uav base

                point = np.append(preMoves[0][0], btH + tH + dH)
                actions = {"gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Inspection")

                # It is the same loop as before. Maybe join them

                k = 0
                for pmove in preMoves:

                    print("k:", k)

                    btH1 = coords_dict[self.route[1:-1][k][0]][2] - bH # base of tower 1 with respect to the uav base
                    btH2 = coords_dict[self.route[1:-1][k][1]][2] - bH # base of tower 2 with respect to the uav base

                    # If the current mode is navigation, go to safety height
                    if "Navigation" == self.routeModes[1:-1][k]:
                        
                        point1 = np.append(pmove[0], btH1 + tH + dH)
                        point2 = np.append(pmove[1], btH2 + tH + dH)

                        # Point towards movement
                        yaw = np.rad2deg(np.arccos(preNdirs[k][1]))
                        if preNdirs[k][0] < 0: yaw = -yaw
                        actions1 = {"gimbal": gimbal, "yaw": yaw}

                        # Point towards movement, which is a point in the next pmove
                        
                        n_dir = preMoves[k][0]-pmove[1]
                        n_dir = n_dir / np.linalg.norm(n_dir)
                        yaw = np.rad2deg(np.arccos(n_dir[1]))
                        if n_dir[0] < 0: yaw = -yaw

                        mode1 = "Navigation"
                        #actions2 = {"gimbal": gimbal, "yaw": yaw}

                        # Add here a new waypoint if inspection is next?

                    # If it is inspection, go to inspection height + v. offset
                    else:

                        point1 = np.append(pmove[0], btH1 + tH + self.missionSettings["Insp. height"])
                        point2 = np.append(pmove[1], btH2 + tH + self.missionSettings["Insp. height"])

                        # If gimbal is perpendicular to the floor, then yaw is on the direction of movement

                        if 90 == self.missionSettings["Cam. angle"]:

                            yaw = np.rad2deg(np.arccos(preNdirs[k][1]))
                            if preNdirs[k][0] < 0: yaw = -yaw

                        else: # If not point towards the tower/power line
                            
                            n_dir = preVdirs[k]
                            yaw = np.rad2deg(np.arccos(n_dir[1]))
                            if n_dir[0] < 0: yaw = -yaw


                        actions1 = {"gimbal": gimbal, "yaw": yaw, "photo": True}
                        mode1 = "Inspection"


                    self.waypoints.add_Waypoint(point1, actions1, mode1)
                    # -----
                    if "Navigation" == self.routeModes[1:-1][k]:
                        ipoints = CO.get_Path_Points(point1, point2, 200)
                        CO.update_UTM_Height_Online(ipoints, utmZone)
                        CO.update_Height(ipoints, tH + dH - bH)
                    
                        if len(ipoints) > 1:
                            for ipoint in ipoints[:-1]:
                                self.waypoints.add_Waypoint(ipoint, actions, "Navigation")
                    # -----
                    self.waypoints.add_Waypoint(point2, actions1, mode1)

                    k += 1

                # The penultimun move should be inspection as it will be redudant otherwise
                # At the last point get to safety height
                    
                n_dir = self.routeUTM[0][0][:2] - preMoves[-1][1]
                n_dir = n_dir / np.linalg.norm(n_dir)

                yaw = np.rad2deg(np.arccos(n_dir[1]))
                if n_dir[0] < 0: yaw = -yaw
                actions = {"gimbal": gimbal, "yaw": yaw}
                
                point2 = np.append(pmove[1], btH2 + tH + dH)
                self.waypoints.add_Waypoint(point2, actions, "Navigation")

                # -----
                ipoints = CO.get_Path_Points(point2, self.routeUTM[-1][1], 200)
                CO.update_UTM_Height_Online(ipoints, utmZone)
                CO.update_Height(ipoints, tH + dH - bH)
                    
                if len(ipoints) > 1:
                    for ipoint in ipoints[:-1]:
                        self.waypoints.add_Waypoint(ipoint, actions, "Navigation")
                # -----

                # Get back to base
                point = np.append(self.routeUTM[-1][1][:2], tH + dH)
                actions = {"gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")

                point = np.append(self.routeUTM[-1][1][:2], fH)
                actions = {"video_stop": 0, "gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")

                print(self.__name+":")
                self.waypoints.print()

            case 1:

                if len(self.route) == 1:
                    return None
                
                # Gimbal is fixed beforehand
                gimbal = - float(self.missionSettings["Cam. angle"])

                # First waypoint is at base but at "fH" height. Start video and point
                # the camera to the next point. Always at Z = fH

                point = np.append(self.routeUTM[0][0][:2], fH)

                n_dir = self.routeUTM[0][1][:2]-self.routeUTM[0][0][:2]
                n_dir = n_dir / np.linalg.norm(n_dir)

                yaw = np.rad2deg(np.arccos(n_dir[1]))
                if n_dir[0] < 0: yaw = -yaw

                actions = {"video_start": 0, "gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")

                # The next point is at the same place but at Z = fH + dH
                point = np.append(self.routeUTM[0][0][:2], tH + dH)
                actions = {"gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")


                m = 0
                for move in self.routeUTM[0:-1]: 

                    # Tower height, this should be a external parameter
                    if "Orbital Points" in self.extra_parameters:
                        n_points = self.extra_parameters["Orbital Points"]
                    else:
                        n_points = 5 

                    orbit, n_dir, v_dirs = CO.compute_Orbital_Trajectory(move[0], move[1], coords_dict[self.route[0:-1][m][0]],
                                                                          self.missionSettings["Insp. horizontal offset"], n_points)

                    # Above the first orbital point

                    
                    ipoints = CO.get_Path_Points(move[0], np.append(orbit[0], 0), 200)
                    CO.update_UTM_Height_Online(ipoints, utmZone)
                    CO.update_Height(ipoints, tH + dH - bH)
                    
                    if len(ipoints) > 1:
                        for ipoint in ipoints[:-1]:
                            self.waypoints.add_Waypoint(ipoint, actions, "Navigation")

                    btH = coords_dict[self.route[0:-1][m][1]][2] - bH   # base of tower with respect to the uav base

                    point = np.append(orbit[0], tH + dH + btH)
                    yaw = np.rad2deg(np.arccos(n_dir[1]))
                    if n_dir[0] < 0: yaw = -yaw
                    actions = {"gimbal": gimbal, "yaw": yaw}
                    self.waypoints.add_Waypoint(point, actions, "Navigation")

                    
                    # Get down, orbit and inspect
                    k = 0

                    for porbit in orbit:
                        
                        point = np.append(porbit, btH + tH + self.missionSettings["Insp. height"])

                        n_dir = v_dirs[k]

                        yaw = np.rad2deg(np.arccos(n_dir[1]))
                        if n_dir[0] < 0: yaw = -yaw
                        actions = {"gimbal": gimbal, "yaw": yaw, "photo": True}

                        self.waypoints.add_Waypoint(point, actions, "Inspection")
                        k += 1

                    # Above the last orbital point
                    point = np.append(orbit[-1], tH + dH + btH)

                    n_dir = self.routeUTM[m+1][1][:2]-orbit[-1]
                    n_dir = n_dir / np.linalg.norm(n_dir)

                    yaw = np.rad2deg(np.arccos(n_dir[1]))
                    if n_dir[0] < 0: yaw = -yaw
                    actions = {"gimbal": gimbal, "yaw": yaw}
                    self.waypoints.add_Waypoint(point, actions, "Navigation")

                    m += 1


                n_dir = self.routeUTM[-1][1][:2]-self.routeUTM[-1][0][:2]
                n_dir = n_dir / np.linalg.norm(n_dir)

                point = np.append(self.routeUTM[0][0][:2], tH + dH)

                yaw = np.rad2deg(np.arccos(n_dir[1]))
                if n_dir[0] < 0: yaw = -yaw

                actions = {"gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")

                point = np.append(self.routeUTM[0][0][:2], fH)

                actions = {"video_start": 0, "gimbal": gimbal, "yaw": yaw}
                self.waypoints.add_Waypoint(point, actions, "Navigation")
                

            case _ :
                print("No such mode exits")
                return None

        return None
            
class UAV_Team():
    # It always initializes as an empty list to avoid adding non UAV instances to the list
    def __init__(self, _list:list = []):
        """
        This class is just a UAV list with additional methods and iterators.
        """

        self.__list = _list

        # Iterator
        self.__iter_counter = 0
        self.__iter_max = 0

        return None

    # ------------------------ Parameter output functions ----------------------
        
    def get_List(self) -> list:
        """
        Gets the UAVs as a Python list.
        """
        return self.__list
    
    def print(self):
        """
        Prints the entire team data
        """

        print("----------------------------UAV Team------------------------------")
        for uav in self:
            print("  ", end = "")
            uav.print()
        print("------------------------------------------------------------------")

    def print_Mission_Settings(self):
        
        for uav in self:
            print("id: " + uav.get_ID() + "->", end = "")
            print(uav.missionSettings)
    
    # ------------------------ Adding UAVs functions ---------------------------

    def add_UAV(self, uav: UAV):
        """
        Add a UAV to the list. Its ID must be unique.
        """

        # This could be slow. I could save all currents used IDs in a list and check it instead
        for uavp in self:
            if uav.get_ID() == uavp.get_ID(): raise Exception("New UAV cannot share its ID with any other UAV in the team")

        self.__list.append(uav)

        return None

    def load_File(self, file:str):
        """
        Load a UAV Team from a valid xml file. "file" is the file path.
        """

        self.empty()
        
        tree = ET.parse(file)
        root = tree.getroot()

        for uav in root.findall('uav'):
        
            self.add_UAV(
                UAV(
                    uav.find('name').text, 
                    uav.attrib["id"],
                    float(uav.find('mass').text),
                    int(uav.find('number_of_Rotors').text), 
                    int(uav.find('rotor_Blades').text), 
                    float(uav.find('rotor_Radius').text), 
                    float(uav.find('blade_Chord').text),
                    float(uav.find('lift_Coefficient').text), 
                    float(uav.find('drag_Coefficient').text), 
                    float(uav.find('kappa').text),
                    float(uav.find('eta').text),
                    float(uav.find('K_mu').text),
                    float(uav.find('flat_area').text), 
                    eval(uav.find('camera').text),
                    UAV_Battery(        # Maybe faster using lambda to avoid calling battery each time
                        uav.find('battery').find('type').text,
                        float(uav.find('battery').find('capacity').text),
                        int(uav.find('battery').find('cells').text),
                        float(uav.find('battery').find('volts_per_cell').text)
                    )
                )
            )
        
        return None


    # ------------------------ Removing UAVs functions ---------------------------
        
    def empty(self):
        """
        Deletes all current UAVs within the team
        """
        self.__list = []
        return None

    def remove_UAV(self, id:str):
        """
        Remove the "which" UAV. It position is given by the order by which it was added.
        NOT TESTED
        """
        uav = self.select_UAV(id)
        del uav
        return None

    # ------------------------ UAV extraction functions ---------------------------

    def select_UAV(self, id:str) -> UAV:
        """
        Selects an UAV based on its unique ID
        """
        for uav in self:
            if id == uav.get_ID(): return uav

        raise Exception('No UAV with such ID')
    
    def select_UAV_by_Order(self, which: int) -> UAV:
        """
        Selects an UAV based on its position within the list. It is determined by the order of addition to the list.
        """
        return self.__list[which]
    
    def compute_Team_Waypoints(self, mode: int, towers: TW.Towers, bases: BA.Bases):
        """
        Computes the corresponding waypoints and actions of the entire team depending on the use case (mode) and
        mission settings

        Modes:
            - "0": Photo and video of segments
            - "1": Photo and video of points

        """

        for uav in self:
            uav.compute_Waypoints(mode, towers, bases)

        return None
    
    def get_Waypoints(self):
        """
        Output the waypoints YAML as an object using the PyYAML package
        """

        return None



    # -------------------------------- Plotting functions -------------------------------------
    
    def plot_Routes(self, axes: plt.Axes):
        """
        Plots the current routes for all UAV
        """

        colorlist = ['red', 'blue', 'green', 'cyan', 'brown'] # After the finish with the existing colors, it loops back.

        k = 0
        for uav in self:
            uav.plot_Route(axes, colorlist[k])
            k += 1

        return None

    # -------------------------------- Iterator definition -------------------------------------

    # Iterator method so we can use "for uav in uavs". Should be good.
    # This method just initialize things
    def __iter__(self) -> "UAV_Team":
        self.__iter_counter = 0
        self.__iter_max = len(self.__list)
        return self
    # Should give the next element each time it is called
    def __next__(self) -> UAV:
        # Sets maximum number of iterations
        if self.__iter_counter < (self.__iter_max): # self.iter_max might be changed to self.number_of_bases
                                                # len(self.list) cannot be used as it will create infinite loops
                                                # if the list length is changed within an iteration
            self.__iter_counter += 1
            return self.__list[self.__iter_counter-1]
        else:
            raise StopIteration



        
            
        