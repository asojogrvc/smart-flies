import numpy as np

class Waypoints():
    def __init__(self):
        """
        Waypoints class to store the physical location of each waypoint and the actions the
        UAV is required to perform at such point. 
        """
        # use __x so they are private
        self.__points_List = []  # It will be a list of np.ndarray
        self.__actions_List = [] # It will be a list of dicts
        self.__modes_List = [] # It will be a list of strs

        self.__iter_counter = 0
        self.__iter_max = 0

        return None
    
    # ---------------------------------------- Add data -------------------------------------------------

    # By enforcing the use of these method, I have more control
    # over what accesible. This also prevents having more points than
    # actions
    def add_Waypoint(self, point: np.ndarray, actions: dict, mode:str):
        """
        Add new waypoint to the list. It takes a 3D vector (usually latlon+Z) and the actions
        that a uav will make at that point as a dictionary.
        """

        # I may need to add some sanity checks here too

        self.__points_List.append(point)
        self.__actions_List.append(actions)
        self.__modes_List.append(mode)

        return None

    def add_Waypoint_List(self, points: np.ndarray, actions_list: list, modes_list:list):
        """
        Add a list of physical points (usually latlon+Z) as a Nx3 matrix and the list of N actions to peform at each of them.
        """

        # I may need to add some sanity checks here too

        k = 0 # just in case any k filters from anywhere
        for k in range(points.shape[0]):
            self.add_Waypoint(points[k, :], actions_list[k], modes_list[k])

        return None
    
    # ---------------------------------------- Get data out -------------------------------------------------

    def get_Waypoint(self, which: int) -> tuple[list, list, list]:
        """
        Outputs the waypoint given by position "which"
        """
        return self.__points_List[which], self.__actions_List[which], self.__modes_List[which]

    def get_Points_List(self) -> list:
        """
        Returns a list with the UTM Coordinates of the each waypoint.
        """
        return self.__points_List
    
    def get_Actions_List(self) -> list:
        """
        Returns a list with the UAV actions of the each waypoint.
        """
        return self.__actions_List
    
    def get_Modes_List(self) -> list:
        """
        Returns a list with the UAV mode of the each waypoint, i.e., Inspection or Navigation
        """
        return self.__modes_List
    
    # ---------------------------------------- Reset data -------------------------------------------------
    
    def reset(self):
        """
        Reset the entire list to empty
        """
        self.__points_List = []
        self.__actions_List = []
        self.__modes_List = []

        return None

    # ---------------------------------------- Iterator -------------------------------------------------

    # Iterator method so we can use "for wp in waypoints". Should be good.
    # This method just initialize things
    def __iter__(self):
        self.__iter_counter = 0
        self.__iter_max = len(self.__points_List)
        return self
    # Should give the next element each time it is called
    def __next__(self):
        # Sets maximum number of iterations
        if self.__iter_counter < (self.__iter_max):
            self.__iter_counter += 1
            return self.get_Waypoint(self.__iter_counter-1)
        else:
            raise StopIteration
        
    # ---------------------------------------- Print data -------------------------------------------------

    def print(self):
        """
        Prints the UTM coordinates and actions of each waypoint.
        """

        k = 0
        for wp in self:

            point_str = np.array2string(wp[0], separator=", ")
            print(f"  WP{k}"+" - {pos: "+point_str+", action: "+str(wp[1])+"}; mode: "+str(wp[2]))
            k += 1

        return None
            

