import numpy as np
import re
from pykml import parser
import matplotlib.pyplot as plt

import modules.coordinates as CO



class Base():

    def __init__(self, name:str = 'B0', coordinates: np.ndarray = np.array([0.0, 0.0, 0.0])):
        """
        Base class definition to represent UAV bases. A base is described by its name and EPSG3035 coordinates (E, N) (with altitud).
        """

        self.__name = name

        # As a 3D vector, UTM Coordinates + Z
        self.__coordinates = coordinates

        return None

    # ------------------------ Setting parameter functions ---------------------------

    def set_Coordinates(self, coordinates: np.ndarray):
        """
        Set the EPSG3035 coordinates as a 2D or 3D vector (E, N).
        """
        self.__coordinates = coordinates
        return None
    
    def set_Name(self, name: str):
        """
        Sets the name of the base.
        """
        self.__name = name
        return None


    # ----------------------------- Getting parameter functions -------------------------------

    def get_Coordinates(self) -> np.ndarray:
        """
        Outputs the 2D or 3D vector with the EPSG3035 of the base (E, N).
        """
        return self.__coordinates
    
    def get_Name(self) -> str:
        """
        Outputs the name of the base as str.
        """
        return self.__name  
    
    
    def print(self):
        """
        Prints the base data
        """
        print(self.__name+" with EPSG:3035 (E, N): "+str(self.get_Coordinates()))

        return None
    
    # -------------------------------- Plotting functions -------------------------------------

    def plot(self, axes: plt.Axes):
        """
        Auxiliary method that connects with an already existing matplotlib figure via axes to plot
        the tower as point with its name at its physical location
        """
        axes.plot(self.__coordinates[0], self.__coordinates[1], 'ro')
        axes.text(self.__coordinates[0], self.__coordinates[1], self.__name, fontsize = 14)

        return None

class Bases():
    
    def __init__(self, base_list:list = []):
        """
        The Bases class is a list of bases with additional methods and properties for easier coding.
        Each base needs to have a unique name.
        """

        self.__list = [] # Where base instances will be stored
        self.__coordinates = np.empty((len(self.__list), 3))

        # If the list is not empty, set coordinate list as an Nx3 numpy array
        if base_list:
            for base in self:
                self.add_Base(base)

        # Iterator attributes
        self.__iter_counter = 0
        self.__iter_max = len(self.__list)

        return None

    # ------------------------ Adding bases functions ---------------------------
    
    def add_Base(self, base: Base):
        """
        Add a single base instance to the bases list
        """

        # If there is a least one base already, check if the new shares the same name
        if self.__list:
            for base_p in self:
                if base_p.get_Name() == base.get_Name():
                    raise Exception("Bases cannot share name")

        self.__list.append(base)
        self.__coordinates = np.vstack((self.__coordinates, base.get_Coordinates()))

        return None

    def load_File(self, file: str, onlineQ: bool):
        """
        Loads a bases list using a valid ".kml" file
        If the boolean "onlineQ" is True, the height coordinates for each base will be updated online
        using a WEB API.
        """

        if file.split(".")[-1] != "kml":
            raise Exception("File is not a .kml")

        # First reset the list in case there is any data already in it
        self.reset()

        # Can parse local file, string variable or from URL
        # https://pythonhosted.org/pykml/tutorial.html#parsing-existing-kml-documents

        # Load kml file
        with open(file) as f:
            doc = parser.parse(f).getroot()
            f.close()

        # Parse kml data for each base and adds it to the list
        base_number = 1
        for point in doc.Document.Folder.Placemark:
            # Parsing string
            pathstr = np.array(re.split(',| ', point.Point.coordinates.text.strip()))

            # From string to float conversion of coords
            pointlatlon = pathstr.astype(float)

            # As the kml gives longitude-latitude, lets flip them
            pointlatlon[0], pointlatlon[1] = pointlatlon[1], pointlatlon[0]
            # Online Height Update
            if onlineQ:
                CO.update_Height_Online(pointlatlon)

            # We need to change from latlon to UTM coords
            pointn = CO.latlon2epsg3035(pointlatlon)

            # Create the base instance and adds it to the list
            base = Base(f"B{base_number}", pointn)
            self.add_Base(base)

            base_number += 1

        return None

    # ------------------------ Removing bases functions ---------------------------

    def remove_Base(self, which: str):
        """
        Remove the base given by its name "which". Not tested
        """

        # Goes one base at time checking its name and if it equal
        # to "which", deletes it.
        if self.__list:
            k = 0
            for base_p in self:
                if which == base_p.get_Name():

                    del self.__list[k]
                    del self.__coordinates[k,:]

                    return None

                k += 1
 
            raise Exception("No base with such name")
        else:
            raise Exception("There is no base to remove")


    def reset(self):
        """
        Resets the bases list to its default state
        """
        self.__list = []
        self.__coordinates = np.empty((len(self.__list), 3))

    # ------------------------ Bases extraction functions ---------------------------
        
    def get_List(self) -> list:
        """
        Return a list of all bases instances with the class
        """
        return self.__list
    
    def get_Base(self, name: str) -> Base:
        """
        Outputs a base within the bases list based on its unique name
        """
        for base in self:
            if name == base.get_Name():
                return base
            
        raise Exception("No base with such name")
    
    def print(self):
        """
        Prints the bases in a human-readable way
        """

        print("----------------------------Bases------------------------------")
        if self.__list:
            for base in self:
                print("  ", end = "")
                base.print()

        print("----------------------------------------------------------------")

    def get_Coordinates(self) -> np.ndarray:
        """
        Outputs the list of the EPSG3035 coordinates
        """

        return self.__coordinates
    

    # -------------------------------- Plotting functions -------------------------------------

    def plot(self, axes: plt.Axes):
        """
        Auxiliary method that connects with an already existing matplotlib figure via axes.
        It just plots every base within the base list
        """
        for base in self:
            base.plot(axes)

        return None

    # -------------------------------- Iterator definition -------------------------------------

    # Iterator method so we can use "for base in bases". Should be good.
    # This method just initialize things
    def __iter__(self) -> "Bases":
        self.__iter_counter = 0
        self.__iter_max = len(self.__list)
        return self
    # Should give the next element each time it is called
    def __next__(self) -> Base:
        # Sets maximum number of iterations
        if self.__iter_counter < (self.__iter_max): # self.iter_max might be changed to self.number_of_bases
                                                # len(self.list) cannot be used as it will create infinite loops
                                                # if the list length is changed within an iteration
            self.__iter_counter += 1
            return self.__list[self.__iter_counter-1]
        else:
            raise StopIteration
    

