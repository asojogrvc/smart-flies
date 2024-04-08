import numpy as np
import re
from pykml import parser
import matplotlib.pyplot as plt

import modules.coordinates as CO


class Bases():
    
    def __init__(self):
        """
        The Bases class is a list of bases with additional methods and properties for easier coding.
        Each base needs to have a unique name.
        """

        self.__bases = {}

        # Iterator attributes
        self.__iter_counter = 0
        self.__iter_max = len(self.__bases)

        return None

    # ------------------------ Adding bases functions ---------------------------
    
    def add_Base(self, name: str, position: np.ndarray):
        """
        Add a single base to the bases instance
        """

        self.__bases[name] = position

        return None


    # ------------------------ Removing bases functions ---------------------------

    def remove_Base(self, which: str):
        """
        Remove the base given by its name "which".
        """

        if which in self.__bases:
            del self.__bases[which]
            return None
        else:
            raise Exception("No base with such name")


    def reset(self):
        """
        Resets the bases list to its default state
        """
        self.__bases = {}

    # ------------------------ Bases extraction functions ---------------------------
        
    def get_Name_List(self) -> list:
        """
        Return a list of the name of all bases
        """
        return list(self.__bases.keys())
    
    def get_Base_Position(self, name: str) -> np.ndarray:
        """
        Outputs the position of "name" base
        """
        try:
             return self.__bases[name]
        except:
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
        Outputs the list of UTM coordinates without Zone
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
    

