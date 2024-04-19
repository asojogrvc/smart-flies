import numpy as np, matplotlib.pyplot as plt

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
    
    # ------------------------ Parameter output functions ----------------------
        
    def get_List(self) -> dict:
        """
        Gets the Bases as a Python dict.
        """
        return self.__bases
    
    def print(self):
        """
        Prints the bases data
        """

        print("------------------------------Bases-------------------------------")
        for name, position in self:
            print(" - Name: ", name, " Position: ", position)
        print("------------------------------------------------------------------")

        return None

    # ------------------------ Adding bases functions ---------------------------
    
    def add_Base(self, name: str, position: np.ndarray):
        """
        Add a single base to the bases instance
        """

        if not name in self.__bases:
            self.__bases[name] = position
        else:
            raise Exception("New base cannot share nane with any other base in the team")

        return None


    # ------------------------ Removing bases functions ---------------------------

    def remove_Base(self, which: str):
        """
        Remove the base given by its name "which".
        """

        if which in self.__bases:
            del self.__bases[which]
        else:
            raise Exception("No such ID")
        
        return None


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
    

    # -------------------------------- Plotting functions -------------------------------------

    def plot(self, axes: plt.Axes):

        for name, position in self:
            axes.scatter(position[0], position[1], c ='r', s = 300, zorder=10)
            axes.annotate(name, (position[0], position[1]), fontsize=12, zorder=15, ha='center', va='center')

        return None

    # -------------------------------- Iterator definition -------------------------------------

    
    # Iterator method so we can use "for base in bases". Should be good.
    # This method just initialize things
    def __iter__(self) -> "Bases":
        self.__iter_counter = 0
        self.__iter_max = len(self.__bases)
        return self
    # Should give the next element each time it is called

    def __next__(self) -> tuple:
        # Sets maximum number of iterations
        if self.__iter_counter < (self.__iter_max):
            self.__iter_counter += 1

            name = list(self.__bases.keys())[self.__iter_counter-1]
            return name, self.__bases[name]
        else:
            raise StopIteration
