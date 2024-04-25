# UAV and UAV Team Classes

# Load Model Database
import json 

f = open('./files/models.json')
models = json.load(f)
f.close() 

class UAV():
    def __init__(self, **kwargs):
        """
        A representation of a UAV. It contains all the necessary parameters 
        """

        if "id" in kwargs:
            self.__id = str(kwargs["id"])
        else:
            self.__id = "UAV_0"

        if "model" in kwargs:
            if kwargs["model"] in models:
                self.__model = kwargs["model"]
            else:
                raise Exception("No such model: "+kwargs["model"])
        else:
            self.__model = "Model A"

        if "base" in kwargs:
            self.__base = kwargs["base"]
        else:
            self.__base = "B"

        #self.__route = []

    # ------------------------ Setting parameter functions ---------------------------

    def set_ID(self, id:str):
        self.__id = str(id)
        return None
        
    def set_Model(self, model:str):

        if model in models:
            self.__model = model
        else:
            raise Exception("No such model")
        return None
    
    def set_Base(self, base:str):
        self.__base = base

    # ----------------------------- Getting parameter functions -------------------------------
    
    def get_ID(self) -> str:
        return self.__id
    
    def get_Model(self) -> str:
        return self.__model
    
    def get_Base(self) -> str:
        return self.__base
        
    
class UAV_Team():
    # It always initializes as an empty list to avoid adding non UAV instances to the list
    def __init__(self):
        """
        This class is just a UAV list with additional methods and iterators.
        """

        self.__list = {}   # id: UAV Instance

        # Iterator
        self.__iter_counter = 0
        self.__iter_max = 0

        return None

    # ------------------------ Parameter output functions ----------------------
        
    def get_List(self) -> dict:
        """
        Gets the UAVs as a Python dict.
        """
        return self.__list
    
    def get_Speeds(self) -> dict:

        speeds = {}

        for uav in self:
            speeds[uav.get_ID()] = models[uav.get_Model()]["Speed"]

        return speeds
    
    def get_Inspection_Speeds(self) -> dict:

        speeds = {}

        for uav in self:
            speeds[uav.get_ID()] = models[uav.get_Model()]["iSpeed"]

        return speeds
    
    def get_Types(self) -> dict:

        types = {}

        for uav in self:
            types[uav.get_ID()] = models[uav.get_Model()]["type"]

        return types

    
    def print(self):
        """
        Prints the entire team data
        """

        print("----------------------------UAV Team------------------------------")
        for uav in self:
            print(" - ID: ", uav.get_ID(), " Model: ", uav.get_Model(), " Base: ", uav.get_Base())
        print("------------------------------------------------------------------")

        return None
    
    # ------------------------ Adding UAVs functions ---------------------------

    def add_UAV(self, uav: UAV):
        """
        Add a UAV to the list. Its ID must be unique.
        """

        if not uav.get_ID() in self.__list:
            self.__list[uav.get_ID()] = uav
        else:
            raise Exception("New UAV cannot share its ID with any other UAV in the team")

        return None

    # ------------------------ Removing UAVs functions ---------------------------
        
    def empty(self):
        """
        Deletes all current UAVs of the team
        """
        self.__list = []

        return None

    def remove_UAV(self, id:str):
        """
        Remove the "which" UAV. It position is given by the order by which it was added.
        """

        if id in self.__list:
            del self.__list[id]
        else:
            raise Exception("No such ID")
        
        return None

    # ------------------------ UAV extraction functions ---------------------------

    def select_UAV(self, id:str) -> UAV:
        """
        Selects an UAV based on its unique ID
        """
        if id in self.__list:
            return self.__list[id]
        else:
            raise Exception("No such ID")

        return None


    # -------------------------------- Iterator definition -------------------------------------

    # Iterator method so we can use "for uav in uavs".
    # This method just initialize things
    def __iter__(self) -> "UAV_Team":
        self.__iter_counter = 0
        self.__iter_max = len(self.__list)
        return self
    # Should give the next element each time it is called
    def __next__(self) -> UAV:
        # Sets maximum number of iterations
        if self.__iter_counter < (self.__iter_max):
            self.__iter_counter += 1

            id = list(self.__list.keys())[self.__iter_counter-1]
            return self.__list[id]
        else:
            raise StopIteration



        
            
        