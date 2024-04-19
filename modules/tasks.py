# Tasks and Towers class

import numpy as np, networkx as nx, matplotlib.pyplot as plt


class Towers():
    """
    A graph representation of the towers.
    """

    def __init__(self):
        self.__graph = nx.empty_graph(0)

    # ---------------------------------- Parameter settings -------------------------------------
    
    def add_Tower(self, name: str, position: np.ndarray, **kwargs):
        """
        Add tower (or node) to the towers graph based on its coordinates and a list
        of connections
        """

        self.__graph.add_node(name, position = position)
        if "connections" in kwargs:
            connections = kwargs["connections"]
            self.__graph.add_edges_from(connections)
        
        return None
    
    def add_Power_Lines(self, list_of_pairs: list):
        """
        Add tower to tower connections given a list of tuples.
        """

        try:
            if tuple == type(list_of_pairs):
                self.__graph.add_edges_from([list_of_pairs])
            else:
                for pair in list_of_pairs:
                    self.__graph.add_edges_from(list_of_pairs)
        except:
            raise Exception("Some connections are not valid")
        
        return None
    
    
    def reset(self):
        """
        Reset the towers list to its empty default status
        """
        self.__graph = nx.empty_graph(0)

        return None

    # ---------------------------------- Parameter Output -------------------------------------
    
    def get_Positions(self) -> dict:
        """
        Gets a dict that contains all the position of the present towers
        """

        return nx.get_node_attributes(self.__graph,'position')
    
    def get_Graph(self) -> nx.MultiDiGraph:
        """
        Gets the Networkx Graph representation of the towers
        """
        return self.__graph

    # ---------------------------------- Plotting ------------------------------------------------

    def plot(self, axes: plt.Axes):
        """
        Plots the towers using their UTM Coordinates into an existing Matplotlib axes
        """
        pos3D = self.get_Positions()

        # Project it into 2D
        pos2D = {}
        for key in pos3D:
            pos2D[key] = pos3D[key][0:2]

        nx.draw_networkx(self.__graph, pos = pos2D, ax=axes, with_labels = True),
                          #verticalalignment="bottom", horizontalalignment = "left")
        
        return None
        
class Tasks():
    """
    Tasks are either point inspection or lineal inspection. This is a list of then.
    """

    def __init__(self):

        self.__list = {} # name, data. 
        
        # data is a dict with:
            # inspection_of: either a str with the tower name or a tuple for two names
            # incompatiblity: a list of incompatible UAV IDs

    def add_Task(self, name: str, inspection_of: str | tuple, **kwargs):
        """
        For punctual inspection: inspection_of is a str with the name of the tower
        For lineal inspection: it is a tuple with the name of the two defining towers.

        It is also possible to define incompatibility with certain UAVs by giving a list with their IDs
        """

        if "incompatible_IDs" in kwargs:
            incompatible_IDs = kwargs["incompatible_IDs"]
            incompatible_IDs = [str(id) for id in incompatible_IDs]
        else:
            incompatible_IDs = []


        self.__list[name] = {"inspection_of": inspection_of, "incompatible_IDs": incompatible_IDs}

        return None
    
    def get_Task_Parsing_Dict(self) -> dict:

        parsing = {}

        for name, data in self:
            if str == type(data["inspection_of"]):
                parsing[name] = [data["inspection_of"]]

            elif tuple == type(data["inspection_of"]) and 2 == len(data["inspection_of"]):
                parsing[name+"_U"] = list(data["inspection_of"])
                parsing[name+"_D"] = list(data["inspection_of"][::-1])

        return parsing
        
    def print(self):

        print("----------------------------Task List-----------------------------")
        for name, data in self:
            print(" - Name: ", name, " Inspection of: ", data["inspection_of"], " Incompatible with IDs: ", data["incompatible_IDs"])
        print("------------------------------------------------------------------")


        return None
    
    def compatible_With(self, id: str) -> list:

        compatible = []
        for name, data in self:
            if not id in data["incompatible_IDs"]:

                if str == type(data["inspection_of"]):
                    compatible.append(name)
                else:
                    compatible.append(name+"_U")
                    compatible.append(name+"_D")


        return compatible


    # -------------------------------- Iterator definition -------------------------------------

    def __iter__(self) -> "Tasks":
        self.__iter_counter = 0
        self.__iter_max = len(self.__list)
        return self


    def __next__(self) -> tuple:
        # Sets maximum number of iterations
        if self.__iter_counter < (self.__iter_max):
            self.__iter_counter += 1

            name = list(self.__list.keys())[self.__iter_counter-1]
            return name, self.__list[name]
        else:
            raise StopIteration

    