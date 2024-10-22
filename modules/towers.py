# Tower and Towers class

import numpy as np
import re
from pykml import parser
import copy
import modules.coordinates as CO
import itertools
import networkx as nx
import matplotlib.pyplot as plt



class Towers():
    """
    Towers are represented as a networkx graph.
        - Each node represents a single tower and contains its name, UTM coordinates (3D)
        - Each edge represents wire connections between towers
    It is assumed that all towers are within the same UTM Zone so we only need the local coordinates
    """

    def __init__(self):
        self.__graph = nx.empty_graph(0)

    # ---------------------------------- Parameter settings -------------------------------------
    
    def add_Tower(self, name: str, epsg3035_Coordinates: np.ndarray, connections: list):
        """
        Add tower (or node) to the towers graph based on its EPSG3035 coordinates and a list
        of connections
        """
        
        self.__graph.add_node(name, Coords = epsg3035_Coordinates)
        self.__graph.add_edges_from(connections)  # As it is, it will allow you to change 
                                                # interal connections that do not concern 
                                                # the added tower
        
        return None
        

    def load_File(self, file: str, onlineQ: bool):
        """
        Method that updates a tower graph using an offline file. Files need to follow Google Earth "kml"s
        format containing paths. As paths cannot represent several branches, the will be represented
        as path with at much one common tower per branch pair.
        If the boolean "onlineQ" is True, height coordinates will be updated online
        """

        # Check if the file a .kml
        if file.split(".")[-1] != "kml":
            raise Exception("File is not a .kml")

        # Reset first to avoid any data overlap
        self.reset()

        # Load raw kml file
        with open(file) as f:
            doc = parser.parse(f).getroot()
            f.close()

        # Parse data to extract paths
        paths = [] # Contains all paths within the file a graph or Towers
        tower_number = 1
        for path in doc.Document.Placemark.MultiGeometry.LineString:
            
            # temp graph to store each path / branch
            Tg = nx.empty_graph(0)

            # Parsing each tower latitude-longitude coordinates within a path
            path_str = np.array(re.split(',| ', path.coordinates.text.strip()))
            n_t = int(len(path_str)/3)
            path_latlon = path_str.astype(float).reshape((n_t, 3)) # Numerical conv and reshape to matrix
            
            # the kml puts longitude first, lets flip lat and long
            path_latlon[:, [0,1]] = path_latlon[:, [1, 0]] 

            # Update height from online API
            if onlineQ:
                CO.update_Height_Online(path_latlon)

            # Conversion to UTM. All towers and bases are assumed to be within the same zone
            path_epsg3035 = CO.latlon2epsg3035(path_latlon)

            # Add each of the towers of the branch in the actual graph 
            # and connect it to the previously added tower.
            Tg.add_node(f'T{tower_number}', Coords = path_epsg3035[0,:])
            tower_number += 1
            for k in range(1, n_t):
                Tg.add_node(f'T{tower_number}', Coords = path_epsg3035[k,:])
                Tg.add_edge(f'T{tower_number-1}', f'T{tower_number}')
                tower_number += 1

            paths.append(copy.deepcopy(Tg))

        self.__graph = delete_Common_Towers(paths)
        return None
    
    def load_from_Arrays(self, list_arrays:list, onlineQ: bool):
        """
        Method that updates a tower graph using a list of arrays, one for each path. As paths cannot represent several branches, the will be represented
        as path with at much at least one common tower per branch pair.
        If the boolean "onlineQ" is True, height coordinates will be updated online
        """

        # Reset first to avoid any data overlap
        self.reset()

        # Parse data to extract paths
        paths = [] # Contains all paths within the file a graph or Towers
        tower_number = 1
        for path_latlon in list_arrays:
            
            # temp graph to store each path / branch
            Tg = nx.empty_graph(0)

            n_t = len(path_latlon)

            # Update height from online API
            if onlineQ:
                CO.update_Height_Online(path_latlon)

            # Conversion to EPSG3035. All towers and bases are assumed to be within the same zone
            path_epsg3035 = CO.latlon2epsg3035(path_latlon)

            # Add each of the towers of the branch in the actual graph 
            # and connect it to the previously added tower.
            Tg.add_node(f'T{tower_number}', Coords = path_epsg3035[0,:])
            tower_number += 1
            for k in range(1, n_t):
                Tg.add_node(f'T{tower_number}', Coords = path_epsg3035[k,:])
                Tg.add_edge(f'T{tower_number-1}', f'T{tower_number}')
                tower_number += 1

            paths.append(copy.deepcopy(Tg))

        self.__graph = delete_Common_Towers(paths)
        return None

    def reset(self):
        """
        Reset the towers list to its empty default status
        """
        self.__graph = nx.empty_graph(0)
        self.__UTM_Zone = (0, "A")

        return None

    # ---------------------------------- Parameter Output -------------------------------------

    def get_ArrayCoordinates(self) -> np.ndarray:
        """
        Get an array that contains all the EPSG3035 coordinates of the present towers
        """

        return np.array(list(nx.get_node_attributes(self.__graph, 'Coords').values()))
    
    def get_DictCoordinates(self) -> dict:
        """
        Gets a dict that contains all the EPSG3035 coordinates of the present towers
        """

        return nx.get_node_attributes(self.__graph, 'Coords')
    
    def get_Towers(self) -> dict:
        """
        Returns a dictionary with all present towers as keys and all of their attributes (as a dict) as the values
        """
        return self.__graph.nodes(data = True)

    def get_Tower_Coordinates(self, which: str) -> np.ndarray:
        """
        Gets the EPSG3035 coordinates of an specific tower
        """
        return self.__graph.nodes[which]["Coords"]
    
    def get_Graph(self) -> nx.MultiDiGraph:
        """
        Gets the Networkx Graph representation of the towers
        """
        return self.__graph

    def print(self):
        """
        Prints the towers data
        """
        coords = self.get_DictCoordinates()

        print("----------------------------Towers------------------------------")
        for tower in coords:

            print("  ", end = "")
            print(tower+" with EPSG:3035 (E, N): "+str(coords[tower]))

        print("----------------------------------------------------------------")

        return None

    # ---------------------------------- Plotting ------------------------------------------------

    def plot(self, axes: plt.Axes):
        """
        Plots the towers using their UTM Coordinates into an existing Matplotlib axes
        """
        pos3D = self.get_DictCoordinates()

        # Project it into 2D
        pos2D = {}
        for key in pos3D:
            pos2D[key] = pos3D[key][0:2]

        nx.draw_networkx(self.__graph, pos = pos2D, ax=axes, with_labels = True)

#-------------------------------------- General Functions -----------------------------------------------
        
def check_Common_Towers(T1: nx.Graph, T2: nx.Graph, threshold: float) -> list:
    """
    Outputs a list of pair of names of common towers of graph T1 and T2 based on a 2D distance threshold.
    The first name of each pair is the tower name in T1 and the second in T2     
    """

    # For some reason, using height too breaks everything. Also, as it needs to wait for the API,
    # if a file is loaded several times in a rapid succesion, it will also break.

    common = []

    # Check for every possible pair of towers of each branch if the distance is short enough
    for pair in list(itertools.product(T1.nodes, T2.nodes)):
        if (np.linalg.norm(T1.nodes[pair[0]]['Coords'][0:2]-T2.nodes[pair[1]]['Coords'][0:2]) < threshold):
            # If it is, add the pair to the common towers list
            common.append((pair[0], pair[1]))

    return common

def delete_Common_Towers(paths: list) -> nx.Graph:
    """
    Given a list of paths (as Networkx graphs) it removes all repeated towers, connect all paths with common towers
    and rename towers based on distance to the origin. It outputs the entire grid defined by all paths 
    as a Networkx Graph
    """
    
    # Just check the pair paths that are not redundant
    indeces_set = list(itertools.combinations(range(len(paths)), 2))
 
    for indeces in indeces_set:
        i = indeces[0]
        j = indeces[1]

        # Gets the list of common towers (or nodes) of paths[i] and paths[j]
        C = check_Common_Towers(paths[i], paths[j], 15)

        # If there is at least 1 common tower, put the same number to the tower on both paths
        if not(len(C) == 0):
            for pair in C:
                paths[i] = nx.relabel_nodes(paths[i], {pair[0]:pair[0]+"D"}, copy = False)
                paths[j] = nx.relabel_nodes(paths[j], {pair[1]:pair[0]+"D"}, copy = False)
                # We add a "D" of duplicated to avoid overlapping with any other tower
                

    # networkx allow you to compose two graphs. If several nodes are present on two graphs with
    # the same names, then the composition will combine those nodes into one. This allow
    # us to delete duplicate towers.

    # Composing every path with renamed towers
    T = nx.empty_graph(0)
    for graph in paths:
         T = nx.compose(graph, T)

    # Rename Towers based distance to origin.
    k = 1
    mapping = {}
    nodes_sorted = sorted(T.nodes(), key=lambda n: np.linalg.norm(T.nodes[n]['Coords']))
    for node in nodes_sorted:
        mapping[node] = f'T{k}'
        k += 1
    
    # Outputs renamed graph
    return nx.relabel_nodes(T, mapping, copy = True)