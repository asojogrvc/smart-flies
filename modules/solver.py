"""
Solver code. It includes several method to solve the GTSP-MUAV problem given the requiered input data
and settings.

As of now, all methods can be seen as a MILP problem than can solved using SCIP and its Python Interface PySCIPOpt
"""

import networkx as nx
import pyscipopt as SCIP

from modules import bases as BA, tasks as TS, uavs as UAVS, weather as WT

class Problem():
    def __init__(self, bases:BA.Bases, towers: TS.Towers, tasks: TS.Tasks, uavs: UAVS.UAV_Team, **kwargs):
        """
        This class contains everything needed to define the tasks planing problem.
        It can be solved with of the already impleted method by using .solve()
        """
       
        # Required input data
        self.__bases = bases
        self.__towers = towers
        self.__tasks = tasks 
        self.__uavs = uavs

        if "weather" in kwargs and WT.Weather == type(kwargs["weather"]):
            self.__weather = kwargs["weather"]

        # Auxiliary parameters: 
        #   - The graph represents the abstract representation of the towers and bases. It dependes on the used solver method
        #   - The SCIP model is just another abstraction layer to communicate the actual MILP problem to SCIP

        self.__graph = nx.MultiDiGraph()
        self.__scip_model = SCIP.Model("MW-TP")
        self.__scip_model.enableReoptimization()

        return None

    def get_Bases(self):
        return self.__bases
    
    def get_Towers(self):
        return self.__towers
    
    def get_Tasks(self):
        return self.__tasks
    
    def get_UAVs(self):
        return self.__uavs
    
    def solve(self, dynamicQ: bool):

        if dynamicQ:
            dynamic_Solver(self)
        else:
            solver(self)

def dynamic_Solver(problem: Problem):
    return None

def solver(problem: Problem):
    return None

def construct_Abstract_Graph(graph: nx.MultiDiGraph, bases: BA.Bases, towers: TS.Towers,
                              tasks: TS.Tasks, uavs: UAVS.UAV_Team, weather: WT.Weather):
    
    # For each base, we need to add one vertex
    for name, position in bases:
        graph.add_node(name, position = position)
    
    # For each task, we need to add at least a vertex
    for name, data in tasks:

        if str == type(data["inspection_of"]): # Punctual inspection
            graph.add_node(name, inspection_of = data["inspection_of"])

        elif tuple == type(data["inspection_of"]) and 2 == len(data["inspection_of"]):
            graph.add_node(name+"_U", inspection_of = data["inspection_of"])
            graph.add_node(name+"_D", inspection_of = data["inspection_of"][::-1])

    # Connect bases with compatible vertices
    for uav in uavs:
        uav.get_Base()


    return graph