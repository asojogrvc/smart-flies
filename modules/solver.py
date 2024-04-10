"""
Solver code. It includes several method to solve the GTSP-MUAV problem given the requiered input data
and settings.

As of now, all methods can be seen as a MILP problem than can solved using SCIP and its Python Interface PySCIPOpt
"""

import networkx as nx, pyscipopt as SCIP, numpy as np
from itertools import combinations

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
    
    def get_Graph(self):
        return self.__graph
    
    def solve(self, dynamicQ: bool):

        if dynamicQ:
            dynamic_Solver(self)
        else:
            solver(self)

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

    
    for uav in uavs:
        
        compatible_tasks = tasks.compatible_With(uav.get_ID())

        # Connect bases with compatible vertices
        for task in compatible_tasks:
            graph.add_edge(uav.get_Base(), task, uav.get_ID())
            graph.add_edge(task, uav.get_Base(), uav.get_ID())

        # Connect all compatible tasks except for the two vertices of the same lineal inspection
        pairs_of_tasks = list(combinations(compatible_tasks, 2))

        for pair in pairs_of_tasks:

            if pair[0].split("_")[0] != pair[1].split("_")[0]:
                graph.add_edge(pair[0], pair[1], uav.get_ID())
                graph.add_edge(pair[1], pair[0], uav.get_ID())
            

    return graph

def construct_SCIP_Model(graph: nx.MultiDiGraph, tasks: TS.Tasks, uavs: UAVS.UAV_Team) -> SCIP.Model:

    scip_model = SCIP.Model("GTSP-MUAV")

    # For each pair of vertices in the graph and uav, we need to define a Z.
    Z = {}
    pairs = list(combinations(graph.nodes, 2))

    # They will be zero by default
    for pair in pairs:
        for uav in uavs:
            Z[str(uav.get_ID())+"Z"+pair[0]+"-"+pair[1]] = 0
            Z[str(uav.get_ID())+"Z"+pair[1]+"-"+pair[0]] = 0

    # Except if they represent an existing edge
    for edge in graph.edges: # edge = (node1, node2, key)
        Z[str(edge[2])+"Z"+edge[0]+"-"+edge[1]] = scip_model.addVar(vtype = 'B', obj = 0.0, name = str(edge[2])+"Z"+edge[0]+"-"+edge[1])

    # UAV usage variables. One per uav
    Y = {
        uav.get_ID(): scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+str(uav.get_ID()))
        for uav in uavs
    }
    
    for uav in uavs:

        # Base exit constraints
        scip_model.addCons(
            SCIP.quicksum(
                Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                for edge in graph.out_edges(uav.get_Base())
            )
            == 
            Y[uav.get_ID()]
        )

        # Base enter constraints
        graph.in_edges()
        scip_model.addCons(
            SCIP.quicksum(
                Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                for edge in graph.in_edges(uav.get_Base())
            )
            == 
            Y[uav.get_ID()]
        )

    # Task Completion constraints
    for name, data in tasks:

        # Punctual inspection
        if str == type(data["inspection_of"]):
            
            scip_model.addCons(
                SCIP.quicksum(
                        Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in graph.in_edges(name)  # As this contains the key (ID), there is no need to sum for UAVs
                )
                == 
                1
            )

            scip_model.addCons(
                SCIP.quicksum(
                        Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in graph.out_edges(name)
                )
                == 
                1
            )

        # Lineal inspection
        else:
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in list(graph.in_edges(name+"_U"))+list(graph.in_edges(name+"_D"))
                )
                == 
                1
            )

            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in list(graph.out_edges(name+"_U"))+list(graph.out_edges(name+"_D"))
                )
                == 
                1
            )

    for uav in uavs:
        for name in tasks.compatible_With(uav.get_ID()):

            Y[name+"|"+str(uav.get_ID())] = scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+name+"|"+str(uav.get_ID()))


            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in list(graph.in_edges(name)) + list(graph.out_edges(name))
                )
                == 
                2 * Y[name+"|"+str(uav.get_ID())]
            )

    return scip_model, Z, Y

def order_Route(route: list) -> list:

    ordered_Route = []

    if 0 < len(route):

        start = ordered_Route[0]

    return ordered_Route

def parse_Solution(sol: SCIP.scip.Solution, Z: dict, uavs: UAVS.UAV_Team):

    routes = {str(uav.get_ID()): [] for uav in uavs}

    for edge in Z:

        try: value = sol[Z[edge]]
        except: value = 0

        if np.abs(value - 1.0) < 1e-6: # To avoid floating point errors
            
            edge_parts = edge.split("Z")
            uav_ID = edge_parts[0]
            nodes = tuple(edge_parts[1].split("-"))

            routes[uav_ID].append(nodes)

    return routes

def get_Subgraph(graph: nx.MultiDiGraph, id: str) -> list:

    edges = [(i, j, k)   for i, j, k in graph.edges if k == id]
    #vertices = set( [n for i, j, k in graph.edges if k == 2  for n in [i, j]] )

    sG = nx.MultiDiGraph()
    sG.add_edges_from(edges)

    return sG

def dynamic_Solver(problem: Problem):

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()


    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs, "")
    scip_model, Z, Y = construct_SCIP_Model(abstract_G, tasks, uavs)

    scip_model.setObjective(SCIP.quicksum(Z[key] for key in Z.keys()))
    scip_model.optimize()

    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)
    print(routes)

    return None

def solver(problem: Problem):


    return None