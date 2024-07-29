"""
Solver code. It includes several method to solve the GTSP-MUAV problem given the requiered input data
and settings.

As of now, all methods can be seen as a MILP problem than can solved using SCIP and its Python Interface PySCIPOpt
"""

import networkx as nx, pyscipopt as SCIP, numpy as np, copy
from itertools import combinations, chain, groupby
from time import time

from modules import bases as BA, tasks as TS, uavs as UAVS

A = 0.5
max_w = 6

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

        if "wind_vector" in kwargs and np.ndarray == type(kwargs["wind_vector"]):
            self.__wind_vector = kwargs["wind_vector"]
        else: self.__wind_vector = np.array([0,0,0])


        # Auxiliary parameters: 
        #   - The graph represents the abstract representation of the towers and bases. It dependes on the used solver method
        #   - The SCIP model is just another abstraction layer to communicate the actual MILP problem to SCIP

        self.__graph = nx.MultiDiGraph()
        self.__subgraphs = {}
        self.__simplified_subgraphs = {}
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
    
    def get_Subgraph(self, id: str):
        return self.__subgraphs[id]
    
    def get_Simplified_Subgraph(self, id: str):
        return self.__simplified_subgraphs[id]
    
    def get_Subgraphs(self):
        return self.__subgraphs
    
    def get_Simplified_Subgraphs(self):
        return self.__simplified_subgraphs
    
    def get_Wind(self):
        return self.__wind_vector
    
    def solve(self, **kwargs) -> dict:

        if "add_sigmas" in kwargs and bool == type(kwargs["add_sigmas"]): add_sigmasQ = kwargs["add_sigmas"]
        else: add_sigmasQ = True
        if "auto_uav_disabling" in kwargs and bool == type(kwargs["auto_uav_disabling"]): auto_uav_disablingQ = kwargs["auto_uav_disabling"]
        else: auto_uav_disablingQ = True
        if "cost_function" in kwargs: cost_functionQ = kwargs["cost_function"]
        else: cost_functionQ = "mtm"

        if "dynamic" in kwargs:
            if False == kwargs["dynamic"]:
                routes = solver(self, add_sigmas = add_sigmasQ, auto_uav_disabling = auto_uav_disablingQ, cost_function = cost_functionQ)
                return routes
            

        routes = dynamic_Solver(self, add_sigmas = add_sigmasQ, auto_uav_disabling = auto_uav_disablingQ, cost_function = cost_functionQ)
        return routes

def construct_Abstract_Graph(graph: nx.MultiDiGraph, bases: BA.Bases, towers: TS.Towers,
                              tasks: TS.Tasks, uavs: UAVS.UAV_Team):
    
    positions = dict(towers.get_Graph().nodes(data = "position"))

    #print(positions)
    
    # For each base, we need to add one vertex
    for name, position in bases:
        graph.add_node(name, start_position = position, end_position = position)
    
    # For each task, we need to add at least a vertex
    for name, data in tasks:

        if str == type(data["inspection_of"]): # Punctual inspection

            graph.add_node(
                name,
                inspection_of = data["inspection_of"],
                start_position = positions[data["inspection_of"]],
                end_position = positions[data["inspection_of"]]
            )

        elif tuple == type(data["inspection_of"]) and 2 == len(data["inspection_of"]):
            graph.add_node(
                name+"_U",
                inspection_of = data["inspection_of"],
                start_position = positions[data["inspection_of"][0]],
                end_position = positions[data["inspection_of"][1]]
            )
            graph.add_node(
                name+"_D",
                inspection_of = data["inspection_of"][::-1],
                start_position = positions[data["inspection_of"][1]],
                end_position = positions[data["inspection_of"][0]])

    
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

def compute_Subgraph(graph: nx.MultiDiGraph, uav:UAVS.UAV) -> nx.MultiDiGraph:

    return nx.from_edgelist([(i, j)  for i, j, k in graph.edges if k == uav.get_ID()])

def construct_SCIP_Model(graph: nx.MultiDiGraph, tasks: TS.Tasks, uavs: UAVS.UAV_Team, **kwargs) -> SCIP.Model:

    speeds = uavs.get_Speeds()
    ispeeds = uavs.get_Inspection_Speeds()

    start_positions = graph.nodes(data = "start_position")
    end_positions = graph.nodes(data = "end_position")

    #print(end_positions)

    scip_model = SCIP.Model("GTSP-MUAV")
    if "is_editable" in kwargs and True == kwargs["is_editable"]:
        scip_model.enableReoptimization()
        #scip_model.hideOutput()

    if "wind_vector" in kwargs and np.ndarray == type(kwargs["wind_vector"]):
        wind_vector = kwargs["wind_vector"]
    else: wind_vector = np.array([0,0,0])

    #print("wind", wind_vector)

    # For each pair of vertices in the graph and uav, we need to define a Z.
    Z = {}
    T = {}
    Y = {}
    Wt = {} # Time Weights
    pairs = list(combinations(graph.nodes, 2))

    orbit_radius = 10

    # They will be zero by default
    for pair in pairs:
        for uav in uavs:

            Z[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]] = 0
            Z[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]] = 0

            # These does not matter
            Wt[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]] = 0
            Wt[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]] = 0

            # O(n^2)
            T[uav.get_ID()+"T"+pair[0]+"-"+pair[1]] = scip_model.addVar(vtype = 'B', obj = 0.0, name = uav.get_ID()+"T"+pair[0]+"-"+pair[1])
            T[uav.get_ID()+"T"+pair[1]+"-"+pair[0]] = scip_model.addVar(vtype = 'B', obj = 0.0, name = uav.get_ID()+"T"+pair[1]+"-"+pair[0])

    # Except if they represent an existing edge

    for edge in graph.edges: # edge = (node1, node2, key)

        Z[edge[2]+"Z"+edge[0]+"-"+edge[1]] = scip_model.addVar(vtype = 'B', obj = 0.0, name = str(edge[2])+"Z"+edge[0]+"-"+edge[1])

        # Task transition
        move_vector = end_positions[edge[0]] - start_positions[edge[1]]
        d = np.linalg.norm(move_vector)
        if 0 == d:
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = 0
        else:
            effective_speed = speeds[edge[2]] - np.dot(wind_vector, move_vector) / d
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = d / effective_speed

        # Task Completion
        move_vector = end_positions[edge[1]] - start_positions[edge[1]]
        d = np.linalg.norm(move_vector)
        if 0 == d:
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + 2 * np.pi * orbit_radius / ispeeds[edge[2]]
        else:
            effective_speed = ispeeds[edge[2]] - np.dot(wind_vector, move_vector) / d
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + d / effective_speed

        if 0 > Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]]: print("Warning: Negative TIME COST. Wind speed might be too high!")


    for uav in uavs:

        edges = list(dict.fromkeys(list(graph.out_edges(uav.get_Base()))))
        
        if "auto_uav_disabling" in kwargs and False == kwargs["auto_uav_disabling"]:
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                1.0
            )

            # Base enter constraints
            edges = list(dict.fromkeys(list(graph.in_edges(uav.get_Base()))))
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                1.0
            )

        else:
            # UAV usage variables. One per uav
            Y[uav.get_ID()] = scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+str(uav.get_ID()))

            # Base exit constraints
            
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                Y[uav.get_ID()]
            )

            # Base enter constraints
            edges = list(dict.fromkeys(list(graph.in_edges(uav.get_Base()))))
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                Y[uav.get_ID()]
            )

    # Task Completion constraints
    for name, data in tasks:

        # Punctual inspection
        if str == type(data["inspection_of"]):
            
            edges = list(dict.fromkeys(list(graph.in_edges(name))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                            Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )    
                == 
                1
            )

            edges = list(dict.fromkeys(list(graph.out_edges(name))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                            Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )
                == 
                1
            )

        # Lineal inspection
        else:
            
            edges = list(dict.fromkeys(list(graph.in_edges(name+"_U"))+list(graph.in_edges(name+"_D"))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                        Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )
                == 
                1
            )

            edges = list(dict.fromkeys(list(graph.out_edges(name+"_U"))+list(graph.out_edges(name+"_D"))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                        Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )
                == 
                1
            )


    for uav in uavs:

        vertices = tasks.compatible_With(uav.get_ID())
        for name in vertices:
            
            
            Y[name+"|"+uav.get_ID()] = scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+name+"|"+uav.get_ID())

            
            edges = list(dict.fromkeys(list(graph.in_edges(name))+list(graph.out_edges(name))))
            scip_model.addCons(
                SCIP.quicksum(
                    Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                2 * Y[name+"|"+uav.get_ID()]
            )

        
        # 9h
        edges = [(i, j, k) for i, j, k in graph.edges if k == uav.get_ID() and i != uav.get_Base() and j != uav.get_Base()]

        for edge in edges:
            scip_model.addCons(
                Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] <= T[uav.get_ID()+"T"+edge[0]+"-"+edge[1]]
            )

        # 9i
            scip_model.addCons(
                T[uav.get_ID()+"T"+edge[0]+"-"+edge[1]] + T[uav.get_ID()+"T"+edge[1]+"-"+edge[0]]
                 <= 
                Y[edge[0]+"|"+uav.get_ID()]
            )


        # 9k
            scip_model.addCons(
                Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] + T[uav.get_ID()+"T"+edge[1]+"-"+edge[0]]
                 <= 
                Y[edge[0]+"|"+uav.get_ID()]
            )

        # 9j
        trinodes = list(combinations(vertices, 3))
        for tri in trinodes:
        
            scip_model.addCons(
                Z[uav.get_ID()+"Z"+tri[0]+"-"+tri[1]] + T[uav.get_ID()+"T"+tri[2]+"-"+tri[0]]
                 <= 
                T[uav.get_ID()+"T"+tri[2]+"-"+tri[1]] + 1.0
            )

   
    scip_model.addCons(
        T["0"+"TtT6-tT10"] == 1.0 #Y["tT7|0"] * Y["tT11|0"]
    )
    
    


    # The binary product x*y can be linearized by doing:
    #      z <= x; z <= y
    #      z >= x + y - 1
    # z represents the product as a new free variable.
    # https://or.stackexchange.com/questions/37/how-to-linearize-the-product-of-two-binary-variables
    

    
    # https://www.sciencedirect.com/science/article/pii/S0305054814001439

    return scip_model, Z, Wt, Y, T

def construct_SCIP_Model_OG(graph: nx.MultiDiGraph, tasks: TS.Tasks, uavs: UAVS.UAV_Team, **kwargs) -> SCIP.Model:

    speeds = uavs.get_Speeds()
    ispeeds = uavs.get_Inspection_Speeds()

    start_positions = graph.nodes(data = "start_position")
    end_positions = graph.nodes(data = "end_position")

    #print(end_positions)

    scip_model = SCIP.Model("GTSP-MUAV")
    if "is_editable" in kwargs and True == kwargs["is_editable"]:
        scip_model.enableReoptimization()
        #scip_model.hideOutput()

    if "wind_vector" in kwargs and np.ndarray == type(kwargs["wind_vector"]):
        wind_vector = kwargs["wind_vector"]
    else: wind_vector = np.array([0,0,0])

    #print("wind", wind_vector)

    # For each pair of vertices in the graph and uav, we need to define a Z.
    Z = {}
    Wt = {} # Time Weights
    pairs = list(combinations(graph.nodes, 2))

    orbit_radius = 10

    # They will be zero by default
    for pair in pairs:
        for uav in uavs:
            Z[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]] = 0
            Z[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]] = 0

            # These does not matter
            Wt[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]] = 0
            Wt[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]] = 0

    # Except if they represent an existing edge
    for edge in graph.edges: # edge = (node1, node2, key)

        Z[edge[2]+"Z"+edge[0]+"-"+edge[1]] = scip_model.addVar(vtype = 'B', obj = 0.0, name = str(edge[2])+"Z"+edge[0]+"-"+edge[1])

        # Task transition
        move_vector = end_positions[edge[0]] - start_positions[edge[1]]
        d = np.linalg.norm(move_vector)
        if 0 == d:
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = 0
        else:
            effective_speed = speeds[edge[2]] - np.dot(wind_vector, move_vector) / d
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = d / effective_speed

        # Task Completion
        move_vector = end_positions[edge[1]] - start_positions[edge[1]]
        d = np.linalg.norm(move_vector)
        if 0 == d:
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + 2 * np.pi * orbit_radius / ispeeds[edge[2]]
        else:
            effective_speed = ispeeds[edge[2]] - np.dot(wind_vector, move_vector) / d
            Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + d / effective_speed

        if 0 > Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]]: print("Warning: Negative TIME COST. Wind speed might be too high!")

      
    Y = {}
    T = {}
    for uav in uavs:

        edges = list(dict.fromkeys(list(graph.out_edges(uav.get_Base()))))
        
        if "auto_uav_disabling" in kwargs and False == kwargs["auto_uav_disabling"]:
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                1.0
            )

            # Base enter constraints
            edges = list(dict.fromkeys(list(graph.in_edges(uav.get_Base()))))
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                1.0
            )

        else:
            # UAV usage variables. One per uav
            Y[uav.get_ID()] = scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+str(uav.get_ID()))

            # Base exit constraints
            
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                Y[uav.get_ID()]
            )

            # Base enter constraints
            edges = list(dict.fromkeys(list(graph.in_edges(uav.get_Base()))))
            scip_model.addCons(
                SCIP.quicksum(
                    Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                Y[uav.get_ID()]
            )

    # Task Completion constraints
    for name, data in tasks:

        # Punctual inspection
        if str == type(data["inspection_of"]):
            
            edges = list(dict.fromkeys(list(graph.in_edges(name))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                            Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )    
                == 
                1
            )

            edges = list(dict.fromkeys(list(graph.out_edges(name))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                            Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )
                == 
                1
            )

        # Lineal inspection
        else:
            
            edges = list(dict.fromkeys(list(graph.in_edges(name+"_U"))+list(graph.in_edges(name+"_D"))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                        Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )
                == 
                1
            )

            edges = list(dict.fromkeys(list(graph.out_edges(name+"_U"))+list(graph.out_edges(name+"_D"))))
            scip_model.addCons(
                SCIP.quicksum(
                    SCIP.quicksum(
                        Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                    )
                for uav in uavs
                )
                == 
                1
            )

    for uav in uavs:
        vertices = tasks.compatible_With(uav.get_ID())
        for name in vertices:
            
            """
            Y[name+"|"+uav.get_ID()] = scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+name+"|"+uav.get_ID())

            
            edges = list(dict.fromkeys(list(graph.in_edges(name))+list(graph.out_edges(name))))
            scip_model.addCons(
                SCIP.quicksum(
                    Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                )
                == 
                2 * Y[name+"|"+uav.get_ID()]
            )
            """

            
            edges_in = list(dict.fromkeys(list(graph.in_edges(name))))
            edges_out = list(dict.fromkeys(list(graph.out_edges(name))))

            scip_model.addCons(
                SCIP.quicksum(
                    Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges_in
                )
                -
                SCIP.quicksum(
                    Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges_out
                )
                == 
                0.0
            )
        
        """
            T[name+"|"+uav.get_ID()] = scip_model.addVar(vtype = 'C', obj = len(vertices), name = "T"+name+"|"+uav.get_ID())

            scip_model.addCons(
                    T[name+"|"+uav.get_ID()]
                    >= 
                    2    # number of tasked vertices + 1 for the base
                )

        T[uav.get_Base()+"|"+uav.get_ID()] = 1

        edges = [(i, j, k) for i, j, k in graph.edges if k == uav.get_ID() and i != uav.get_Base() and j != uav.get_Base()]

        print(len(vertices))

        for edge in edges:
            
            scip_model.addCons(
                    T[edge[0]+"|"+uav.get_ID()] - T[edge[1]+"|"+uav.get_ID()] + 1
                    <= 
                    len(vertices) * (1-Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]) # number of tasked vertices + 1 for the base
                )
            
            # https://www.sciencedirect.com/science/article/pii/S0305054814001439

        """ 

    return scip_model, Z, Wt, Y, T

def add_Cost_Function(scip_model: SCIP.Model, which: str, uavs: UAVS.UAV_Team, Z: dict, Wt: dict, graph: nx.MultiDiGraph, **kwargs):

    match which:
        case "min_the_max" | "mtm":
            print("Using Min The max Cost Function")

            M = scip_model.addVar(vtype = 'C', obj = 1.0, name = "M")

            edges = list(dict.fromkeys([(edge[0], edge[1]) for edge in graph.edges]))

            for uav in uavs:

                scip_model.addCons(
                    SCIP.quicksum(
                        Wt[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] * Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                    )      
                    <= 
                    M
                )

        case "min-all-routes" | "mar":

            print("Using Min All Routes Cost Function")

            Sigmas = {}
            edges = list(dict.fromkeys([(edge[0], edge[1]) for edge in graph.edges]))

            for uav in uavs:

                Sigmas[uav.get_ID()] = scip_model.addVar(vtype = 'C', obj = 1.0, name = "S"+uav.get_ID())

                scip_model.addCons(
                    SCIP.quicksum(
                        Wt[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] * Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in edges
                    )      
                    <= 
                    Sigmas[uav.get_ID()]
                )

        case "min-the-sum" | "mts":

            print("Using Min The Sum Cost Function")

            if "add_sigmas" in kwargs and kwargs["add_sigmas"]:

                Sigmas = {}

                print("Time Difference optimization Added")
        
                # Time difference constraints
                uav_pairs = list(combinations(uavs.get_List().keys(), 2))
                edges = list(dict.fromkeys([edge[:2] for edge in graph.edges]))

                for uav1, uav2 in uav_pairs:
                    Sigmas[uav1+"-"+uav2] = scip_model.addVar(vtype = 'C', obj = 0.0, name = "Si"+uav1+"-"+uav2)

                    # Constraint 1
                    scip_model.addCons(
                        SCIP.quicksum(
                                Z[uav1+"Z"+edge[0]+"-"+edge[1]] * Wt[uav1+"Z"+edge[0]+"-"+edge[1]]
                            for edge in edges
                        )
                      - SCIP.quicksum(
                                Z[uav2+"Z"+edge[0]+"-"+edge[1]] * Wt[uav2+"Z"+edge[0]+"-"+edge[1]]
                            for edge in edges
                        )
                        <= 
                        Sigmas[uav1+"-"+uav2]
                    )
                    # Constraint 2
                    scip_model.addCons(
                        SCIP.quicksum(
                                Z[uav2+"Z"+edge[0]+"-"+edge[1]] * Wt[uav2+"Z"+edge[0]+"-"+edge[1]]
                            for edge in edges
                        )
                      - SCIP.quicksum(
                                Z[uav1+"Z"+edge[0]+"-"+edge[1]] * Wt[uav1+"Z"+edge[0]+"-"+edge[1]]
                            for edge in edges
                        )
                        <= 
                        Sigmas[uav1+"-"+uav2]
                    )

            if 0 == A:
                scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))
            elif 0 == (A - 1):
                scip_model.setObjective(SCIP.quicksum(sigma for sigma in Sigmas.values()))
            else:
                scip_model.setObjective(A * SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys())
                                + (1 - A) * SCIP.quicksum(sigma for sigma in Sigmas.values()))

        case _: raise Exception("No valid cost function selected")

    return None

def compute_Subtour_Subset_from_Route(route: list, base: str) -> list:

    from_vertices = []
    to_vertices = []

    for pair in route:
        from_vertices.append(pair[0])
        to_vertices.append(pair[1])

    vertices = list(dict.fromkeys(from_vertices + to_vertices))

    if base in vertices:
        vertices.remove(base)

    return vertices

def add_DFJ_Subtour_Constraint(Q: list, uav_id: str, Z:dict, model: SCIP.Model):

    if not Q: return None

    # Compute all pairs of Q vertices:
    pairs = list(combinations(Q, 2))

    if pairs:
        model.addCons(
            SCIP.quicksum(
                    Z[uav_id+"Z"+pair[0]+"-"+pair[1]] + Z[uav_id+"Z"+pair[1]+"-"+pair[0]]
                for pair in pairs)
                    <= 
                len(Q) - 1.0
        )

    return None

def add_Subpath_DFJ_Constraint(path: list, uav_id: str, Z:dict, model: SCIP.Model):

    if not path: return None

    model.addCons(
        SCIP.quicksum(
                Z[uav_id+"Z"+pair[0]+"-"+pair[1]]
            for pair in path)
                <=
            len(path) -  1
    )

    return None

def find_Loop(route: list) -> tuple[list, list]:

    from_nodes = copy.deepcopy([move[0] for move in route])
    to_nodes = copy.deepcopy([move[1] for move in route])

    moves_left = copy.deepcopy(route[1:])
    loop = [route[0]]

    search_node = to_nodes[0]

    notBackQ = True
    while notBackQ:

        idx = from_nodes.index(search_node)

        if to_nodes[0] == to_nodes[idx]:
            notBackQ = False
            break

        search_node = to_nodes[idx]
        loop.append((from_nodes[idx], to_nodes[idx]))
        moves_left.remove((from_nodes[idx], to_nodes[idx]))

    
    return loop, moves_left

def list_Loops(route: list) -> tuple[list]:

    loop_list = []

    #print("route", route)

    if not(route):
        return []

    loop, left = find_Loop(route)
    #print("loop, left", loop, left)

    loop_list.append(loop)
    #print("Loops", loop_list)

    while left:

        loop, left = find_Loop(left)
        #print("loop, left", loop, left)


        loop_list.append(loop)
        #print("Loops", loop_list)

    return loop_list

def does_Contain_Vertex(route: list, vertex: str) -> tuple[bool, list]:

    from_vertices = []
    to_vertices = []

    for move in route:
        from_vertices.append(move[0])
        to_vertices.append(move[1])

    vertices = list(dict.fromkeys(from_vertices+to_vertices))

    if vertex in vertices:
        return True, copy.deepcopy(vertices).remove(vertex)

    return False, vertices

def does_Contain_Vertices(route:list, vertices: list) -> bool:

    withinQ = len(vertices) * [False]

    for idx, vertex in enumerate(vertices):
        withinQ[idx], _ = does_Contain_Vertex(route, vertex)

    if all(withinQ):
        return True

    return False

def remove_Invalid_Paths(paths: list, complex_tasks:list):
    
    returned_paths = copy.deepcopy(paths)
    for path in paths:
        for complex_task in complex_tasks:
            if does_Contain_Vertices(path, [complex_task+"_U", complex_task+"_D"]): 
                returned_paths.remove(path)
                break

    return returned_paths

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

def order_Route(route: list, base: str):

    if not route:
        return []

    from_nodes = copy.deepcopy([move[0] for move in route])
    to_nodes = copy.deepcopy([move[1] for move in route])

    # Find the base
    idx = from_nodes.index(base)

    moves_left = copy.deepcopy(route[:idx] + route[idx+1 :])
    ordered_route = [route[idx]]

    search_node = to_nodes[idx]

    notBackQ = True
    while notBackQ:

        idx = from_nodes.index(search_node)

        search_node = to_nodes[idx]
        ordered_route.append((from_nodes[idx], to_nodes[idx]))
        moves_left.remove((from_nodes[idx], to_nodes[idx]))

        if base == to_nodes[idx]:
            notBackQ = False
            break

    return ordered_route

def order_Routes(routes:dict, uavs: UAVS.UAV_Team) -> dict:
    ordered_routes = {}
    for uav in uavs:

        ordered_routes[uav.get_ID()] = order_Route(routes[uav.get_ID()], uav.get_Base())


    return ordered_routes

def parse_Route(route: list, vertices_dict: dict) -> list:

    if not route:
        return []

    points = vertices_dict[route[0][0]] + vertices_dict[route[0][1]]

    for move in route[1:]:
        points = points + vertices_dict[move[1]]

    return points

def parse_Routes(routes: dict, vertices_dict: dict) -> dict:

    parsed = {}

    for uav_id in routes:
        parsed[uav_id] = parse_Route(routes[uav_id], vertices_dict)

    return parsed

def plan_Time(routes: dict, Wt: dict):

    current_max_id = ""
    current_max_time = -1

    for uav_id in routes:

        time = 0
        for edge in routes[uav_id]:
            time += Wt[uav_id+"Z"+edge[0]+"-"+edge[1]]
        
        print("(id, time): ", uav_id, time)
        
        if time > current_max_time: 
            current_max_id = uav_id
            current_max_time = time

    return current_max_id, current_max_time

def solver(problem: Problem, **kwargs) -> dict:

    print("Non-Dynamic Solver")

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    complex_tasks = tasks.get_Complex_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()
    subgraphs = problem.get_Subgraphs()
    simp_subgraphs = problem.get_Simplified_Subgraphs()

    if "auto_uav_disabling" in kwargs and bool == type(kwargs["auto_uav_disabling"]): auto_uav_disablingQ = kwargs["auto_uav_disabling"]
    else: auto_uav_disablingQ = True
    
    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs)

    scip_model, Z, Wt, Y = construct_SCIP_Model(abstract_G, tasks, uavs, wind_vector = problem.get_Wind(),
                                                        auto_uav_disabling = auto_uav_disablingQ)
    
    # Precedence constraints v1 (Brute Force) ------------------------------------------
    tasks_order = tasks.get_Order()
    for uav in uavs:

        id = uav.get_ID()
        sG = compute_Subgraph(abstract_G, uav)
        subgraphs[id] = sG
        ssG = copy.deepcopy(sG)
        ssG.remove_node(uav.get_Base())
        simp_subgraphs[id] = ssG

        paths_to_delete = []

        if id in tasks_order:
            for order_pair in tasks_order[id]:
            
                # We list all subpaths with the inverse precedence to delete all routes that contain them
                paths = list(nx.all_simple_edge_paths(ssG, order_pair[1], order_pair[0]))
                #print(id, paths, len(paths))
                paths = remove_Invalid_Paths(paths, complex_tasks)
                #print(id, paths, len(paths))

                paths_to_delete = paths_to_delete + paths

            # Remove duplicates (as list are not hashable, we need an unsual approach)
            paths_to_delete.sort()
            paths_to_delete = list(j for j, _ in groupby(paths_to_delete))

            # We delete each of them using DFJ constraints. Maybe, I can limit the length of deleted paths as larger paths won't fit optimality
            for path in paths_to_delete:
                add_Subpath_DFJ_Constraint(path, id, Z, scip_model)     

    # ----------------------------------------------------------------------------------

    #print(Wt)

    # Subtour elimination
    vertices = list(abstract_G.nodes)
    for uav in uavs:
        try:
            vertices.remove(uav.get_Base())
        except:
            None

    Qlist = list(chain.from_iterable(list(combinations(vertices, r)) for r in range(2, len(vertices)+1)))

    k = 1
    for Q in Qlist:
        #print("Q", Q)
        for uav in uavs:
            add_DFJ_Subtour_Constraint(Q, uav.get_ID(), Z, scip_model)
        
        print("Subtour Constraints Addition: ", 100 * k / len(Qlist),end="\r")
        k += 1

    # --------------------------------------------------------------------------------------------------

    if "cost_function" in kwargs:
        which = kwargs["cost_function"]
    else:
        which = "mtm"
    
    add_Cost_Function(scip_model, which, uavs, Z, Wt, abstract_G, **kwargs)

    #scip_model.writeProblem('scip_model.cip')
    t0 = time()
    scip_model.optimize()
    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)
    print("Solver Routes", routes)
    
    dt = time() - t0
    print("Solved in:", dt, "s")

    print("(ID, MAX. Plan Time): ", plan_Time(routes, Wt))

    return order_Routes(routes, uavs)

def dynamic_Solver(problem: Problem, **kwargs) -> dict:

    print("Dynamic Solver")

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    complex_tasks = tasks.get_Complex_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()
    subgraphs = problem.get_Subgraphs()
    simp_subgraphs = problem.get_Simplified_Subgraphs()

    if "add_sigmas" in kwargs and bool == type(kwargs["add_sigmas"]): add_sigmasQ = kwargs["add_sigmas"]
    else: add_sigmasQ = True
    if "auto_uav_disabling" in kwargs and bool == type(kwargs["auto_uav_disabling"]): auto_uav_disablingQ = kwargs["auto_uav_disabling"]
    else: auto_uav_disablingQ = True

    
    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs)
    scip_model, Z, Wt, Y, T = construct_SCIP_Model(abstract_G, tasks, uavs, add_sigmas = add_sigmasQ, is_editable = True,
                                                        wind_vector = problem.get_Wind(), auto_uav_disabling = auto_uav_disablingQ)
    
     # Precedence constraints v1 (Brute Force) ------------------------------------------
    tasks_order = tasks.get_Order()
    for uav in uavs:

        id = uav.get_ID()
        sG = compute_Subgraph(abstract_G, uav)
        subgraphs[id] = sG
        ssG = copy.deepcopy(sG)
        ssG.remove_node(uav.get_Base())
        simp_subgraphs[id] = ssG

        paths_to_delete = []

        if id in tasks_order:
            for order_pair in tasks_order[id]:
            
                # We list all subpaths with the inverse precedence to delete all routes that contain them
                paths = list(nx.all_simple_edge_paths(ssG, order_pair[1], order_pair[0]))
                #print(id, paths, len(paths))
                paths = remove_Invalid_Paths(paths, complex_tasks)
                #print(id, paths, len(paths))

                paths_to_delete = paths_to_delete + paths

            # Remove duplicates (as list are not hashable, we need an unsual approach)
            paths_to_delete.sort()
            paths_to_delete = list(j for j, _ in groupby(paths_to_delete))

            # We delete each of them using DFJ constraints. Maybe, I can limit the length of deleted paths as larger paths won't fit optimality
            for path in paths_to_delete:
                add_Subpath_DFJ_Constraint(path, id, Z, scip_model)     

    # ----------------------------------------------------------------------------------

    if "cost_function" in kwargs:
        which = kwargs["cost_function"]
    else:
        which = "mtm"
    
    add_Cost_Function(scip_model, which, uavs, Z, Wt, abstract_G, **kwargs)
    
    scip_model.writeProblem('scip_model.cip')
    print("-------------Initial Iteration-------------")
    t0 = time()

    scip_model.optimize()
    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)

    k = 1
    subroutesQ = True
    while subroutesQ:

        subroutesQ = False
    
        # Routes must be only one loop and contain the base
        print("  Routes: ", routes)
        for uav in uavs:
            
            loops = list_Loops(routes[uav.get_ID()])
            print("  Loops:")
            print("   - ID: "+uav.get_ID(), ": ", loops)

            # UAV is not active
            if 0 == len(loops): continue

            if 1 == len(loops) and does_Contain_Vertex(loops[0], uav.get_Base())[0]:
                continue
            
            Q_list = []

            subroutesQ = True
            scip_model.freeReoptSolve()
            for loop in loops:
                if not does_Contain_Vertex(loop, uav.get_Base())[0]:
                    Q = compute_Subtour_Subset_from_Route(loop, "")
                    Q_list.append(Q)
                

        
        if subroutesQ:

            for uav in uavs:
                for Q in Q_list:
                    
                    length = len(Q)

                    if max_w <= length:
                        print("Too large!")
                        subsets = chain.from_iterable(list(combinations(Q[:max_w], r)) for r in range(2, max_w+1))
                        add_DFJ_Subtour_Constraint(Q, uav.get_ID(), Z, scip_model)
                    else:
                        subsets = chain.from_iterable(list(combinations(Q, r)) for r in range(2, length+1))
                    

                    for Qs in subsets:
                        add_DFJ_Subtour_Constraint(Qs, uav.get_ID(), Z, scip_model)

        
            print("-------------Iteration: "+str(k)+"-------------")
            scip_model.optimize()
            sol = scip_model.getBestSol()
            routes = parse_Solution(sol, Z, uavs)

        k += 1

    dt = time() - t0
    print("Solved in:", dt, "s")

    print("(ID, MAX. Plan Time): ", plan_Time(routes, Wt))

    """
    for key in Y:
        if sol[Y[key]] == 1.0: print("Y"+key+" = ", sol[Y[key]])
    
    """
    for key in T:
        if sol[T[key]] == 1.0: print("T"+key+" = ", sol[T[key]])
    

    return order_Routes(routes, uavs)