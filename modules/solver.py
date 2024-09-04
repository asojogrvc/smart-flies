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
            if True == kwargs["dynamic"]:
                if not self.__tasks.get_Order():
                    routes = dynamic_Solver(self, add_sigmas = add_sigmasQ, auto_uav_disabling = auto_uav_disablingQ, cost_function = cost_functionQ)
                    return routes
                print("Precedence constraint are present. Dynamic Solver is not compatible. Changing to the regular solver.")
            
        routes = solver(self, add_sigmas = add_sigmasQ, auto_uav_disabling = auto_uav_disablingQ, cost_function = cost_functionQ)
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

        if "inspection_of" in data:
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
        
        if "custom_task_at" in data:

            graph.add_node(
                    name,
                    custom_task_at = data["custom_task_at"],
                    start_position = positions[data["custom_task_at"]],
                    end_position = positions[data["custom_task_at"]],
                    time_costs = data["cost"]
                )
        

    
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

def construct_SCIP_Model(graph: nx.MultiDiGraph, tasks: TS.Tasks, uavs: UAVS.UAV_Team, **kwargs) -> tuple[SCIP.Model, dict, dict, dict, dict]:

    speeds = uavs.get_Speeds()
    ispeeds = uavs.get_Inspection_Speeds()

    start_positions = graph.nodes(data = "start_position")
    end_positions = graph.nodes(data = "end_position")
    custom_costs_temp = list(graph.nodes(data = "time_costs"))
    custom_costs = {}

    for pair in custom_costs_temp:
        if None != pair[1]: custom_costs[pair[0]] = pair[1]
            

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
    Y = {}
    Wt = {} # Time Weights
    V_costs = {}

    for node in list(graph):
            for uav in uavs:
                Y[node+"|"+uav.get_ID()] = 0.0
                V_costs[node+"|"+uav.get_ID()] = 0.0

    pairs = list(combinations(graph.nodes, 2))

    orbit_radius = 10

    # They will be zero by default
    for pair in pairs:
        for uav in uavs:

            Z[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]] = 0
            Z[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]] = 0

            # This does not matter
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
        if edge[1] in custom_costs:
            if edge[2] in custom_costs[edge[1]]:
                V_costs[edge[1]+"|"+edge[2]] = custom_costs[edge[1]][edge[2]]
                Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + custom_costs[edge[1]][edge[2]]
        
        else:
            move_vector = end_positions[edge[1]] - start_positions[edge[1]]
            d = np.linalg.norm(move_vector)
            if 0 == d:
                if "B" != edge[1][-1]:
                    V_costs[edge[1]+"|"+edge[2]] = 2 * np.pi * orbit_radius / ispeeds[edge[2]]
                    Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + V_costs[edge[1]+"|"+edge[2]]
                else: None
            else:
                effective_speed = ispeeds[edge[2]] - np.dot(wind_vector, move_vector) / d
                V_costs[edge[1]+"|"+edge[2]] = d / effective_speed
                Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] + V_costs[edge[1]+"|"+edge[2]]

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

        if "inspection_of" in data:
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
            
        else:
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

    return scip_model, Z, Wt, V_costs, Y

def add_Cost_Function(scip_model: SCIP.Model, which: str, uavs: UAVS.UAV_Team, Z: dict, Wt: dict, graph: nx.MultiDiGraph, **kwargs):

    try: 
        Ot = kwargs["Ot"]
    except:
        Ot = {}
        for uav in uavs:
            Ot[uav.get_ID()] = 0.0

    match which:
        case "min_the_max" | "mtm":
            print("Using Min The max Cost Function")

            M = scip_model.addVar(vtype = 'C', obj = 1.0, name = "M")

            edges = list(dict.fromkeys([(edge[0], edge[1]) for edge in graph.edges]))

            for uav in uavs:

                try:
                    scip_model.addCons(
                        SCIP.quicksum(
                            Wt[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] * Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                        )
                        + Ot[uav.get_ID()]      
                        <= 
                        M
                    )
                except:
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

                try:
                    scip_model.addCons(
                        SCIP.quicksum(
                            Wt[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] * Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                        for edge in edges
                        )    
                        + Ot[uav.get_ID()]   
                        <= 
                        Sigmas[uav.get_ID()]
                    )
                except:
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

            try:
                if 0 == A:
                    scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()) + SCIP.quicksum( Ot[uav.get_ID()] for uav in uavs))
                elif 0 == (A - 1):
                    scip_model.setObjective(SCIP.quicksum(sigma for sigma in Sigmas.values()))
                else:
                    scip_model.setObjective(A * ( SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()) + SCIP.quicksum( Ot[uav.get_ID()] for uav in uavs))
                                + (1 - A) * SCIP.quicksum(sigma for sigma in Sigmas.values()))
            except:
                if 0 == A:
                    scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))
                elif 0 == (A - 1):
                    scip_model.setObjective(SCIP.quicksum(sigma for sigma in Sigmas.values()))
                else:
                    scip_model.setObjective(A * ( SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))
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

def add_MTZ_Subtour_Constraints(nodelist: list, Wt:dict, tasks: TS.Tasks, uavs: UAVS.UAV_Team, Z:dict, Y:dict, scip_model: SCIP.Model) -> tuple[dict, dict, dict, dict]:

    U = {}
    Tf = {}
    Ot = {}
    ms = {}

    if tasks.get_Precedence() and len(uavs.get_List()) != 1:
        # Wait time variable
        for uav in uavs:
            Ot[uav.get_ID()] = scip_model.addVar(vtype = 'C', obj = 0.0, name = "Ot"+uav.get_ID())

            scip_model.addCons(
                    Ot[uav.get_ID()] >= 0.0
            )

    if tasks.get_Precedence():
        for uav in uavs:
            for node in nodelist:
                Tf[node+"|"+uav.get_ID()] = 0.0

    for uav in uavs:

        vertices = tasks.compatible_With(uav.get_ID())
        m = len(vertices)
        ms[uav.get_ID()] = m

        edges = list(combinations(vertices+[uav.get_Base()], 2))
        for name in vertices:
            
            U[name+"|"+uav.get_ID()] = scip_model.addVar(vtype = 'I', obj = 0.0, name = "U"+name+"|"+uav.get_ID())
            

            scip_model.addCons(
                U[name+"|"+uav.get_ID()] <= m * Y[name+"|"+uav.get_ID()]
            )

            scip_model.addCons(
                U[name+"|"+uav.get_ID()] >= Z[uav.get_ID()+"Z"+uav.get_Base()+"-"+name]
            )

            if tasks.get_Precedence():
                Tf[name+"|"+uav.get_ID()] = scip_model.addVar(vtype = 'C', obj = 0.0, name = "Tf"+name+"|"+uav.get_ID())

                scip_model.addCons(
                    Tf[name+"|"+uav.get_ID()] <= 1E6 * Y[name+"|"+uav.get_ID()]
                )


                if len(uavs.get_List()) != 1:

                    scip_model.addCons(
                        Tf[name+"|"+uav.get_ID()] >= Wt[uav.get_ID()+"Z"+uav.get_Base()+"-"+name] * Z[uav.get_ID()+"Z"+uav.get_Base()+"-"+name] + Ot[uav.get_ID()]
                    )

                    scip_model.addCons(
                        Tf[name+"|"+uav.get_ID()] 
                        <=
                        SCIP.quicksum(
                            Wt[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] * Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] +
                            Wt[uav.get_ID()+"Z"+edge[1]+"-"+edge[0]] * Z[uav.get_ID()+"Z"+edge[1]+"-"+edge[0]]
                            for edge in edges
                        )
                        + 
                        Ot[uav.get_ID()]
                        -
                        SCIP.quicksum(
                            Wt[uav.get_ID()+"Z"+v+"-"+uav.get_Base()] * Z[uav.get_ID()+"Z"+v+"-"+uav.get_Base()]
                            for v in vertices
                        )
                    )
                else:

                    scip_model.addCons(
                        Tf[name+"|"+uav.get_ID()] >= Wt[uav.get_ID()+"Z"+uav.get_Base()+"-"+name] * Z[uav.get_ID()+"Z"+uav.get_Base()+"-"+name]
                    )

                    scip_model.addCons(
                        Tf[name+"|"+uav.get_ID()] 
                        <=
                        SCIP.quicksum(
                            Wt[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] * Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]] +
                            Wt[uav.get_ID()+"Z"+edge[1]+"-"+edge[0]] * Z[uav.get_ID()+"Z"+edge[1]+"-"+edge[0]]
                            for edge in edges
                        )
                        -
                        SCIP.quicksum(
                            Wt[uav.get_ID()+"Z"+v+"-"+uav.get_Base()] * Z[uav.get_ID()+"Z"+v+"-"+uav.get_Base()]
                            for v in vertices
                        )
                    )


        nodes = list(combinations(vertices, 2))
        for pair in nodes:
            scip_model.addCons(
                U[pair[0]+"|"+uav.get_ID()] - U[pair[1]+"|"+uav.get_ID()] + 1 <=  m * (1 - Z[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]])
            )
            scip_model.addCons(
                U[pair[1]+"|"+uav.get_ID()] - U[pair[0]+"|"+uav.get_ID()] + 1 <=  m * (1 - Z[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]])
            )

            if tasks.get_Precedence():

                scip_model.addCons(
                    Tf[pair[0]+"|"+uav.get_ID()] - Tf[pair[1]+"|"+uav.get_ID()] + Wt[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]] <=  1E6 * (1 - Z[uav.get_ID()+"Z"+pair[0]+"-"+pair[1]])
                )
                scip_model.addCons(
                    Tf[pair[1]+"|"+uav.get_ID()] - Tf[pair[0]+"|"+uav.get_ID()] + Wt[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]] <=  1E6 * (1 - Z[uav.get_ID()+"Z"+pair[1]+"-"+pair[0]])
                )   

    return U, Tf, Ot, ms

def add_Order_Constraint(uav_id: str, tasks_order: list[str], m: int, U:dict, Y:dict, scip_model:SCIP.Model):

    # tasks_order[1] > tasks_order[0]
    scip_model.addCons(
        U[tasks_order[1]+"|"+uav_id] - U[tasks_order[0]+"|"+uav_id] >= - m * (2 - Y[tasks_order[1]+"|"+uav_id] - Y[tasks_order[0]+"|"+uav_id] )
    )

    return None

def add_Precedence_Constraint(uavs: UAVS.UAV_Team, tasks_p: list[str], V_costs: dict, Tf:dict, Y:dict, scip_model:SCIP.Model):

    scip_model.addCons(
        SCIP.quicksum(
                Tf[tasks_p[0]+"|"+uav.get_ID()]# End time
            for uav in uavs
        )
            <=
        SCIP.quicksum(
                Tf[tasks_p[1]+"|"+uav.get_ID()] - V_costs[tasks_p[1]+"|"+uav.get_ID()] * Y[tasks_p[1]+"|"+uav.get_ID()]  # Start time
            for uav in uavs
        )
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
        
        print("(id, time without waiting time): ", uav_id, time)
        
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
    task_order = tasks.get_Order()
    task_precedence = tasks.get_Precedence()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()
    subgraphs = problem.get_Subgraphs()
    simp_subgraphs = problem.get_Simplified_Subgraphs()

    if "auto_uav_disabling" in kwargs and bool == type(kwargs["auto_uav_disabling"]): auto_uav_disablingQ = kwargs["auto_uav_disabling"]
    else: auto_uav_disablingQ = True
    
    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs)
    nodelist = list(abstract_G)


    scip_model, Z, Wt, V_costs, Y = construct_SCIP_Model(abstract_G, tasks, uavs, wind_vector = problem.get_Wind(),
                                                        auto_uav_disabling = auto_uav_disablingQ)
    
  
    # --------------------------------------------------------------------------------------------------
    #                                     Subtour Elimination
    # --------------------------------------------------------------------------------------------------

    U, Tf, Ot, ms = add_MTZ_Subtour_Constraints(nodelist, Wt, tasks, uavs, Z, Y, scip_model)
    
    print(Tf)

    # --------------------------------------------------------------------------------------------------
    #                                      Order Constraints
    # --------------------------------------------------------------------------------------------------
    for uav_id in task_order:
        print("Adding order constraints to model")
        m = ms[uav_id]
        for pair in task_order[uav_id]:
            add_Order_Constraint(uav_id, pair, m, U, Y, scip_model)

    # --------------------------------------------------------------------------------------------------
    #                                    Precedence Constraints
    # --------------------------------------------------------------------------------------------------
    if task_precedence: print("Adding precedence constraints to model")
    for pair in task_precedence:
        #print(pair)
        add_Precedence_Constraint(uavs, pair, V_costs, Tf, Y, scip_model)


    if "cost_function" in kwargs:
        which = kwargs["cost_function"]
    else:
        which = "mtm"
    
    add_Cost_Function(scip_model, which, uavs, Z, Wt, abstract_G, **kwargs, Ot = Ot)

    scip_model.writeProblem('scip_model.cip')
    t0 = time()
    scip_model.optimize()
    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)
    print("Solver Routes", routes)
    
    dt = time() - t0
    print("Solved in:", dt, "s")

    print("(ID, MAX. Plan Time): ", plan_Time(routes, Wt))

    for key in Tf:
        try:
            print(key, sol[Tf[key]]-V_costs[key] * sol[Y[key]], sol[Tf[key]])
        except: None
    
    for key in Ot:
        print("Ot", key, sol[Ot[key]])


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
    scip_model, Z, Wt, Wtb, Y = construct_SCIP_Model(abstract_G, tasks, uavs, add_sigmas = add_sigmasQ, is_editable = True,
                                                        wind_vector = problem.get_Wind(), auto_uav_disabling = auto_uav_disablingQ)
    
   
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

    return order_Routes(routes, uavs)