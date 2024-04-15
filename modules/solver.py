"""
Solver code. It includes several method to solve the GTSP-MUAV problem given the requiered input data
and settings.

As of now, all methods can be seen as a MILP problem than can solved using SCIP and its Python Interface PySCIPOpt
"""

import networkx as nx, pyscipopt as SCIP, numpy as np, copy
from itertools import combinations, chain

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
    
    def solve(self, **kwargs) -> dict:

        if "dynamic" in kwargs:
            if False == kwargs["dynamic"]:
                routes = solver(self)
                return routes

        routes = dynamic_Solver(self)
        return routes

def construct_Abstract_Graph(graph: nx.MultiDiGraph, bases: BA.Bases, towers: TS.Towers,
                              tasks: TS.Tasks, uavs: UAVS.UAV_Team, weather: WT.Weather):
    
    positions = dict(towers.get_Graph().nodes(data = "position"))

    #print(positions)
    
    # For each base, we need to add one vertex
    for name, position in bases:
        graph.add_node(name, start_position = position, end_position = position)
    
    # For each task, we need to add at least a vertex
    for name, data in tasks:

        if str == type(data["inspection_of"]): # Punctual inspection

            graph.add_node(name, inspection_of = data["inspection_of"], start_position = positions[data["inspection_of"]], end_position = positions[data["inspection_of"]])

        elif tuple == type(data["inspection_of"]) and 2 == len(data["inspection_of"]):
            graph.add_node(name+"_U", inspection_of = data["inspection_of"], start_position = positions[data["inspection_of"][0]], end_position = positions[data["inspection_of"][1]])
            graph.add_node(name+"_D", inspection_of = data["inspection_of"][::-1], start_position = positions[data["inspection_of"][1]], end_position = positions[data["inspection_of"][0]])

    
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

def construct_SCIP_Model(graph: nx.MultiDiGraph, tasks: TS.Tasks, uavs: UAVS.UAV_Team, **kwargs) -> SCIP.Model:

    start_positions = graph.nodes(data = "start_position")
    end_positions = graph.nodes(data = "end_position")

    #print(end_positions)

    scip_model = SCIP.Model("GTSP-MUAV")
    if "is_editable" in kwargs and True == kwargs["is_editable"]:
        scip_model.enableReoptimization()
        scip_model.hideOutput()

    # For each pair of vertices in the graph and uav, we need to define a Z.
    Z = {}
    Wt = {} # Time Weights
    pairs = list(combinations(graph.nodes, 2))

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

        Wt[edge[2]+"Z"+edge[0]+"-"+edge[1]] = np.linalg.norm(end_positions[edge[0]] - start_positions[edge[1]])

    # UAV usage variables. One per uav
    Y = {
        uav.get_ID(): scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+str(uav.get_ID()))
        for uav in uavs
    }
    
    for uav in uavs:

        # Base exit constraints
        edges = list(dict.fromkeys(list(graph.out_edges(uav.get_Base()))))
        scip_model.addCons(
            SCIP.quicksum(
                Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                for edge in edges
            )
            == 
            1 #Y[uav.get_ID()]
        )

        # Base enter constraints
        edges = list(dict.fromkeys(list(graph.in_edges(uav.get_Base()))))
        scip_model.addCons(
            SCIP.quicksum(
                Z[str(uav.get_ID())+"Z"+edge[0]+"-"+edge[1]]
                for edge in edges
            )
            == 
            1 # Y[uav.get_ID()]
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
        for name in tasks.compatible_With(uav.get_ID()):

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

    Sigmas = {}
    if "add_sigmas" in kwargs and kwargs["add_sigmas"]:

        print("Time Difference optimization Added")
        
        # Time difference constraints
        uav_pairs = pairs = list(combinations(uavs.get_List().keys(), 2))
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
            # Constraint 1
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

    return scip_model, Z, Wt, Y, Sigmas

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

    # Delete pairs that are not edges in the graph
    #for pair in pairs:
    #    if not G.has_edge(pair[0], pair[1], key = uav_id):
    #        pairs.remove(pair)
    #    if not G.has_edge(pair[1], pair[0], key = uav_id):
    #        try: pairs.remove(pair)
    #        except: None # It is already removed

    #print("Pairs", pairs)

    if pairs:
        model.addCons(
            SCIP.quicksum(
                    Z[uav_id+"Z"+pair[0]+"-"+pair[1]] + Z[uav_id+"Z"+pair[1]+"-"+pair[0]]
                for pair in pairs)
                    <= 
                len(Q) - 1.0
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

    if not(route):
        return []

    loop, left = find_Loop(route)
    # print("loop, left", loop, left)

    loop_list.append(loop)

    while left:

        loop, left = find_Loop(left)
        # print("loop, left", loop, left)

        if left:
            loop_list.append(loop)

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

    edges = [(i, j, k)   for i, j, k in graph.edges if k == str(id)]
    #vertices = set( [n for i, j, k in graph.edges if k == 2  for n in [i, j]] )

    sG = nx.MultiDiGraph()
    sG.add_edges_from(edges)

    return sG

def solver(problem: Problem) -> dict:

    print("Non-Dynamic Solver")

    A = 0.25

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()

    
    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs, "")
    scip_model, Z, Wt, Y, Sigmas = construct_SCIP_Model(abstract_G, tasks, uavs, add_sigmas = True)

    #print(Wt)

    # Subtour elimination
    vertices = list(abstract_G.nodes)
    for uav in uavs:
        try:
            vertices.remove(uav.get_Base())
        except:
            None

    Qlist = chain.from_iterable(list(combinations(vertices, r)) for r in range(2, len(vertices)+1))

    for Q in Qlist:
        #print("Q", Q)
        for uav in uavs:
            add_DFJ_Subtour_Constraint(Q, uav.get_ID(), Z, scip_model)
    # --------------------------------------------------------------------------------------------------
    if 0 == A:
        scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))
    if 0 == (A - 1):
        scip_model.setObjective(SCIP.quicksum(sigma for sigma in Sigmas.values()))
    else:
        scip_model.setObjective(A * SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys())
                                + (1 - A) * SCIP.quicksum(sigma for sigma in Sigmas.values()))

    scip_model.writeProblem('scip_model.cip')
    scip_model.optimize()
    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)

    return routes

def dynamic_Solver(problem: Problem) -> dict:

    A = 0.25

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()

    
    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs, "")
    scip_model, Z, Wt, Y, Sigmas = construct_SCIP_Model(abstract_G, tasks, uavs, add_sigmas = True)

    if 0 == A:
        scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))
    if 0 == (A - 1):
        scip_model.setObjective(SCIP.quicksum(sigma for sigma in Sigmas.values()))
    else:
        scip_model.setObjective(A * SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys())
                                + (1 - A) * SCIP.quicksum(sigma for sigma in Sigmas.values()))

    scip_model.writeProblem('scip_model.cip')

    
    print("-------------Initial Iteration-------------")
    scip_model.optimize()
    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)

    k = 1
    subroutesQ = True
    while subroutesQ:
    
        # Routes must be only one loop and contain the base
        for uav in uavs:
            loops = list_Loops(routes[uav.get_ID()])
            ("  Loops:")
            print("   - ID: "+uav.get_ID(), ": ", loops)

            # AQUI CHECK DE QUE NO NECESARIAMENTE EL PRIMER LOOP ES EL DE LA BASE
            if 1 == len(loops) and does_Contain_Vertex(loops[0], uav.get_Base()):
                subroutesQ = False
                
                # Y AQUI QUÉ
                for loop in loops: # AQUI VAN TODOS LOS LOOPS SIN LA BASE
                    Q = compute_Subtour_Subset_from_Route(loop, "")
                    add_DFJ_Subtour_Constraint(Q, uav.get_ID(), Z, scip_model)

        print("-------------Iteration: "+str(k)+"-------------")

        if subroutesQ:

            scip_model.optimize()
            sol = scip_model.getBestSol()
            routes = parse_Solution(sol, Z, uavs)

        k += 1
        subroutesQ = False

    return routes