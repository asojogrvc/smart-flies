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

    print(end_positions)

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

            Y[name+"|"+uav.get_ID()] = scip_model.addVar(vtype = 'B', obj = 0.0, name = "Y"+name+"|"+uav.get_ID())


            scip_model.addCons(
                SCIP.quicksum(
                    Z[uav.get_ID()+"Z"+edge[0]+"-"+edge[1]]
                    for edge in list(graph.in_edges(name)) + list(graph.out_edges(name))
                )
                == 
                2 * Y[name+"|"+uav.get_ID()]
            )

    return scip_model, Z, Wt, Y

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

def add_DFJ_Subtour_Constraint(Q: list, uav_id: str, G: nx.MultiDiGraph, Z:dict, model: SCIP.Model):

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

    starting_move = route[0]

    loop = [starting_move, ]

    unlisted_moves = route.copy()


    checklist = unlisted_moves.copy()

    foundQ = True

    while foundQ:

        foundQ = False

        for move in checklist:
            if starting_move[1] == move[0]: #The next point of the loop is found

                loop.append(move)
                starting_move = move
                del unlisted_moves[checklist.index(move)]

                foundQ = True
                break

        if foundQ:
            checklist = unlisted_moves.copy()
    
    return loop, unlisted_moves

def list_Loops(route: list) -> tuple[list]:

    loop_list = []

    if not(route):
        return []

    loop, left = find_Loop(route)

    loop_list.append(loop)

    while left:

        loop, left = find_Loop(left)

        if not left:
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

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()


    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs, "")
    scip_model, Z, Wt, Y = construct_SCIP_Model(abstract_G, tasks, uavs)

    print(Wt)

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
            add_DFJ_Subtour_Constraint(Q, uav.get_ID(), abstract_G, Z, scip_model)

    scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))

    scip_model.writeProblem('scip_model.cip')

    scip_model.optimize()

    sol = scip_model.getBestSol()

    routes = parse_Solution(sol, Z, uavs)

    return routes

def dynamic_Solver(problem: Problem) -> dict:

    print("Dynamic Solver")

    bases = problem.get_Bases()
    towers = problem.get_Towers()
    tasks = problem.get_Tasks()
    uavs = problem.get_UAVs()
    abstract_G = problem.get_Graph()


    abstract_G = construct_Abstract_Graph(abstract_G, bases, towers, tasks, uavs, "")
    scip_model, Z, Wt, Y = construct_SCIP_Model(abstract_G, tasks, uavs, is_editable = True)


    scip_model.setObjective(SCIP.quicksum( Wt[key] * Z[key] for key in Z.keys()))
    scip_model.writeProblem('scip_model.cip')

    iter = 0
    print("------------- Initial Iteration ------------")
    scip_model.optimize()
    routes = parse_Solution(scip_model.getBestSol(), Z, uavs)

    scip_model.freeReoptSolve()
    
    # As of now, it just deletes routes without the base
    baseQ = False
    while not baseQ:

        for uav in uavs:
            baseQ, rest_of_vertices = does_Contain_Vertex(routes[uav.get_ID()], uav.get_Base())

            print("vertices", rest_of_vertices)

            if not baseQ:
                
                add_DFJ_Subtour_Constraint(rest_of_vertices, uav.get_ID(), abstract_G, Z, scip_model)

        scip_model.optimize()
        routes = parse_Solution(scip_model.getBestSol(), Z, uavs)
        
        scip_model.freeReoptSolve()

    for uav in uavs:
        print(list_Loops(routes[uav.get_ID()]))

    return routes