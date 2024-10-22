"""
Solver code. It includes several method to solve the GTSP-MUAV problem given the requiered input data
and settings.

As of now, all methods can be seen as a MILP problem than can solved using SCIP and its Python Interface PySCIPOpt
"""

import networkx as nx, copy
import numpy as np
import itertools
import pyscipopt as SCIP # Model, quicksum, SCIP_PARAMSETTING
from PyQt6.QtWidgets import QProgressBar

from modules import bases as BA, towers as TW, uav as UAVS, weather as WT, weights as WE

class Problem():
    def __init__(self, id:str, towers: TW.Towers, bases: BA.Bases, uavs: UAVS.UAV_Team, weather: WT.Weather, mode: int, **kwargs):
        """
        This class contains everything needed to define the GTSP-MUAV problem in any of its forms. That is Towers, Bases, UAV Team and weather conditions.
        It can be solved with of the already impleted methods by using .solve()
        """
       
        # Required input data
        self.__towers = towers   
        self.__bases = bases   
        self.__uav_team = uavs
        self.__weather = weather
        self.__id = id

        # Use case mode:
        #   0 = Segment Inspection
        #   1 = Point Inspection
        self.__mode = mode

        # This allows the introduction of arbitrary external parameters into the problem. 
        if "Parameters" in kwargs:
            self.__mode_parameters = kwargs["Parameters"]

        # Auxiliary parameters: 
        #   - The graph represents the abstract representation of the towers and bases. It dependes on the used solver method
        #   - The SCIP model is just another abstraction layer to communicate the actual MILP problem to SCIP
        #   - The progressBar is linked to any progress on the UI to then update it from within the class
        self.__graph = nx.MultiDiGraph()
        self.__scip_model = SCIP.Model("GTSP-MUAV")
        self.__scip_model.enableReoptimization()
        self.__progressBar = None

        return None
    
    # ------------------------ Parameters setting functions ----------------
    # As to get an problem instance, all the requiered parameters are set,
    # these functions just replace the current values

    def set_Towers(self, towers: TW.Towers):
        self.__towers = towers
        return None
    
    def set_Bases(self, bases: BA.Bases):
        self.__bases = bases
        return None
    
    def set_UAV_Team(self, uavs: UAVS.UAV_Team):
        self.__uav_team = uavs
        return None
    
    def set_Weather(self, weather: WT.Weather):
        self.__weather = weather
        return None
    
    def set_Mission_Mode(self, mode:int):
        self.__mode = mode
        return None
    
    def set_ID(self, id: str):
        self.__id = id
        return None

    # ------------------------ Parameters output functions -----------------

    def get_Towers(self) -> TW.Towers:
        return self.__towers
    
    def get_Bases(self) -> BA.Bases:
        return self.__bases
    
    def get_UAV_Team(self) -> UAVS.UAV_Team:
        return self.__uav_team
    
    def get_Weather(self) -> WT.Weather:
        return self.__weather
    
    def get_Graph(self) -> nx.MultiDiGraph:
        return self.__graph
    
    def get_SCIP_Model(self) -> SCIP.Model:
        return self.__scip_model
    
    def get_Mission_Mode(self) -> int:
        return self.__mode
    
    def get_ID(self) -> str:
        return self.__id

    # ------------------------ Auxiliary functions -------------------------
    def set_Progress_Bar_Value(self, value: float):
        self.__progressBar.setValue(value)
        return None
    
    def progress_Bar_LinkedQ(self) -> bool:
        return self.__progressBar != None
    
    def solve(self, which_solver:str) -> bool:
        """
        Solves an existing problem with any of the implement formulations / methods. As of now:
            - "abstract_DFJ": This formulation creates a abstraction layer that assigns a graph to the problem which nodes
                are not the towers but the power line segments or towers that require inspection. It uses an adaptation
                of the DFJ subtour elimination constraints. Too Slow to be used.
            - "abstract_dynamic_DFJ": It is the same as the previous one but instead of adding all the subtours constraints
                from the start, it adds them dynamically. It is usually faster but less stable.
        """

        # Check first if case is not lidar inspection, which won't need any solver. It is just geometry

        if 2 == self.get_Mission_Mode():
            status = solve_Lidar_Case(self)
            return status

        # Depending on the selection, exec one of the solvers
        match which_solver:

            case "abstract_DFJ":
                status = abstract_DFJ_Solver(self)
            case "abstract_dynamic_DFJ":
                status = abstract_Dynamic_DFJ_Solver(self)

            # case "GRASP":    # Aerial-Core heuristic GRASP Method. NOT YET IMPLETED
            #    GRASP_Solver()

            # Default case
            case "":
                status = abstract_Dynamic_DFJ_Solver(self)

            # In case that the given solver name is not found.
            case _:
                raise Exception("No such solver is implemented")
        
        return status

    def link_Progress_Bar(self, ui_progressbar: QProgressBar):
        """
        Connects a PyQt progress bar to the problem. This allows us to update any UI progress within the class
        """
        self.__progressBar = ui_progressbar

        return None

# -------------------------------------- Solvers --------------------------------------------- 

def solve_Lidar_Case(problem: Problem) -> bool:
    """
    This mode only expects one UAV and one lineal branch. If something else
    is given, the solver won't function properly. This case can be solved geometrically 
    without any graph or complex solver and as such, it is done independly
    """

    # Let's get the UAV and its base.
    uav: UAVS.UAV = problem.get_UAV_Team().get_List()[0]
    base = problem.get_Bases().get_Base(uav.missionSettings['Base'])

    # Let's get the closest tower to the UAV base. It only works with the one at the end or the start.
    # Just need to check
    towers = problem.get_Towers()
    towers_coordinates = towers.get_DictCoordinates()
    fTower, lTower = get_First_and_Last_Towers(towers) # This is a bit stupid but it is for future-proofing

    fDist = np.linalg.norm(towers_coordinates[fTower] - base.get_Coordinates())
    lDist = np.linalg.norm(towers_coordinates[lTower] - base.get_Coordinates())
    
    if fDist <= lDist:

        uav.route = [(uav.missionSettings['Base'], fTower)]
        uav.routeModes = ["Navigation"]

        for i in range(1, len(towers_coordinates)):
            uav.route.append(("T"+str(i), "T"+str(i+1)))
            uav.routeModes.append("Inspection")

        for i in reversed(range(1, len(towers_coordinates))):
            uav.route.append(("T"+str(i+1), "T"+str(i)))
            uav.routeModes.append("Inspection")

        uav.route.append((fTower, uav.missionSettings['Base']))
        uav.routeModes.append("Navigation")


    else:

        uav.route = [(uav.missionSettings['Base'], lTower)]
        uav.routeModes = ["Navigation"]

        for i in reversed(range(1, len(towers_coordinates))):
            uav.route.append(("T"+str(i+1), "T"+str(i)))
            uav.routeModes.append("Inspection")

        for i in range(1, len(towers_coordinates)):
            uav.route.append(("T"+str(i), "T"+str(i+1)))
            uav.routeModes.append("Inspection")


        uav.route.append((lTower, uav.missionSettings['Base']))
        uav.routeModes.append("Navigation")

    uav.routeAbstract = uav.route

    print("route", uav.route)
    print("routeAbstract", uav.routeAbstract)
    print("routeModes", uav.routeModes)

    uav.routeCoords = route_to_UTM(uav.route, towers, problem.get_Bases())
    
    print("routeCoords", uav.routeCoords)
    

    return True

def abstract_DFJ_Solver(problem: Problem) -> bool:
    """
    It is the same as the abstract solver excepts that it implements the DFJ Subtour elimination constrains.
    """

    ptowers = problem.get_Towers() # This does not duplicate in memory, just "reference" it.
                                   # This is just default python. Just for faster coding and less referencing
    pbases = problem.get_Bases()
    puavs = problem.get_UAV_Team()
    pweather = problem.get_Weather()
    pgraph = problem.get_Graph()
    pmodel =  problem.get_SCIP_Model() 

    #--------------------------------- Graph ---------------------------------------------------------

    construct_Abstract_Graph(pbases, ptowers, puavs, pweather, pgraph, problem.get_Mission_Mode())

    # With this, the abstract graph is fully constructed.     
        
    # ------------------------------ SCIP ------------------------------------
    # Let's define the SCIP problem, compute the solution and update the UAVs routes
 
    Z, Y, sigmas, t, e = construct_Abstract_SCIP_Model(pbases, ptowers, puavs, pgraph, pmodel, problem.get_Mission_Mode())

    # --------------------- Subroute DFJ Solution ----------------------------
    
    pnodes_B = list(pgraph.nodes)

    for uav in puavs:
        pnodes = [node for node in pnodes_B if (uav.missionSettings["Base"] != node)]

        #print("Subtour Nodes:", pnodes)

        Qlist = itertools.chain.from_iterable(list(itertools.combinations(pnodes, r)) for r in range(2, len(pnodes)+1))

        # For each Q subset, a constrain is added:
        for Q in Qlist:
            #print("Adding constraint Q: ", Q)
            add_DFJ_Subtour_Constraint(Q, Z, uav, pmodel, problem.get_Mission_Mode())

    # --------------------- Subroute DFJ Solution ----------------------------
    

    f_k = 1.0 #0.5 # This parameter requieres finetunning. It is useful not to fix it at 1.0

    pmodel.setObjective(SCIP.quicksum(t[key]*Z[key] for key in Z.keys()) 
                                     + 
                                f_k * SCIP.quicksum(sigmas[key] for key in sigmas.keys()))

    #print(pmodel.getObjective())

    #pmodel.setHeuristics(SCIP_PARAMSETTING.OFF) # Disable HEURISTICS
    #pmodel.disablePropagation()                 # Disable solution Propagation

    # Write Scipi Problem externally into a human-readable file
    # pmodel.writeProblem('scip_model.cip')
            
    pmodel.optimize()
    if "infeasible" == pmodel.getStatus():
        return False
    
    sol = pmodel.getBestSol()

    print("Problem Status is:", pmodel.getStatus())

    #print(problem.scipi_model.checkSol(sol))

    # -------------------- Route parsing -----------------------------

    parse_Abstract_Routes(sol, Z, puavs, problem.get_Mission_Mode())

    for uav_pair in sigmas:
        print('sigma_', uav_pair, ' = ' ,sol[sigmas[uav_pair]])

    for uav in puavs:
        print('NO = ', uav.route)
        uav.route, uav.routeModes = order_Route(uav.missionSettings['Base'], uav.route, uav.routeModes)
        print('O = ', uav.route)
        uav.route, uav.routeModes = fix_Route_Valid_Subloops(uav.route, uav.routeModes)
        print('OF = ', uav.route)
        
        uav.routeCoords = route_to_UTM(uav.route, ptowers, pbases)
        
        print(uav.routeModes)

    return True

def abstract_Dynamic_DFJ_Solver(problem: Problem) -> bool:
    """
    It is the same as the abstract solver excepts that it implements a dynamic DFJ Subtour elimination constraint.
    """

    ptowers = problem.get_Towers() # This does not duplicate in memory, just "reference" it.
                                   # This is just default python. Just for faster coding and less referencing
    pbases = problem.get_Bases()
    puavs = problem.get_UAV_Team()
    pweather = problem.get_Weather()
    pgraph = problem.get_Graph()
    pmodel =  problem.get_SCIP_Model() 

    #--------------------------------- Graph ---------------------------------------------------------

    construct_Abstract_Graph(pbases, ptowers, puavs, pweather, pgraph, problem.get_Mission_Mode())

    # With this, the abstract graph is fully constructed.     
        
    # ------------------------------ SCIP ------------------------------------
    # Let's define the SCIP problem, compute the solution and update the UAVs routes

    Z, Y, sigmas, t, e = construct_Abstract_SCIP_Model(pbases, ptowers, puavs, pgraph, pmodel, problem.get_Mission_Mode())

    tgraph = ptowers.get_Graph()
    
    # If the towers and not entirely connected, then add some subtour constrains before the first iteration
    if not nx.is_connected(tgraph):

        # Compute a list with the list of towers of each disjoint component
        SC = [tgraph.subgraph(c).copy() for c in nx.connected_components(tgraph)]

        # Use case 0
        if 0 == problem.get_Mission_Mode():

            subsets = []
            # For each disjoint component
            for subcomponent in SC:
                
                # For each connection of towers, add SUP and SDOWN to the subset of nodes
                # of the corresponding disjoint component
                subset = []
                for line_segment in subcomponent.edges():
                    edge_str = line_segment[0]+','+line_segment[1]
                    if 'SUP_{'+edge_str+'}' in pgraph.nodes():
                        subset.append('SUP_{'+edge_str+'}')
                        subset.append('SDOWN_{'+edge_str+'}')
                    else:
                        subset.append('SUP_{'+line_segment[1]+','+line_segment[0]+'}')
                        subset.append('SDOWN_{'+line_segment[1]+','+line_segment[0]+'}')


                subsets.append(subset)

        # Use case 1
        elif 1 == problem.get_Mission_Mode():

            subsets = []
            for subcomponent in SC:
                # Each of the subsets contains all towers from each subcomponent
                subsets.append(list(subcomponent.nodes()))
        

        for W in subsets:
            
            # To keep thing fast, lets limit the max length of each subset.
            if len(W) < 10: # This might be adjusted by the user.
                
                # We need to compute all possible subsets of each Q
                Qlist = itertools.chain.from_iterable(itertools.combinations(W, r) for r in range(2, len(W)+1))

                # Add the corresponding constraint.
                for Q in Qlist:
                    for uav in puavs:
                        add_DFJ_Subtour_Constraint(Q, Z, uav, pmodel, problem.get_Mission_Mode())



    f_k = 1.0 #0.5 # This parameter requieres finetunning. It is useful not to fix it at 1.0

    pmodel.setObjective(SCIP.quicksum(t[key]*Z[key] for key in Z.keys()) 
                                     + 
                                f_k * SCIP.quicksum(sigmas[key] for key in sigmas.keys()))

    #print(pmodel.getObjective())

    #pmodel.setHeuristics(SCIP_PARAMSETTING.OFF) # Disable HEURISTICS
    #pmodel.disablePropagation()                 # Disable solution Propagation

    pmodel.hideOutput()

    print("")
    print("Initial iter")
    print("----------------------------------------")

    pmodel.optimize()
    if "infeasible" == pmodel.getStatus():
        return False

    sol = pmodel.getBestSol()
    #print(problem.scipi_model.checkSol(sol))

    # -------------------- Route parsing -----------------------------

    parse_Abstract_Routes(sol, Z, puavs, problem.get_Mission_Mode())

    for uav in puavs:
        print("UAV:", uav.get_ID())
        print("Route: ", uav.routeAbstract)
    
    k = 1
    print("")
    print("Dynamic DFJ Subtour elimination iter: ", k)
    print("----------------------------------------")

   
    # For each UAV, let's check if there are loops and delete them
    subtoursQ = False
    for uav in puavs:

        print("UAV:", uav.get_ID())

        loopsp, _ = list_Loops(uav.routeAbstract, [])

        print("Route: ", uav.routeAbstract)
        print("Loops: ", loopsp)

        loops = delete_Loops_with_Base(loopsp, uav.missionSettings["Base"])

        # if there are more than one loop or if the only loop does not contain the actual UAV base
        if loops:
        # if len(loops) > 1 or (1 == len(loops) and not does_Contain_Node(uav.missionSettings["Base"], loops[0])):

             # Let's free the problem so we can modify it
            if not subtoursQ: pmodel.freeReoptSolve()
            #if not subtoursQ: pmodel.freeTransform()
    
            Q = get_Q_from_loops(loops)
            print("Q: ", Q)

            for Q_s in get_Subsets_from_Q(Q, problem.get_Mission_Mode()):
                add_DFJ_Subtour_Constraint(Q_s, Z, uav, pmodel, problem.get_Mission_Mode())

            subtoursQ = True
                
    # If any new constraint is added, compute new solution
    if subtoursQ: 
        pmodel.writeProblem('scip_model_DFJ.cip')
        pmodel.optimize()
        if "infeasible" == pmodel.getStatus():
            return False
            
        sol = pmodel.getBestSol()

        parse_Abstract_Routes(sol, Z, puavs, problem.get_Mission_Mode())

    
    k += 1
    # Check for subroutes and find new solution until there are not
    # Assume that there are subroutes still
    while subtoursQ:

        print("")
        print("Dynamic DFJ Subtour elimination iter: ", k)
        print("----------------------------------------")

        subtoursQ = False
        for uav in puavs:

            print("UAV:", uav.get_ID())

            print("Route: ", uav.routeAbstract)

            loopsp, _ = list_Loops(uav.routeAbstract, [])
            print("Loops: ", loopsp)

            loops = delete_Loops_with_Base(loopsp, uav.missionSettings["Base"])

            # if len(loops) > 1 or (1 == len(loops) and not does_Contain_Node(uav.missionSettings["Base"], loops[0])):
            if loops:    
                if not subtoursQ: pmodel.freeReoptSolve()
                #if not subtoursQ: pmodel.freeTransform()

                Q = get_Q_from_loops(loops)
                print("Q: ", Q)

                for Q_s in get_Subsets_from_Q(Q, problem.get_Mission_Mode()):
                    add_DFJ_Subtour_Constraint(Q_s, Z, uav, pmodel, problem.get_Mission_Mode())

                subtoursQ = True

        if subtoursQ:
            pmodel.writeProblem('scip_model_DFJ.cip')
            pmodel.optimize()
            if "infeasible" == pmodel.getStatus():
                return False
            sol = pmodel.getBestSol()

            parse_Abstract_Routes(sol, Z, puavs, problem.get_Mission_Mode())

        k += 1

    # Write Scipi Problem externally into a human-readable file
    pmodel.writeProblem('scip_model_DFJ.cip')

    for uav_pair in sigmas:
        print('sigma_', uav_pair, ' = ' ,sol[sigmas[uav_pair]])

    for uav in puavs:

        loops, _ = list_Loops(uav.routeAbstract, [])

        print("Loops List: ", loops)

        print('NO = ', uav.route)
        uav.route, uav.routeModes = order_Route(uav.missionSettings['Base'], uav.route, uav.routeModes)
        print('O = ', uav.route)
        uav.route, uav.routeModes = fix_Route_Valid_Subloops(uav.route, uav.routeModes)
        print('OF = ', uav.route)

        uav.routeCoords = route_to_UTM(uav.route, ptowers, pbases)
        
        print(uav.routeModes)

    return True

def GRASP_Solver():
    """
    Using the abstractSolver, let's implement the GRASP method to find a solution. To be implement
    """
    # TBD
    return None

# -------------------------------------- Auxiliary Functions ---------------------------------------------

def get_First_and_Last_Towers(towers: TW.Towers) -> tuple[str, str]:
    """
    Outputs the first and last added towers to the towers class.
    """

    # The code is such that towers name already gives them the correct order (?)
    first_tower = "T1"
    last_tower = "T"+str(len(towers.get_Towers()))

    return first_tower, last_tower

def delete_Loops_with_Base(loops: list, base: str) -> list:

    loops_no_base = []

    for loop in loops:
        if not does_Contain_Node(base, loop): loops_no_base.append(loop)

    return loops_no_base

def order_Route(start: str, edge_list: list, mode_list:list) -> tuple[list, list]:
    """
    Given and unordered route, it orders following the nodes from the starting base.
    It suposses that routes are closed and if it has any subroutes or ambiguity it will
    take a choice based on ordering and order each subpath.

    Here, the route considered is physical and not the abstract one.
    """

    n_e = len(edge_list)

    if 0 == n_e:
        return [(start, start)], [("Navigation")]

    orderedRoute = []
    orderedModes = []

    # Search for the exit base edge
    foundQ = False
    for k in range(n_e):
        edge_l = edge_list[k].split('->')
        if edge_l[0] == start:
            foundQ = True
            break
    
    # If no base is found, start the ordering from the first node on the route
    if not(foundQ):
        print('orderRoute::Start node is not in the route')
    orderedRoute.append((edge_l[0], edge_l[1]))
    orderedModes.append(mode_list[k])
    # Delete edge from list
    del edge_list[k]
    del mode_list[k]

    # From there, continue connecting consecutive edges
    while not(len(orderedRoute) == n_e):
        foundQ = False
        for k in range(len(edge_list)):
            edge_l = edge_list[k].split('->')
            if edge_l[0] == orderedRoute[-1][1]:
                foundQ = True
                break

        # If the next node is found, add it to the ordered list        
        if foundQ:
            orderedRoute.append((edge_l[0], edge_l[1]))
            orderedModes.append(mode_list[k])
            # Delete edge from list
            del edge_list[k]
            del mode_list[k]
        # If it is not found and there are nodes left, them the route
        # has subroutes. Order the subroute and continue
        else:
            print('orderRoute::Route has two or more disjoint subpaths')
            temp, temp_m = order_Route(edge_list[0].split('->')[0], edge_list, mode_list)
            orderedRoute += temp
            orderedModes += temp_m
    
    # The ordering stops when the ordered route has the same number of nodes as
    # the initial route
    return orderedRoute, orderedModes

def fix_Route_Valid_Subloops(ordered_route: list, ordered_modes:list):
    """
    If a route has subloops but are joint by exactly one node, them the route is valid and should be ordered correctly.
    This function takes an ordered route and fixes the ordering so the UAV covers the entire route before
    getting back to the base loop. By now, it supports just two loops.
    """

    fixed_route = ordered_route
    fixed_modes = ordered_modes

    loops, loops_modes = list_Loops(fixed_route, fixed_modes)

    while len(loops) > 1:

        # Find the loop with the base
        k = 0
        for loop in loops:
            for edge in loop:
                if "B" == edge[0][0]: 
                    first_loop = loop
                    rest_loops = loops[:k] + loops[(k + 1):]
        
            k += 1

        # Find the common node of the first loop and the next
        for loop in rest_loops:
            foundQ, node, f_k, loop_k = find_common_node(first_loop, loop)

            if foundQ: 
                fixed_route = first_loop[:f_k+1]+loop[loop_k:]+loop[:loop_k]+first_loop[(f_k+1):]
                break

        loops, loops_modes = list_Loops(fixed_route, fixed_modes)
    
    return fixed_route, fixed_modes

def find_common_node(route1: list, route2: list) -> tuple[bool, str, int, int]:

    k1 = 0 
    for edge1 in route1:
        k2 = 0
        for edge2 in route2:
            if edge1[1] == edge2[0]: return True, edge1[1], k1, k2
            k2 += 1
        
        k1 +=1
    
    return False, "",  float('NaN'), float('NaN')

def route_to_UTM(route: list, towers: TW.Towers, bases:BA.Bases):
    """
    Transforms each node on each edge of the route tu UTM coordinates.
    Return a list with a 2-tuple (UTM1, UTM2) per original edge
    """
    routeCoords = []
    for edge in route:
        
        if edge[0][0] == 'B' and edge[1][0] == 'T':
            UTM1 = bases.get_Base(edge[0]).get_Coordinates()
            UTM2 = towers.get_Tower_Coordinates(edge[1])
        
        elif edge[0][0] == 'T' and edge[1][0] == 'B':
            UTM1 = towers.get_Tower_Coordinates(edge[0])
            UTM2 = bases.get_Base(edge[1]).get_Coordinates()

        elif edge[0][0] == 'B' and edge[1][0] == 'B':
            UTM1 = bases.get_Base(edge[0]).get_Coordinates()
            UTM2 = bases.get_Base(edge[1]).get_Coordinates()
        
        else:
            UTM1 = towers.get_Tower_Coordinates(edge[0])
            UTM2 = towers.get_Tower_Coordinates(edge[1])
        
        routeCoords.append((UTM1, UTM2))

    return routeCoords

def construct_Abstract_Graph(pbases: BA.Bases, ptowers: TW.Towers, puavs: UAVS.UAV_Team,
                             pweather: WT.Weather, pgraph: nx.MultiDiGraph, mode:int):
    
    """
    Auxiliary function for the abstract series of solvers. Given a problem, compute
    the abstract graph. It edits pgraph in place.
    """

    match mode:

        # Segment Inspection
        case 0:
    
            # Add all S (Sup and Sdown) nodes
            # Iterate every pair of connected towers to create the segment nodes
            for line_segment in ptowers.get_Graph().edges():

                # Inspection in one direction. S^UPARROW
                pgraph.add_node(
                    'SUP_{'+line_segment[0]+','+line_segment[1]+'}'
                )
        
                # The other. S^DOWNARROW
                pgraph.add_node(
                    'SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'
                )
    
            # Prefetch all tower coordinates
            coords = nx.get_node_attributes(ptowers.get_Graph(), 'UTM')

            # Add connection between S with itself
            # Iterate over all possible pairs of different line power segments
            for line_segment_pair in list(itertools.combinations(ptowers.get_Graph().edges(), 2)):

                # Parses the 2 towers pairs into nodes
                t1 = line_segment_pair[0][0]
                t2 = line_segment_pair[0][1]
                t3 = line_segment_pair[1][0]
                t4 = line_segment_pair[1][1]

                s1_tag = t1+','+t2
                s2_tag = t3+','+t4

                node1 = [(t1, coords[t1]), (t2, coords[t2])]
                node2 = [(t3, coords[t3]), (t4, coords[t4])]

                # Compute weights for each added edge

                # One edge direction 
                # Sup and Sup connection
                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1,
                                            node2,
                                            puavs,
                                            pweather, mode)

        
                pgraph.add_edge(
                    'SUP_{'+s1_tag+'}',
                    'SUP_{'+s2_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                # Sup and Sdown connection
                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1,
                                            node2[::-1],
                                            puavs,
                                            pweather, mode)


                pgraph.add_edge(
                    'SUP_{'+s1_tag+'}',
                    'SDOWN_{'+s2_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                # Sdown and Sdown connection
                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1[::-1],
                                            node2[::-1],
                                            puavs,
                                            pweather, mode)

                pgraph.add_edge(
                    'SDOWN_{'+s1_tag+'}',
                    'SDOWN_{'+s2_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                # Sdown and Sdown connection
                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1[::-1],
                                            node2,
                                            puavs,
                                            pweather, mode)

                pgraph.add_edge(
                    'SDOWN_{'+s1_tag+'}',
                    'SUP_{'+s2_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                # Now the same but with the edge in the other direction
                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2,
                                            node1,
                                            puavs,
                                            pweather, mode)

                pgraph.add_edge(
                    'SUP_{'+s2_tag+'}',
                    'SUP_{'+s1_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2,
                                            node1[::-1],
                                            puavs,
                                            pweather, mode)


                pgraph.add_edge(
                   'SUP_{'+s2_tag+'}',
                    'SDOWN_{'+s1_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2[::-1],
                                            node1[::-1],
                                            puavs,
                                            pweather, mode)

                pgraph.add_edge(
                    'SDOWN_{'+s2_tag+'}',
                    'SDOWN_{'+s1_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2[::-1],
                                            node1,
                                            puavs,
                                            pweather, mode)

                pgraph.add_edge(
                    'SDOWN_{'+s2_tag+'}',
                    'SUP_{'+s1_tag+'}',
                    type = 'TT',
                    Wt = W_t,
                    We = W_e 
                )

            # Add base nodes and connect them with all S nodes
            for base in pbases:
                pgraph.add_node(
                    base.get_Name()
                )
        
                for line_segment in ptowers.get_Graph().edges():

                    t1 = line_segment[0]
                    t2 = line_segment[1]
                    s_tag = t1+','+t2

                    node1 = (base.get_Name(), base.get_Coordinates())
                    node2 = [(t1, coords[t1]), (t2, coords[t2])]

                    W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1,
                                            node2,
                                            puavs,
                                            pweather, mode)

                    pgraph.add_edge(
                        base.get_Name(),
                        'SUP_{'+s_tag+'}',
                        type = 'BT',
                        Wt = W_t,
                        We = W_e
                    )

                    W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2,
                                            node1,
                                            puavs,
                                            pweather, mode)

                    pgraph.add_edge(
                        'SUP_{'+s_tag+'}',
                        base.get_Name(),
                        type = 'TB',
                        Wt = W_t,
                        We = W_e
                    )

                    W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1,
                                            node2[::-1],
                                            puavs,
                                            pweather, mode)

                    pgraph.add_edge(
                        base.get_Name(),
                        'SDOWN_{'+s_tag+'}',
                        type = 'BT',
                        Wt = W_t,
                        We = W_e
                    )

                    W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2[::-1],
                                            node1,
                                            puavs,
                                            pweather, mode)

                    pgraph.add_edge(
                       'SDOWN_{'+s_tag+'}',
                        base.get_Name(),
                        type = 'TB',
                        Wt = W_t,
                        We = W_e
                    )

        # Point Inspection
        case 1:
            
            # Prefetch all tower coordinates
            coords = nx.get_node_attributes(ptowers.get_Graph(), 'UTM')
            
            edges = list(itertools.combinations(coords.keys(), 2))

            # Set all towers to the problem graph. No Power Line segment connection are needed.
            pgraph.add_nodes_from(coords)
            
            for edge in edges:

                node1 = (edge[0], coords[edge[0]])
                node2 = (edge[1], coords[edge[1]])

                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1,
                                            node2,
                                            puavs,
                                            pweather, mode)

                pgraph.add_edge(edge[0], edge[1],
                    Wt = W_t,
                    We = W_e 
                    )
                
                W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2,
                                            node1,
                                            puavs,
                                            pweather, mode)


                pgraph.add_edge(edge[1], edge[0],
                    Wt = W_t,
                    We = W_e 
                    )
                    
                
            for base in pbases:

                pgraph.add_node(
                    base.get_Name(),
                    UTM = base.get_Coordinates()
                )

                for tower in ptowers.get_Graph().nodes():

                    node1 = (base.get_Name(), base.get_Coordinates())
                    node2 = (tower, coords[tower])

                    W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node1,
                                            node2,
                                            puavs,
                                            pweather, mode)

                    pgraph.add_edge(base.get_Name(), tower,
                        Wt = W_t,
                        We = W_e 
                        )
                
                    W_t, W_e = WE.compute_Abstract_Edge_Weights(
                                            node2,
                                            node1,
                                            puavs,
                                            pweather, mode)


                    pgraph.add_edge(tower, base.get_Name(),
                        Wt = W_t,
                        We = W_e 
                        )
    
    return None

def parse_Abstract_Routes(sol:dict, Z:dict, puavs: UAVS.UAV_Team, mode: int):

    for uav in puavs:
        uav.route = []
        uav.routeAbstract = []
    
    for edge in Z:

        if np.abs(sol[Z[edge]] - 1.0) < 1e-6: # To avoid floating point errors
            
            
            temp = edge.split('|')

            # puavs.selectUAV(temp[1]).route.append(temp[0])

            nodes = temp[0].split('->')

            match mode:
                case 0:

                    puavs.select_UAV(temp[1]).routeAbstract.append((nodes[0], nodes[1]))

            
                    #print(nodes, temp[1])

                    # B to S
                    if nodes[0][0] == 'B':

                        #print('B to S')
                
                        mode_segment = nodes[1].split('_')
                        towers = mode_segment[1][1:-1].split(',')

                        # Add edges to UAV Route. Later, it will be ordered.
                        if mode_segment[0] == 'SUP':
                            # Base to Tower
                            puavs.select_UAV(temp[1]).route.append(nodes[0]+'->'+towers[0])
                            puavs.select_UAV(temp[1]).routeModes.append('Navigation')
                            # Inspection SUP
                            puavs.select_UAV(temp[1]).route.append(towers[0]+'->'+towers[1])
                            puavs.select_UAV(temp[1]).routeModes.append('Inspection')
                        else:
                            # Base to Tower
                            puavs.select_UAV(temp[1]).route.append(nodes[0]+'->'+towers[1])
                            puavs.select_UAV(temp[1]).routeModes.append('Navigation')
                            # Inspection SDOWN
                            puavs.select_UAV(temp[1]).route.append(towers[1]+'->'+towers[0])
                            puavs.select_UAV(temp[1]).routeModes.append('Inspection')

                    # S to B
                    elif nodes[1][0] == 'B':

                        #print('S to B')

                        mode_segment = nodes[0].split('_')
                        towers = mode_segment[1][1:-1].split(',')

                        # Add edges to UAV Route. Later, it will be ordered.
                        if mode_segment[0] == 'SUP':
                            # Inspection SUP
                            # problem.uav_team.selectUAV(temp[1]).route.append(towers[0]+'->'+towers[1])
                            # problem.uav_team.selectUAV(temp[1]).routeModes.append('Inspection')
                            # Base to Tower
                            puavs.select_UAV(temp[1]).route.append(towers[1]+'->'+nodes[1])
                            puavs.select_UAV(temp[1]).routeModes.append('Navigation')
                        else:
                            # Inspection SDOWN
                            # problem.uav_team.selectUAV(temp[1]).route.append(towers[1]+'->'+towers[0])
                            # problem.uav_team.selectUAV(temp[1]).routeModes.append('Inspection')
                            # Base to Tower
                            puavs.select_UAV(temp[1]).route.append(towers[0]+'->'+nodes[1])
                            puavs.select_UAV(temp[1]).routeModes.append('Navigation')

                    # S to S
                    else:

                        #print('S to S')

                        mode_segment1 = nodes[0].split('_')
                        towers1 = mode_segment1[1][1:-1].split(',')

                        mode_segment2 = nodes[1].split('_')
                        towers2 = mode_segment2[1][1:-1].split(',')

                        if mode_segment1[0] == 'SUP' and mode_segment2[0] == 'SUP':

                            # Inspection SUP 1
                            # problem.uav_team.selectUAV(temp[1]).route.append(towers1[0]+'->'+towers1[1])
                            # problem.uav_team.selectUAV(temp[1]).routeModes.append('Inspection')
                    
                            # Tower to Tower movement
                            if towers1[1] != towers2[0]:
                                puavs.select_UAV(temp[1]).route.append(towers1[1]+'->'+towers2[0])
                                puavs.select_UAV(temp[1]).routeModes.append('Navigation')
                            # Inspection SUP 2
                            puavs.select_UAV(temp[1]).route.append(towers2[0]+'->'+towers2[1])
                            puavs.select_UAV(temp[1]).routeModes.append('Inspection')

                        elif mode_segment1[0] == 'SUP' and mode_segment2[0] == 'SDOWN':

                            # Inspection SUP 1
                            # problem.uav_team.selectUAV(temp[1]).route.append(towers1[0]+'->'+towers1[1])
                            # problem.uav_team.selectUAV(temp[1]).routeModes.append('Inspection')

                            # Tower to Tower movement
                            if towers1[1] != towers2[1]:
                                puavs.select_UAV(temp[1]).route.append(towers1[1]+'->'+towers2[1])
                                puavs.select_UAV(temp[1]).routeModes.append('Navigation')
                            # Inspection SDOWN 2
                            puavs.select_UAV(temp[1]).route.append(towers2[1]+'->'+towers2[0])
                            puavs.select_UAV(temp[1]).routeModes.append('Inspection')

                        elif mode_segment1[0] == 'SDOWN' and mode_segment2[0] == 'SUP':
                    
                            # Inspection SDOWN 1
                            # problem.uav_team.selectUAV(temp[1]).route.append(towers1[1]+'->'+towers1[0])
                            # problem.uav_team.selectUAV(temp[1]).routeModes.append('Inspection')

                            # Tower to Tower movement
                            if towers1[0] != towers2[0]:
                                puavs.select_UAV(temp[1]).route.append(towers1[0]+'->'+towers2[0])
                                puavs.select_UAV(temp[1]).routeModes.append('Navigation')

                            # Inspection SUP 2
                            puavs.select_UAV(temp[1]).route.append(towers2[0]+'->'+towers2[1])
                            puavs.select_UAV(temp[1]).routeModes.append('Inspection')

                        else:

                            # Inspection SDOWN 1
                            # problem.uav_team.selectUAV(temp[1]).route.append(towers1[1]+'->'+towers1[0])
                            # problem.uav_team.selectUAV(temp[1]).routeModes.append('Inspection')

                            # Tower to Tower movement
                            if towers1[0] != towers2[1]:
                                puavs.select_UAV(temp[1]).route.append(towers1[0]+'->'+towers2[1])
                                puavs.select_UAV(temp[1]).routeModes.append('Navigation')
                            # Inspection SDOWN 2
                            puavs.select_UAV(temp[1]).route.append(towers2[1]+'->'+towers2[0])
                            puavs.select_UAV(temp[1]).routeModes.append('Inspection')
            
                    #print('t = ', puavs.get_List()[0].route)
                            
                case 1:

                    puavs.select_UAV(temp[1]).routeAbstract.append((nodes[0], nodes[1]))
                    puavs.select_UAV(temp[1]).route.append(nodes[0]+'->'+nodes[1])
                    puavs.select_UAV(temp[1]).routeModes.append('Navigation')

def construct_Abstract_SCIP_Model(pbases: BA.Bases, ptowers: TW.Towers, puavs: UAVS.UAV_Team,
                                   pgraph: nx.MultiDiGraph, pmodel: SCIP.Model, mode: int) -> tuple[dict, dict, dict, dict, dict]:
    
    # Dictionaries to store SCIP variables
    Z = {}
    sigmas = {}
    Y = {}

    # Dictionaries to store weights for easy access
    t = {}
    e = {}

    Wt = nx.get_edge_attributes(pgraph, 'Wt')
    We = nx.get_edge_attributes(pgraph, 'We')

    

    tgraph = ptowers.get_Graph()

    distance_threshold = 1000 # in meters
    # This detects if UAVs are too far to any of the groups of towers and allows deactivating them.
    SC = [tgraph.subgraph(c).copy() for c in nx.connected_components(tgraph)]
    distant_subgraphsQ = any([np.linalg.norm(list(dict(subset.nodes(data="UTM")).values())[0] - base.get_Coordinates()) > distance_threshold
                                  for subset in SC for base in pbases])
    
    # distant_subgraphsQ = False   # This is a patch. I need to fix some other things

    if distant_subgraphsQ: print("Subgroup of towers might be too far from some of the UAVs bases. Allowing automatic disabling")
        
    match mode:
         
        case 0:

            tooManyUAVSQ = len(puavs.get_List()) > len(list(ptowers.get_Graph().edges()))
            if tooManyUAVSQ: print("There are less inspection nodes than UAVs. Enabling automatic disabling of UAVs")

            # For each edge in the graph assign a Z variable and assign the corresponding weights
            # to its key in the weights dictionaries
            for edge in pgraph.edges():

                for uav in puavs:

                    edge_uav = edge[0]+'->'+edge[1]+'|'+uav.get_ID() # Edge|UAV Tag

                    Z[edge_uav] = pmodel.addVar(vtype = 'B', obj = 0.0, name = uav.get_ID()+"Z"+edge[0]+edge[1])
            
                    t[edge_uav] = Wt[(edge[0], edge[1], 0)][uav.get_ID()]
                    e[edge_uav] = We[(edge[0], edge[1], 0)][uav.get_ID()]


            # Only one inspection constrain
            for line_segment in ptowers.get_Graph().edges():

                nodelist = copy.deepcopy(list(pgraph.nodes()))
                nodelist.remove('SUP_{'+line_segment[0]+','+line_segment[1]+'}')
                nodelist.remove('SDOWN_{'+line_segment[0]+','+line_segment[1]+'}')

                pmodel.addCons(
                    SCIP.quicksum(
                        SCIP.quicksum(Z[node+'->'+  'SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                            + Z[node+'->'+'SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                        for uav in puavs)
                    for node in nodelist)
                        == 
                    1.0
                )

                pmodel.addCons(
                    SCIP.quicksum(
                        SCIP.quicksum(Z[  'SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                            + Z['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                        for uav in puavs)
                    for node in nodelist)
                       == 
                    1.0
                )

                # Continuity. The UAV that inspects must be the one that exits the segment
                for uav in puavs:
                    
                    # Disabled since they do nothing. Subtour elimination and the disabling of the base already do this job
                    if False: #tooManyUAVSQ or distant_subgraphsQ:

                        Y[uav.get_ID()] = pmodel.addVar(vtype = 'B', obj = 0.0, name = "Y"+uav.get_ID())

                        pmodel.addCons(
                            SCIP.quicksum(Z[node+'->'+  'SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                                    + Z[node+'->'+'SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                            for node in nodelist)
                                <= 
                            Y[uav.get_ID()]
                        )

                        pmodel.addCons(
                            SCIP.quicksum(Z['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                                + Z['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                            for node in nodelist)
                                <= 
                            Y[uav.get_ID()]
                        )

                    Y['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B', name = uav.get_ID()+"Y"+line_segment[0]+line_segment[1])

                    pmodel.addCons(
                        SCIP.quicksum(Z[node+'->'+'SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                            + Z['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                        for node in nodelist)
                            == 
                        2 * Y['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                    )

                    Y['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B', name = uav.get_ID()+"Y"+line_segment[0]+line_segment[1])

                    pmodel.addCons(
                        SCIP.quicksum(Z[node+'->'+'SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                            + Z['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                        for node in nodelist)
                            == 
                        2 * Y['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                    )

    
            for uav in puavs:

                nodelist = copy.deepcopy(list(pgraph.nodes()))

                for base in pbases:
                    nodelist.remove(base.get_Name())

                if tooManyUAVSQ or distant_subgraphsQ:

                    Y[uav.get_ID()] = pmodel.addVar(vtype = 'B', obj = 0.0, name = "Y"+uav.get_ID())

                    # Base exit constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                               Z[uav.missionSettings['Base']+'->'+node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            Y[uav.get_ID()]
                    )

                    # Base entering constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                                Z[node+'->'+uav.missionSettings['Base']+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            Y[uav.get_ID()]
                    )

                else:
                    pmodel.addCons(
                            SCIP.quicksum(
                               Z[uav.missionSettings['Base']+'->'+node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            1
                    )

                    # Base entering constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                                Z[node+'->'+uav.missionSettings['Base']+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            1
                    )

                for base in pbases:
                    
                    if base.get_Name() != uav.missionSettings["Base"]:

                        
                        # Not assigned base entering prohibition
                        pmodel.addCons(
                            SCIP.quicksum(
                                Z[node+'->'+ base.get_Name() +'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            0.0
                        )

                        # Not assigned base exit prohibition
                        pmodel.addCons(
                            SCIP.quicksum(
                                Z[base.get_Name() +'->'+ node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            0.0
                        )

                # Energy constrain
                pmodel.addCons(
                    SCIP.quicksum(e[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] 
                            for edge in pgraph.edges()) 
                        <= 
                    1.0
                )


            # The N(N-1)/2 UAVs pairs. No repetitions and no order.
            uav_pairs = list(itertools.combinations(puavs, 2))

            # Create the sigma variables on SCIP. Also add the sigma definition constrains
            for pair in uav_pairs:
                    pair_tag = pair[0].get_ID()+','+pair[1].get_ID()
                    sigmas[pair_tag] = pmodel.addVar(vtype = 'C', name = "Sigma"+pair[0].get_ID()+pair[1].get_ID())

                    # Sigma constrains
                    # Linearized version. It is way less problematic
            
                    pmodel.addCons(
                        SCIP.quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] 
                               - t[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] 
                                    for edge in pgraph.edges())
                        - sigmas[pair_tag] 
                                    <= 
                        0.0
                    )

                    pmodel.addCons(
                        SCIP.quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] 
                            - t[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] 
                                    for edge in pgraph.edges())
                        - sigmas[pair_tag] 
                                    <= 
                        0.0
                    )
        
        case 1:

            tooManyUAVSQ = len(puavs.get_List()) > len(list(ptowers.get_Graph().nodes))
            if tooManyUAVSQ: print("There are less inspection nodes than UAVs. Enabling automatic disabling of UAVs")
            
            # For each edge in the graph assign a Z variable and assign the corresponding weights
            # to its key in the weights dictionaries
            for edge in pgraph.edges():

                for uav in puavs:

                    edge_uav = edge[0]+'->'+edge[1]+'|'+uav.get_ID() # Edge | UAV Tag

                    Z[edge_uav] = pmodel.addVar(vtype = 'B', obj = 0.0, name = uav.get_ID()+"Z"+edge[0]+edge[1])
            
                    t[edge_uav] = Wt[(edge[0], edge[1], 0)][uav.get_ID()]
                    e[edge_uav] = We[(edge[0], edge[1], 0)][uav.get_ID()]

            # Only one inspection per tower and continutiy. The UAV that inspects must be the one that exits the segment
            for tower in ptowers.get_Graph().nodes():

                nodelist = copy.deepcopy(list(pgraph.nodes()))
                nodelist.remove(tower)

                pmodel.addCons(
                    SCIP.quicksum(
                        SCIP.quicksum(Z[node+'->'+tower+'|'+uav.get_ID()] for uav in puavs)
                    for node in nodelist)
                        == 
                    1.0
                )

                pmodel.addCons(
                    SCIP.quicksum(
                        SCIP.quicksum(Z[tower+'->'+node+'|'+uav.get_ID()] for uav in puavs)
                    for node in nodelist)
                        == 
                    1.0
                )

                for uav in puavs:
                    
                    if False: #tooManyUAVSQ or distant_subgraphsQ:

                        Y[uav.get_ID()] = pmodel.addVar(vtype = 'B', obj = 0.0, name = "Y"+uav.get_ID())

                        pmodel.addCons(
                        SCIP.quicksum(
                            Z[node+'->'+tower+'|'+uav.get_ID()]
                        for node in nodelist)
                            <= 
                        Y[uav.get_ID()]
                        )

                        pmodel.addCons(
                        SCIP.quicksum(
                            Z[tower+'->'+node+'|'+uav.get_ID()]
                        for node in nodelist)
                            <= 
                        Y[uav.get_ID()]
                        )

                    Y[tower+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B', name = uav.get_ID()+"Y"+tower)

                    pmodel.addCons(
                        SCIP.quicksum(Z[node+'->'+tower+'|'+uav.get_ID()] + Z[tower+'->'+node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            2 * Y[tower+'|'+uav.get_ID()]
                    )
            
            for uav in puavs:

                nodelist = copy.deepcopy(list(pgraph.nodes()))

                for base in pbases:
                    nodelist.remove(base.get_Name())

                if tooManyUAVSQ or distant_subgraphsQ:

                    Y[uav.get_ID()] = pmodel.addVar(vtype = 'B', obj = 0.0, name = "Y"+uav.get_ID())
                    
                    # Base exit constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                                Z[uav.missionSettings['Base']+'->'+node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            Y[uav.get_ID()]
                    )

                    # Base entering constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                                Z[node+'->'+uav.missionSettings['Base']+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            Y[uav.get_ID()]
                    )
                else:
                    # Base exit constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                                Z[uav.missionSettings['Base']+'->'+node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            1
                    )

                    # Base entering constrain
                    pmodel.addCons(
                            SCIP.quicksum(
                                Z[node+'->'+uav.missionSettings['Base']+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            1
                    )

                for base in pbases:
                    
                    if base.get_Name() != uav.missionSettings["Base"]:

                        
                        # Not assigned base entering prohibition
                        pmodel.addCons(
                            SCIP.quicksum(
                                Z[node+'->'+ base.get_Name() +'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            0.0
                        )

                        # Not assigned base exit prohibition
                        pmodel.addCons(
                            SCIP.quicksum(
                                Z[base.get_Name() +'->'+ node+'|'+uav.get_ID()]
                            for node in nodelist)
                                == 
                            0.0
                        )


                # Energy constrain
                pmodel.addCons(
                    SCIP.quicksum(e[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] 
                            for edge in pgraph.edges()) 
                        <= 
                    1.0
                )

            # The N(N-1)/2 UAVs pairs. No repetitions and no order.
            uav_pairs = list(itertools.combinations(puavs, 2))

            # Create the sigma variables on SCIP. Also add the sigma definition constrains
            for pair in uav_pairs:
                    pair_tag = pair[0].get_ID()+','+pair[1].get_ID()
                    sigmas[pair_tag] = pmodel.addVar(vtype = 'C', name = "Sigma"+pair[0].get_ID()+pair[1].get_ID())

                    # Sigma constrains
                    # Linearized version. It is way less problematic
            
                    pmodel.addCons(
                        SCIP.quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] 
                               - t[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] 
                                    for edge in pgraph.edges())
                        - sigmas[pair_tag] 
                                    <= 
                        0.0
                    )

                    pmodel.addCons(
                        SCIP.quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] 
                            - t[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] 
                                    for edge in pgraph.edges())
                        - sigmas[pair_tag] 
                                    <= 
                        0.0
                    )

    return Z, Y, sigmas, t, e

def add_DFJ_Subtour_Constraint(Q: list, Z:dict, uav: UAVS.UAV, pmodel: SCIP.Model, mode: int):
    """
    Adds one of the DFJ subtour elimination constraints for the subset Q. This is used to dynamically add
    subtour constrains if after one unconstrained run, subtours appear.
    """

    if not Q: return None

    # Compute all pair of Q nodes:
    pairs_t = list(itertools.combinations(Q, 2))

    #print("Pairs_t:", pairs_t)

    # Each constrain is added for each UAV
    pairs = []

    match mode:
        case 0:
            for pair in pairs_t:

                if pair[0][0] == "S" and pair[1][0] == "S" and pair[0].split("_")[1] == pair[1].split("_")[1]:
                    continue
                elif pair[0][0] == "B" and pair[1][0] == "B": continue

                pairs.append(pair)

        case 1:
            for pair in pairs_t:
                if pair[0][0] == "B" and pair[1][0] == "B": continue

                pairs.append(pair)

    #print("Pairs:", pairs)

    if pairs:
        pmodel.addCons(
            SCIP.quicksum(
                Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] + Z[edge[1]+'->'+edge[0]+'|'+uav.get_ID()] 
            for edge in pairs)
                <= 
            len(Q) - 1.0
        )

    return None

def find_Loop(abstract_Route: list, route_modes: list) -> tuple[list, list, list, list]:

    modesQ = len(route_modes) > 1

    starting_move = abstract_Route[0]
    if modesQ: starting_mode = route_modes[0]

    loop = [starting_move, ]
    if modesQ: modes_loop = [starting_mode, ]
    else: modes_loop = []

    unlisted_moves = abstract_Route.copy()
    del unlisted_moves[abstract_Route.index(starting_move)]

    if modesQ: unlisted_modes = route_modes.copy()
    else: unlisted_modes = []
    if modesQ: del unlisted_modes[route_modes.index(starting_mode)]

    checklist = unlisted_moves.copy()
    if modesQ: checklist_modes = unlisted_modes.copy()

    foundQ = True

    while foundQ:

        foundQ = False

        for move in checklist:
            if starting_move[1] == move[0]: #The next point of the loop is found

                if modesQ: mode = checklist_modes[checklist.index(move)]

                loop.append(move)
                if modesQ: modes_loop.append(mode)

                starting_move = move
                if modesQ: starting_mode = mode

                del unlisted_moves[checklist.index(move)]
                if modesQ: del unlisted_modes[checklist_modes.index(mode)]

                foundQ = True
                break

        if foundQ:
            checklist = unlisted_moves.copy()
            if modesQ: checklist_modes = unlisted_modes.copy()
    
    return loop, unlisted_moves, modes_loop, unlisted_modes

def list_Loops(abstract_Route: list, route_modes:list) -> tuple[list, list]:

    modesQ = len(route_modes) > 1

    loop_list = []
    modes_list = []

    if not(abstract_Route):
        return [], []

    loop, left, loop_modes, left_modes = find_Loop(abstract_Route, route_modes)

    loop_list.append(loop)
    if modesQ: modes_list.append(loop_modes)

    while left:

        loop, left, loop_modes, left_modes = find_Loop(left, left_modes)

        if not left:
            loop_list.append(loop)
            if modesQ: modes_list.append(loop_modes)

    return loop_list, modes_list

def get_Q_from_loops(loops: list) -> list:

    Q = []
    
    for loop in loops:
        Q.append(loop[0][0])

        for move in loop:
            Q.append(move[1])

        Q = list(dict.fromkeys(Q))

        return Q
        
def get_Subsets_from_Q(Q: list, mode:int) -> list:

    nodes = []

    if 0 == mode:
    
        for node in Q:
            towers = node.split("_")[1]
            nodes.append("SUP_"+towers)
            nodes.append("SDOWN_"+towers)

        nodes = list(dict.fromkeys(nodes))

        Qlist = []

        #subsets = list(itertools.chain.from_iterable(itertools.combinations(nodes, r) for r in range(2, len(nodes) + 1)))# len(nodes) + 1

        subsets = itertools.combinations(nodes, len(Q))

        for subset in subsets: 
            nodes = []
            for node in subset:
                towers = node.split("_")[1]
                nodes.append(towers)
        
            if len(nodes) == len(list(dict.fromkeys(nodes))):
                Qlist.append(list(subset))

        return Qlist
    
    if 1 == mode:
        return itertools.combinations(Q, len(Q))

def does_Contain_Node(node: str, loop: list) -> bool:

    for edge in loop:

        if node == edge[0] or node == edge[1]: return True

    return False