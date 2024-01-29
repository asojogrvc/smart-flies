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
    def __init__(self, towers: TW.Towers, bases: BA.Bases, uavs: UAVS.UAV_Team, weather = WT.Weather):
        """
        This class contains everything needed to define the GTSP-MUAV problem in any of its forms. That is Towers, Bases, UAV Team and weather conditions.
        It can be solved with of the already impleted methods by using .solve()
        """
       
        # Required input data
        self.__towers = towers   
        self.__bases = bases   
        self.__uav_team = uavs
        self.__weather = weather

        # Auxiliary parameters: 
        #   - The graph represents the abstract representation of the towers and bases. It dependes on the used solver method
        #   - The SCIP model is just another abstraction layer to communicate the actual MILP problem to SCIP
        #   - The progressBar is linked to any progress on the UI to then update it from within the class

        self.__graph = nx.MultiDiGraph()
        self.__scip_model = SCIP.Model("GTSP-MUAV")
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
    
    # ------------------------ Auxiliary functions -------------------------
    def set_Progress_Bar_Value(self, value: float):
        self.__progressBar.setValue(value)
        return None
    
    def progress_Bar_LinkedQ(self) -> bool:
        return self.__progressBar != None
    
    def solve(self, which_solver:str):
        """
        Solves an existing problem with any of the implement formulations / methods. As of now:
            - "regular": This formulation uses a direct interpretation of the power line network
                as a graph. It is simpler but limited. Based on Javi's Matlab PLIP
            - "abstract": This formulation creates a astraction layer that assigns a graph to the problem which nodes
                are not the towers but the power line segments that require inspection. It scales faster than
                "regular" with the size of the problem but it is more general and less limited.
        """

        # As I store the solution within UAVs, I might just pass a flag with the problem status to avoid
        # redudant computation

        # Depending on the selection, exec one of the solvers
        match which_solver:
            case "regular":  # Javi's
                regular_Solver(self)
            case "abstract": # Alvaro's
                abstract_Solver(self)
            case "GRASP":    # Aerial-Core heuristic GRASP Method. NOT YET IMPLETED
                GRASP_Solver()

            # In case that the given solver name is not found.
            case _:
                raise Exception("No such solver is implemented")
        
        return None

    def link_Progress_Bar(self, ui_progressbar: QProgressBar):
        """
        Connects a PyQt progress bar to the problem. This allows us to update any UI progress within the class
        """
        self.__progressBar = ui_progressbar

        return None

def regular_Solver(problem: Problem):
    """
    Solves the given GTSP-MUAV problem using a simple formulation. This formulation uses a direct interpretation of the power line network
    as a graph. It is simpler but limited. Based on Javi's Matlab PLIP.
    """

    # Maybe, to avoid using explicit ordering of stuff, you could pass data from networkx directly to SCIPI

    # --------------------------------- Graph ---------------------------------------------------------

    # First we need to construct the multi-directed-graph with all weights and attributes.
    # Instead of creating one copy of each edge for each UAV, lets just embed all UAVs Weights
    # into the attributes of the edges as dict. This allows some degrees of readbility of the abstract graph.

    
    ptowers = problem.get_Towers() # This does not duplicate in memory, just "reference" it.
                                   # This is just default python. Just for faster coding and less referencing
    pbases = problem.get_Bases()
    puavs = problem.get_UAV_Team()
    pweather = problem.get_Weather()
    pgraph = problem.get_Graph()
    pmodel =  problem.get_SCIP_Model() 

    
    pgraph.add_nodes_from(ptowers.get_Towers())
    nx.set_node_attributes(pgraph, 'Tower', 'type')

    tower_nodes = copy.deepcopy(pgraph.nodes())

    # Iterate every pair of towers
    for pair in list(itertools.combinations(tower_nodes, 2)):

        # If the edge is on the original tower graph, is a power Line segment
        if ptowers.get_Graph().has_edge(pair[0], pair[1]):
            type1 = 'Sup'
            type2 = 'Sdown'
        # If it is not, them it is not a power line segment
        else:
            type1 = 'TT'
            type2 = 'TT'

        # One direction
        W_t, W_e = WE.compute_Edge_Weights((pair[0], pgraph._node[pair[0]]),
                           (pair[1], pgraph._node[pair[1]]),
                            puavs, pweather)

        pgraph.add_edge(pair[0], pair[1], type = type1, Wt = W_t, We = W_e)  

        # The other direction
        W_t, W_e = WE.compute_Edge_Weights((pair[0], pgraph._node[pair[0]]),
                           (pair[1], pgraph._node[pair[1]]),
                            puavs, pweather)

        pgraph.add_edge(pair[1], pair[0], type = type2, Wt = W_t, We = W_e) 

    base_list = []
    for base in pbases:
        base_list.append(base.get_Name())

        pgraph.add_node(base.get_Name(), UTM = base.get_Coordinates(), type = 'Base')
        for node in tower_nodes:

            W_t, W_e = WE.compute_Edge_Weights((node, pgraph._node[node]),
                               (base.get_Name(), pgraph._node[base.get_Name()]),
                                puavs, pweather)
            
            pgraph.add_edge(node, base.get_Name(), type = 'TB', Wt = W_t, We = W_e)

            W_t, W_e = WE.compute_Edge_Weights((node, pgraph._node[node]),
                               (base.get_Name(), pgraph._node[base.get_Name()]),
                                puavs, pweather)
            
            pgraph.add_edge(base.get_Name(), node, type = 'BT', Wt = W_t, We = W_e)

    # The current graph is a complete graph that connects all towers and bases with weighted edges
    # Now, lets setup SCIPI

    # --------------------------------- SCIPI ---------------------------------------------------------

    # Update progress bar if it is linked
    if problem.progress_Bar_LinkedQ(): problem.set_Progress_Bar_Value(25)

    # Add variables to the model

    Z = {}        # Type Binary free parameteres that active travels within a route
    sigmas = {}   # Type Double 'free' parameters that measure time differences between UAV's paths

    t = {}        # To store ALL time weights as a dictionary
    e = {}        # To store ALL enery weights as a dictionary

    Wt = nx.get_edge_attributes(pgraph, 'Wt')
    We = nx.get_edge_attributes(pgraph, 'We')
    
    # Create the z variables on SCIPI. Also creates the t and e dictionaries
    for edge in pgraph.edges():
            for uav in puavs:

                Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B',
                                                    name = 'z_{'+edge[0]+'->'+edge[1]+'|'+uav.get_ID()+'}')
                
                t[edge[0]+'->'+edge[1]+'|'+uav.get_ID()]  = Wt[(edge[0], edge[1], 0)][uav.get_ID()]
                e[edge[0]+'->'+edge[1]+'|'+uav.get_ID()]  = We[(edge[0], edge[1], 0)][uav.get_ID()]


    # The N(N-1)/2 UAVs pairs. No repetitions and no order.
    uav_pairs = list(itertools.combinations(puavs.get_List(), 2))

    # Create the sigma variables on SCIPI. Also add the sigma definition constrains
    for pair in uav_pairs:
            sigmas[pair[0].get_ID()+','+pair[1].get_ID()] = pmodel.addVar(vtype = 'C',
                                                        name='sigma_{'+pair[0].get_ID()+','+pair[1].get_ID()+'}')

            # Sigma constrains

            """
            # Nonlinear version. Might freeze SCIP even if it is able to handle it
            problem.scipi_model.addCons(

                sigmas[pair[0].id+','+pair[1].id] 
                            >= 
                np.abs(quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[0].id] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].id] 
                              - t[edge[0]+'->'+edge[1]+'|'+pair[1].id] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].id] 
                            for edge in problem.graph.edges()))

            )
            """
            # Linearized version. It is way less problematic
            pmodel.addCons(
                SCIP.quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] 
                              - t[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] 
                            for edge in pgraph.edges())
                - sigmas[pair[0].get_ID()+','+pair[1].get_ID()] 
                            <= 
                0.0
            )

            pmodel.addCons(
                SCIP.quicksum(t[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[1].get_ID()] 
                              - t[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+pair[0].get_ID()] 
                            for edge in pgraph.edges())
                - sigmas[pair[0].get_ID()+','+pair[1].get_ID()] 
                            <= 
                0.0
            )

    # Lets define all constrain except for the already defined sigmas
    for uav in puavs:
        # Energy constrain. Used battery needs to be less than 100%
        pmodel.addCons(
            SCIP.quicksum(e[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] * Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] 
                     for edge in pgraph.edges()) 
                <= 
            1.0
        )

        # Starting and finishing at base.
        pmodel.addCons(
            SCIP.quicksum(Z[uav.missionSettings['Base']+'->'+tower+'|'+uav.get_ID()] for tower in tower_nodes)
                ==
            1.0
        )

        pmodel.addCons(
            SCIP.quicksum(Z[tower+'->'+uav.missionSettings['Base']+'|'+uav.get_ID()] for tower in tower_nodes)
                ==
            1.0
        )

        # Each UAV needs to exit a node (that is not a base, towers in this case) if it enters it
        # THIS DOES NOT ENSURE CONNECTIVITY. FIX
        
        for node1 in tower_nodes:
            other_towers = copy.deepcopy(list(pgraph.nodes()))
            other_towers.remove(node1)
            pmodel.addCons(
                SCIP.quicksum(Z[node2+'->'+node1+'|'+uav.get_ID()] - Z[node1+'->'+node2+'|'+uav.get_ID()] for node2 in other_towers)
                    ==
                0
            )
        
    """
    for node in tower_nodes:
        for uav in problem.uav_team.list:

            Y[node+','+uav.id] = problem.scipi_model.addVar(vtype = 'B',
                                                            name='Y_{'+node+','+uav.id+'}')
            
            other_towers = copy.deepcopy(list(problem.graph.nodes()))
            other_towers.remove(node)

            problem.scipi_model.addCons(
                quicksum(Z[node2+'->'+node+'|'+uav.id] + Z[node+'->'+node2+'|'+uav.id] for node2 in other_towers)
                    ==
                2 * Y[node+','+uav.id]
            )
    """

    # Select power line segments
    edges_types = nx.get_edge_attributes(pgraph, 'type')
    power_lines = []
    for edge in edges_types:
        if edges_types[edge] == 'Sup': power_lines.append(edge)

    # ????? Ask JAVI ???????
    for edge in power_lines:
        pmodel.addCons(
            SCIP.quicksum(Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] + Z[edge[1]+'->'+edge[0]+'|'+uav.get_ID()]
                     for uav in puavs)
                ==
            1.0
        )

        pmodel.addCons(
            SCIP.quicksum(Z[edge[0]+'->'+edge[1]+'|'+uav.get_ID()] for uav in puavs)
                <=
            1.0
        )

        pmodel.addCons(
            SCIP.quicksum(Z[edge[1]+'->'+edge[0]+'|'+uav.get_ID()] for uav in puavs)
                <=
            1.0
        )
        
    # Let's set the objective function for SCIPI. Let's add a fixed parameter to tweak the importance
    # of each term. It should arrive at the same the solution regardless but it might help convergence

    f_k = 0.5 # This parameter requieres finetunning. It is useful not to fix it at 1.0

    pmodel.setObjective(SCIP.quicksum(t[key]*Z[key] for key in Z.keys()) 
                                     + 
                                f_k * SCIP.quicksum(sigmas[key] for key in sigmas.keys()))

    #print(problem.scipi_model.getObjective())

    #problem.scipi_model.setHeuristics(SCIP_PARAMSETTING.OFF) # Disable HEURISTICS
    #problem.scipi_model.disablePropagation()                 # Disable solution Propagation
    

    # Update progress bar if it is linked
    if problem.progress_Bar_LinkedQ(): problem.set_Progress_Bar_Value(30)

    # Write Scipi Problem externally into a human-readable file
    pmodel.writeProblem('scip_model.cip')

    # Update progress bar if it is linked
    if problem.progress_Bar_LinkedQ(): problem.set_Progress_Bar_Value(35)

    # Optimize
    pmodel.optimize()

    # Update progress bar if it is linked
    if problem.progress_Bar_LinkedQ(): problem.set_Progress_Bar_Value(90)

    # Get the best solution
    sol = pmodel.getBestSol()

    #print(pmodel.checkSol(sol))

    # --------------------------------- Routes Parsing ---------------------------------------------------------

    # Parse solution into each UAV Route

    # First reset every UAV Routes
    for uav in puavs:
        uav.route = []

    for edge in Z:
        if sol[Z[edge]] == 1.0:

            temp = edge.split('|')

            # Add edge to UAV Route. Later, it will be ordered.
            puavs.select_UAV(temp[1]).route.append(temp[0])

            nodes = temp[0].split('->')
            if nodes[0][0] == 'B' or nodes[1][0] == 'B':
                puavs.select_UAV(temp[1]).routeModes.append('Navigation')
            else:
                puavs.select_UAV(temp[1]).routeModes.append('Inspection')

    for uav_pair in sigmas:
        print('sigma_', uav_pair, ' = ' ,sol[sigmas[uav_pair]])

    for uav in puavs:
        print('NO = ', uav.route)
        uav.route, uav.routeModes = order_Route(uav.missionSettings['Base'], uav.route, uav.routeModes)
        uav.routeUTM = route_to_UTM(uav.route, ptowers, pbases)
        print('O = ', uav.route)
        print(uav.routeModes)
    
    # Update progress bar if it is linked
    if problem.progress_Bar_LinkedQ(): problem.set_Progress_Bar_Value(100)

    return None

def abstract_Solver(problem: Problem):
    """
    This formulation creates a astraction layer that assigns a graph to the problem which nodes
    are not the towers but the power line segments that require inspection. It scales faster than
    "regular" with the size of the problem but it is more general and less limited. Currently, charging stations are not implemented. 
    """

    ptowers = problem.get_Towers() # This does not duplicate in memory, just "reference" it.
                                   # This is just default python. Just for faster coding and less referencing
    pbases = problem.get_Bases()
    puavs = problem.get_UAV_Team()
    pweather = problem.get_Weather()
    pgraph = problem.get_Graph()
    pmodel =  problem.get_SCIP_Model() 

    #--------------------------------- Graph ---------------------------------------------------------

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
                                            pweather)

        
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
                                            pweather)


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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)


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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

            pgraph.add_edge(
                'SDOWN_{'+s_tag+'}',
                base.get_Name(),
                type = 'TB',
                Wt = W_t,
                We = W_e
            )

    # With this, the abstract graph is fully constructed.     
        
    # ------------------------------ SCIP ------------------------------------
    # Let's define the SCIP problem, compute the solution and update the UAVs routes
    
    # Dictionaries to store SCIP variables
    Z = {}
    sigmas = {}
    Y = {}

    # Dictionaries to store weights for easy access
    t = {}
    e = {}

    Wt = nx.get_edge_attributes(pgraph, 'Wt')
    We = nx.get_edge_attributes(pgraph, 'We')

    # For each edge in the graph assign a Z variable and assign the corresponding weights
    # to its key in the weights dictionaries
    for edge in pgraph.edges():

        for uav in puavs:

            edge_uav = edge[0]+'->'+edge[1]+'|'+uav.get_ID() # Edge|UAV Tag

            Z[edge_uav] = pmodel.addVar(vtype = 'B',
                                                  name = 'Z_{'+edge_uav+'}')
            
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

            Y['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B',
              name = 'Y_{SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()+'}')

            pmodel.addCons(
                SCIP.quicksum(Z[node+'->'+'SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                       + Z['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                for node in nodelist)
                    == 
                2 * Y['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
            )

            Y['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B',
              name = 'Y_{SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()+'}')

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
            sigmas[pair_tag] = pmodel.addVar(vtype = 'C',
                                                        name='sigma_{'+pair_tag+'}')

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

    # THIS THING ONLY WORKS FOR SOME CASES. IT STILL PRESENTS SUBROUTES FOR MANY OTHERS
    # --------------------- Subroute S.T.U.P.I.D. Solution ----------------------------
            
    # First see if the towers are connected or not.
            
    tgraph = ptowers.get_Graph()
    
    if not nx.is_connected(tgraph):

        # If the towers are not connected, then compute a list with the list of towers of each component
        SC = [tgraph.subgraph(c).copy() for c in nx.connected_components(tgraph)]
        S = [list(tgraph.subgraph(c).nodes()) for c in nx.connected_components(tgraph)]
        print(S)

        # Connectivity for a pair of subcomponents. First, we need all the different pairs of towers
        # from each subcomponent

        towers_pairs = list(itertools.product(*S))

        # Lets create SUP and SDOWN for each subcomponent

        SC_Nodes = []

        for subcomponent in SC:

            sup = []
            sdown = []
            
            for line_segment in subcomponent.edges():


                # Tower ordering might differ. This ensures it is the same
                if pgraph.has_node('SUP_{'+line_segment[0]+','+line_segment[1]+'}'):
                    sup.append('SUP_{'+line_segment[0]+','+line_segment[1]+'}')
                    sdown.append('SDOWN_{'+line_segment[0]+','+line_segment[1]+'}')
                else: 
                    sup.append('SUP_{'+line_segment[1]+','+line_segment[0]+'}')
                    sdown.append('SDOWN_{'+line_segment[1]+','+line_segment[0]+'}')

            SC_Nodes.append(sup+sdown)


        print(SC_Nodes)


        # Lets force the entry of at least one UAV in the subcomponent. Bases are still needed to be added
        pmodel.addCons(
            SCIP.quicksum(
                SCIP.quicksum(
                    SCIP.quicksum(

                            Z[node0+'->'+node1+'|'+uav.get_ID()]

                    for node1 in SC_Nodes[1])
                for node0 in SC_Nodes[0])
            for uav in problem.get_UAV_Team())
            >= 
            1.0
        )
        
            

    # --------------------- Subroute S.T.U.P.I.D. Solution ----------------------------
    

    f_k = 1.0 #0.5 # This parameter requieres finetunning. It is useful not to fix it at 1.0

    pmodel.setObjective(SCIP.quicksum(t[key]*Z[key] for key in Z.keys()) 
                                     + 
                                f_k * SCIP.quicksum(sigmas[key] for key in sigmas.keys()))

    #print(pmodel.getObjective())

    #pmodel.setHeuristics(SCIP_PARAMSETTING.OFF) # Disable HEURISTICS
    #pmodel.disablePropagation()                 # Disable solution Propagation

    # Write Scipi Problem externally into a human-readable file
    pmodel.writeProblem('scip_model.cip')
            
    pmodel.optimize()
    sol = pmodel.getBestSol()

    #print(problem.scipi_model.checkSol(sol))

    # -------------------- Route parsing -----------------------------

    for uav in puavs:
        uav.route = []
        uav.routeAbstract = []
    
    for edge in Z:

        if np.abs(sol[Z[edge]] - 1.0) < 1e-6: # To avoid floating point errors
            
            
            temp = edge.split('|')

            # puavs.selectUAV(temp[1]).route.append(temp[0])

            nodes = temp[0].split('->')

            puavs.select_UAV(temp[1]).routeAbstract.append(temp[0])

            
            print(nodes, temp[1])

            # B to S
            if nodes[0][0] == 'B':

                print('B to S')
                
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

                print('S to B')

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

                print('S to S')

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
            
            print('t = ', puavs.get_List()[0].route)

    for uav_pair in sigmas:
        print('sigma_', uav_pair, ' = ' ,sol[sigmas[uav_pair]])

    for uav in puavs:
        print('NO = ', uav.route)
        uav.route, uav.routeModes = order_Route(uav.missionSettings['Base'], uav.route, uav.routeModes)
        uav.routeUTM = route_to_UTM(uav.route, ptowers, pbases)
        print('O = ', uav.route)
        print(uav.routeModes)

    return None

def abstract_Solver_with_ordering(problem: Problem):
    """
    It is the same as the abstract solver excepts that it implements some extra variables to assign order within a UAV Route. 
    This allows for a basic but general implementation for subroute elimination.
    """

    ptowers = problem.get_Towers() # This does not duplicate in memory, just "reference" it.
                                   # This is just default python. Just for faster coding and less referencing
    pbases = problem.get_Bases()
    puavs = problem.get_UAV_Team()
    pweather = problem.get_Weather()
    pgraph = problem.get_Graph()
    pmodel =  problem.get_SCIP_Model() 

    #--------------------------------- Graph ---------------------------------------------------------

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
                                            pweather)

        
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
                                            pweather)


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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)


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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

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
                                            pweather)

            pgraph.add_edge(
                'SDOWN_{'+s_tag+'}',
                base.get_Name(),
                type = 'TB',
                Wt = W_t,
                We = W_e
            )

    # With this, the abstract graph is fully constructed.     
        
    # ------------------------------ SCIP ------------------------------------
    # Let's define the SCIP problem, compute the solution and update the UAVs routes
    
    # Dictionaries to store SCIP variables
    Z = {}
    sigmas = {}
    Y = {}

    # Dictionaries to store weights for easy access
    t = {}
    e = {}

    Wt = nx.get_edge_attributes(pgraph, 'Wt')
    We = nx.get_edge_attributes(pgraph, 'We')

    # For each edge in the graph assign a Z variable and assign the corresponding weights
    # to its key in the weights dictionaries
    for edge in pgraph.edges():

        for uav in puavs:

            edge_uav = edge[0]+'->'+edge[1]+'|'+uav.get_ID() # Edge|UAV Tag

            Z[edge_uav] = pmodel.addVar(vtype = 'B',
                                                  name = 'Z_{'+edge_uav+'}')
            
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

            Y['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B',
              name = 'Y_{SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()+'}')

            pmodel.addCons(
                SCIP.quicksum(Z[node+'->'+'SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
                       + Z['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'->'+node+'|'+uav.get_ID()]
                for node in nodelist)
                    == 
                2 * Y['SUP_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()]
            )

            Y['SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()] = pmodel.addVar(vtype = 'B',
              name = 'Y_{SDOWN_{'+line_segment[0]+','+line_segment[1]+'}'+'|'+uav.get_ID()+'}')

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
            sigmas[pair_tag] = pmodel.addVar(vtype = 'C',
                                                        name='sigma_{'+pair_tag+'}')

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

    # THIS THING ONLY WORKS FOR SOME CASES. IT STILL PRESENTS SUBROUTES FOR MANY OTHERS
    # --------------------- Subroute S.T.U.P.I.D. Solution ----------------------------
            
    # First see if the towers are connected or not.
            
    tgraph = ptowers.get_Graph()
    
    if not nx.is_connected(tgraph):

        # If the towers are not connected, then compute a list with the list of towers of each component
        SC = [tgraph.subgraph(c).copy() for c in nx.connected_components(tgraph)]
        S = [list(tgraph.subgraph(c).nodes()) for c in nx.connected_components(tgraph)]
        print(S)

        # Connectivity for a pair of subcomponents. First, we need all the different pairs of towers
        # from each subcomponent

        towers_pairs = list(itertools.product(*S))

        # Lets create SUP and SDOWN for each subcomponent

        SC_Nodes = []

        for subcomponent in SC:

            sup = []
            sdown = []
            
            for line_segment in subcomponent.edges():


                # Tower ordering might differ. This ensures it is the same
                if pgraph.has_node('SUP_{'+line_segment[0]+','+line_segment[1]+'}'):
                    sup.append('SUP_{'+line_segment[0]+','+line_segment[1]+'}')
                    sdown.append('SDOWN_{'+line_segment[0]+','+line_segment[1]+'}')
                else: 
                    sup.append('SUP_{'+line_segment[1]+','+line_segment[0]+'}')
                    sdown.append('SDOWN_{'+line_segment[1]+','+line_segment[0]+'}')

            SC_Nodes.append(sup+sdown)


        print(SC_Nodes)


        # Lets force the entry of at least one UAV in the subcomponent. Bases are still needed to be added
        pmodel.addCons(
            SCIP.quicksum(
                SCIP.quicksum(
                    SCIP.quicksum(

                            Z[node0+'->'+node1+'|'+uav.get_ID()]

                    for node1 in SC_Nodes[1])
                for node0 in SC_Nodes[0])
            for uav in problem.get_UAV_Team())
            >= 
            1.0
        )
        
            

    # --------------------- Subroute S.T.U.P.I.D. Solution ----------------------------
    

    f_k = 1.0 #0.5 # This parameter requieres finetunning. It is useful not to fix it at 1.0

    pmodel.setObjective(SCIP.quicksum(t[key]*Z[key] for key in Z.keys()) 
                                     + 
                                f_k * SCIP.quicksum(sigmas[key] for key in sigmas.keys()))

    #print(pmodel.getObjective())

    #pmodel.setHeuristics(SCIP_PARAMSETTING.OFF) # Disable HEURISTICS
    #pmodel.disablePropagation()                 # Disable solution Propagation

    # Write Scipi Problem externally into a human-readable file
    pmodel.writeProblem('scip_model.cip')
            
    pmodel.optimize()
    sol = pmodel.getBestSol()

    #print(problem.scipi_model.checkSol(sol))

    # -------------------- Route parsing -----------------------------

    for uav in puavs:
        uav.route = []
        uav.routeAbstract = []
    
    for edge in Z:

        if np.abs(sol[Z[edge]] - 1.0) < 1e-6: # To avoid floating point errors
            
            
            temp = edge.split('|')

            # puavs.selectUAV(temp[1]).route.append(temp[0])

            nodes = temp[0].split('->')

            puavs.select_UAV(temp[1]).routeAbstract.append(temp[0])

            
            print(nodes, temp[1])

            # B to S
            if nodes[0][0] == 'B':

                print('B to S')
                
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

                print('S to B')

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

                print('S to S')

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
            
            print('t = ', puavs.get_List()[0].route)

    for uav_pair in sigmas:
        print('sigma_', uav_pair, ' = ' ,sol[sigmas[uav_pair]])

    for uav in puavs:
        print('NO = ', uav.route)
        uav.route, uav.routeModes = order_Route(uav.missionSettings['Base'], uav.route, uav.routeModes)
        uav.routeUTM = route_to_UTM(uav.route, ptowers, pbases)
        print('O = ', uav.route)
        print(uav.routeModes)

    return None

def GRASP_Solver():
    """
    Using the abstractSolver, let's implement the GRASP method to find a solution. To be implement
    """
    # TBD
    return None

def order_Route(start: str, edge_list: list, mode_list:list):
    """
    Given and unordered route, it orders following the nodes from the starting base.
    It suposses that routes are closed and if it has any subroutes or ambiguity it will
    take a choice based on ordering and order each subpath.

    Here, the graph considered is physical and not the abstract one.

    # NOT FULLY TESTED CODE. MIGHT ENTER INFINITE LOOP
    """

    n_e = len(edge_list)

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

def route_to_UTM(route: list, towers: TW.Towers, bases:BA.Bases):
    """
    Transforms each node on each edge of the route tu UTM coordinates.
    Return a list with a 2-tuple (UTM1, UTM2) per original edge
    """
    routeUTM = []
    for edge in route:
        
        if edge[0][0] == 'B':
                UTM1 = bases.get_Base(edge[0]).get_Coordinates()
                UTM2 = towers.get_Tower_Coordinates(edge[1])
        
        elif edge[1][0] == 'B':
                UTM1 = towers.get_Tower_Coordinates(edge[0])
                UTM2 = bases.get_Base(edge[1]).get_Coordinates()
        
        else:
            UTM1 = towers.get_Tower_Coordinates(edge[0])
            UTM2 = towers.get_Tower_Coordinates(edge[1])
        
        routeUTM.append((UTM1, UTM2))

    return routeUTM


