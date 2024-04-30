import numpy as np, json, networkx as nx, random
from itertools import combinations, groupby
from hilbert import decode

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO

def load_Problem_from_File(file_path: str) -> SO.Problem:

    f = open(file_path)
    mission = json.load(f)
    f.close() 

    bases = BA.Bases()
    for name in mission["Bases"]:
        bases.add_Base(
            name,
            np.array(mission["Bases"][name])
        )

    towers = TS.Towers()
    for name in mission["Towers"]["List"]:
        towers.add_Tower(
            name,
            np.array(mission["Towers"]["List"][name])
        )
    for power_line in mission["Towers"]["Lines"]:
        towers.add_Power_Lines((power_line[0], power_line[1]))

    uavs = UAVS.UAV_Team()
    for id_ in mission["UAVs"]:
        uavs.add_UAV(UAVS.UAV(
            id = id_, model = mission["UAVs"][id_]["Model"], base = mission["UAVs"][id_]["Base"]
        ))

    tasks = TS.Tasks()
    for name in mission["Tasks"]:

        if list == type(mission["Tasks"][name]["inspection_of"]) and 2 == len(mission["Tasks"][name]["inspection_of"]):
            try:
                tasks.add_Task(
                    name,
                    inspection_of = (mission["Tasks"][name]["inspection_of"][0], mission["Tasks"][name]["inspection_of"][1]),
                    incompatible_IDs = mission["Tasks"][name]["incompatible_IDs"]
                )
            except:
                tasks.add_Task(
                    name,
                    inspection_of = (mission["Tasks"][name]["inspection_of"][0], mission["Tasks"][name]["inspection_of"][1])
                )

        else:
            try:
                tasks.add_Task(
                    name,
                    inspection_of = mission["Tasks"][name]["inspection_of"],
                    incompatible_IDs = mission["Tasks"][name]["incompatible_IDs"]
                )
            except:
                tasks.add_Task(
                    name,
                    inspection_of = mission["Tasks"][name]["inspection_of"]
                )
        

    if "Wind" in mission:
        return SO.Problem(bases, towers, tasks, uavs, wind_vector = np.array(mission["Wind"]))
    else: return SO.Problem(bases, towers, tasks, uavs)

def gen_Hilbert_Map(num_dims: int, num_bits: int, d_towers: float):

    max_h = 2**(num_bits*num_dims)

    # Generate a sequence of Hilbert integers.
    hilberts = np.arange(max_h)

    # Compute the 2-dimensional locations.
    locs =  d_towers * decode(hilberts, num_dims, num_bits)

    base_p = int(max_h / 2)
    data = {
        "Bases": {
            "B": [float(locs[base_p,0]), float(locs[base_p,1]), 0]
        },
        "UAVs": {
            "0": {
            "Model": "A",
            "Base": "B"
        },
        },
        "Wind": [0, 0, 0]
    }

    tlist = {}
    tasks = {}
    lines = []
    for k in range(base_p):
        tlist["T"+str(k)] = [float(locs[k,0]), float(locs[k,1]), 0]

        if k < base_p-1:
            edge = ["T"+str(k),"T"+str(k+1)] 
            lines.append(edge)
            tasks["tS"+str(k)] = {"inspection_of": edge}
    
    for k in range(base_p+1, max_h):
        tlist["T"+str(k)] = [float(locs[k,0]), float(locs[k,1]), 0]
        if k < max_h-1: 
            edge = ["T"+str(k),"T"+str(k+1)] 
            lines.append(edge)
            tasks["tS"+str(k)] = {"inspection_of": edge}

    data["Towers"] = {"List": tlist, "Lines": lines}
    data["Tasks"]= tasks

    with open('./files/HILBERT_U1S'+str(max_h-1)+'.json', 'w') as f:
        json.dump(data, f, indent=4)

def gnp_random_connected_graph(n, p):
    """
    Generates a random undirected graph, similarly to an Erdős-Rényi 
    graph, but enforcing that the resulting graph is conneted
    """
    edges = combinations(range(n), 2)
    G = nx.Graph()
    G.add_nodes_from(range(n))
    if p <= 0:
        return G
    if p >= 1:
        return nx.complete_graph(n, create_using=G)
    for _, node_edges in groupby(edges, key=lambda x: x[0]):
        node_edges = list(node_edges)
        random_edge = random.choice(node_edges)
        G.add_edge(*random_edge)
        for e in node_edges:
            if random.random() < p:
                G.add_edge(*e)
    return G

def generate_random_PLN(n_towers: int, p_connections: float) -> nx.Graph:

    graph = nx.empty_graph()

    notplanar = True
    while notplanar:
        graph = gnp_random_connected_graph(n_towers, p_connections)
        is_planar = nx.is_planar(graph)
        notplanar = not is_planar

    return graph

def transform_TSPLIB_File(i_file_path: str, o_file_path: str):

    f = open(i_file_path, 'r')
    contents = f.readlines()

    tlist = {}
    tasks = {}

    firstQ = True
    for line in contents[7:]:

        if "DEMAND_SECTION\n" == line:
            break

        parsed_line = line.strip().split(" ")

        if firstQ:
            data = {"Bases": {
                "B": [float(parsed_line[1]), float(parsed_line[2]), 0]
                    },
                "UAVs": {
                    "0": {
                        "Model": "A",
                        "Base": "B"
                    },
                },
                "Wind": [0, 0, 0]
            }
            firstQ = False
            continue

        tlist["P"+parsed_line[0]] = [float(parsed_line[1]), float(parsed_line[2]), 0]
        tasks["t"+"P"+parsed_line[0]] = {"inspection_of": "P"+parsed_line[0]}

    #del tasks["P"+parsed_line[0]]

    


    data["Towers"] = {"List": tlist, "Lines": {}}
    data["Tasks"] = tasks

    with open(o_file_path, 'w') as f:
        json.dump(data, f, indent=4)

    return None

