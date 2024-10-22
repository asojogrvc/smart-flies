# Weights computing code

import numpy as np
from scipy.optimize import fsolve

import modules.uav as UAVS, modules.weather as WT

# These parameters might be loaded externally o get passed to the functions

g = 9.81     # Gravity acceleration [m/s^2]

# -------------------------------------- General functions -----------------------------------

def compute_Consumption(r_vector: np.ndarray, v:float, uav: UAVS.UAV, weather: WT.Weather) -> float:
    """
    Based on the Aerial-Core Consumption Model, computes the energy neede for a travel "r_vector" at
    speed "v" for the UAV "uav" and weather conditions "weather"
    """

    data = uav.get_UAV_Parameters()

    mass = data["mass"]
    number_of_Rotors = data["number_of_Rotors"]
    rotor_Blades = data["rotor_Blades"]
    rotor_Radius = data["rotor_Radius"]
    blade_Chord = data["blade_Chord"]
    lift_Coefficient = data["lift_Coefficient"]
    drag_Coefficient = data["drag_Coefficient"]
    kappa = data["kappa"]
    eta = data["eta"]
    K_mu  = data["K_mu"]
    flat_Area = data["flat_Area"]
    data["camera"]
    battery_capacity_Joules = data["battery"].capacity_Joules()

    # What if a travel is too long and a UAV dies mid travel?

    # Travel distance
    d = np.linalg.norm(r_vector) #[m]

    # UAV air speed as a composition of ground speed and wind speed
    v_inf = np.linalg.norm(v*r_vector/d-weather.get_Wind_Vector()) #[m/s]

    # Drag 
    Dr = 0.5 *  weather.get_Air_Density() * v_inf**2 * flat_Area #[kg/m^3 * m^2 / s^2 * m*2 = N]

    # Thrust and thrust angle 
    alpha_r = np.arctan( Dr / (mass * g)) #[rad]
    Th = mass * g / (number_of_Rotors * np.cos(alpha_r)) #[N]

    # Rotational speed of the rotors [s^-1]
    Omega = 6 * Th / (weather.get_Air_Density() * rotor_Blades * blade_Chord * rotor_Radius**3 * lift_Coefficient)
    Omega = np.sqrt( Omega - 1.5 * ((v_inf * np.cos(alpha_r)) / (rotor_Radius))**2)


    # Parisitic power due to drag
    P_f = Dr * v_inf #[N * m/s = W]

    # Rotor blades drag power profile [W]
    P_o = 0.125 * weather.get_Air_Density() * rotor_Radius**4 * Omega**3 * blade_Chord * rotor_Blades * drag_Coefficient
    P_o = P_o * (1 + K_mu * (v_inf * np.cos(alpha_r) / (Omega * rotor_Radius))**2)

    # Auxiliary param and induced speed [m/s]. Gamma can be solved analitically (?)
    f = lambda Gamma: Gamma - v_inf * np.sin(alpha_r) - Th / (2 * weather.get_Air_Density() * np.pi * rotor_Radius**2) / np.sqrt(Gamma**2 + (v_inf * np.cos(alpha_r))**2)
    v_i = fsolve(f, v_inf)[0] - v_inf * np.sin(alpha_r)

    # Speed up rotor power [N * m/s = W]
    P_i = kappa * Th * v_i

    # Total mechanical power [W]
    P_a = number_of_Rotors * ( P_i + P_o ) + P_f

    # Electrical Power [W]
    P_e = P_a / eta
    
    # Energy consumed as a fraction of the UAV capacity
    # It can be bigger than one if the travel is too long.
    # Might be a good idea to check this somehow to warn the user.

    return (P_e * d/v) / battery_capacity_Joules

# -------------------------------------- Regular Solver functions -----------------------------------

def compute_Edge_Weights(node1:tuple, node2:tuple, uavs: UAVS.UAV_Team, weather: WT.Weather) -> tuple[dict, dict]:
    """
    It takes an edge as a ordered pair of nodes Node1, Node2 each with
    the corresponding attributes and a UAV team.
    Then it computes the corresponding time and energy weights depending type.

    node = (Name, dict_Attributes)
    type can be computed from the two nodes attributes
    """

    travel_vector = node2[1]['Coords']-node1[1]['Coords']
    d = np.linalg.norm(travel_vector)

    W_t = {}
    W_e = {}

    # Line segment edge
    if node1[1]['type'] == 'Tower' and node2[1]['type'] == 'Tower':

        # UAVs are distinguished using their IDs, not their names
        for uav in uavs:

            i_speed = uav.missionSettings['Insp. speed']

            # As it is independent of the UAV, time can be computed beforehand
            W_t[uav.get_ID()] = d / i_speed

            
            W_e[uav.get_ID()] = compute_Consumption(travel_vector, i_speed, uav, weather)  # + whatever
    
    # Base-Tower or Tower-Base edge
    else:
    
        # UAVs are distinguished using their IDs, not their names
        for uav in uavs:

            c_speed = uav.missionSettings['Nav. speed']

            # As it is independent of the UAV, time can be computed beforehand
            W_t[uav.get_ID()] = d / c_speed

            
            W_e[uav.get_ID()] = compute_Consumption(travel_vector, c_speed, uav, weather)

    return W_t, W_e

# -------------------------------------- Abstract Solver functions -----------------------------------

def compute_Abstract_Edge_Weights(node1: tuple, node2: tuple, uavs: UAVS.UAV_Team, weather: WT.Weather, mode: int) -> tuple[dict, dict]:
    """
    Arguments depend on case

    Mode 0:
     - Base to Tower: node1 is a tuple ('Name', Coords); node2 is a list (tower1, tower2) with toweri = ('Name', Coords)
     - Tower to Base: node2 is a tuple ('Name', Coords); node1 is a list (tower1, tower2) with toweri = ('Name', Coords)
     - Tower to Tower: node1 and node2 are lists (tower1, tower2) with toweri = ('Name', Coords)

    Mode 1:
     - node1 and node2 are tuples ('Name', Coords)
    """

    match mode:
        case 0:

            W_t = {}
            W_e1 = {}
            W_e2 = {}

            # Parsing cases

            # Base to Tower
            if node1[0][0] == 'B':

                travel_vector = node2[0][1] - node1[1]
                d_travel = np.linalg.norm(travel_vector)

                insp_vector = node2[1][1] - node2[0][1]
                d_insp = np.linalg.norm(insp_vector)

                # UAVs are distinguished using their IDs, not their names
                for uav in uavs:

                    c_speed = uav.missionSettings['Nav. speed']
                    i_speed = uav.missionSettings['Insp. speed']

                    W_t[uav.get_ID()] = d_insp / i_speed + d_travel / c_speed

                    W_e1[uav.get_ID()] = compute_Consumption(travel_vector, c_speed, uav, weather)
                    W_e2[uav.get_ID()] = compute_Consumption(insp_vector, i_speed, uav, weather)

                #print('BT')
                return W_t, {key: W_e1[key] + W_e2[key] for key in W_e1}
    
            # Tower to Base
            elif node2[0][0] == 'B':

                travel_vector = node2[1] - node1[1][1]
                d_travel = np.linalg.norm(travel_vector)

                for uav in uavs:

                    c_speed = uav.missionSettings['Nav. speed']

                    W_t[uav.get_ID()] = d_travel / c_speed

                    W_e1[uav.get_ID()] = compute_Consumption(travel_vector, c_speed, uav, weather)

                #print('SB')
                return W_t, W_e1
    
            # S to S
            else:

                insp_vector = node2[1][1] - node2[0][1]
                d_insp = np.linalg.norm(insp_vector)

                if node2[0][0] == node1[1][0]:
                    for uav in uavs:
                        
                        i_speed = uav.missionSettings['Insp. speed']

                        W_t[uav.get_ID()] = d_insp / i_speed
                        W_e1[uav.get_ID()] = compute_Consumption(insp_vector, i_speed, uav, weather)

                    # print('SS2')
                    return W_t, W_e1
        
                travel_vector = node2[0][1] - node1[1][1]
                d_travel = np.linalg.norm(travel_vector)
                
                # AQUI

                #insp_vector = node2[1][1] - node2[0][1]
                #d_insp = np.linalg.norm(insp_vector)

                # UAVs are distinguished using their IDs, not their names
                for uav in uavs:

                    c_speed = uav.missionSettings['Nav. speed']
                    i_speed = uav.missionSettings['Insp. speed']

                    W_t[uav.get_ID()] = d_insp / i_speed + d_travel / c_speed

                    W_e1[uav.get_ID()] = compute_Consumption(travel_vector, c_speed, uav, weather)
                    W_e2[uav.get_ID()] = compute_Consumption(insp_vector, i_speed, uav, weather)

                #print('SS')
                return W_t, {key: W_e1[key] + W_e2[key] for key in W_e1}
        
        
        # Point Inspection
        case 1:
            
            W_t = {}
            W_e = {}

            travel_vector = node2[1] - node1[1]
            d_travel = np.linalg.norm(travel_vector)


            # UAVs are distinguished using their IDs, not their names
            for uav in uavs:

                c_speed = uav.missionSettings['Nav. speed']

                W_t[uav.get_ID()] = d_travel / c_speed
                W_e[uav.get_ID()] = compute_Consumption(travel_vector, c_speed, uav, weather)

            return W_t, W_e


            


    



