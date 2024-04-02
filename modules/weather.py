# Weather status class and functions

import numpy as np, requests

# Weather is considered homogeneus on the entire mission area and constant throughout the inspection
class Weather():

    # Init of the class with a default weather status. There is no need to check if values
    # are physical as user input is already limited on the UI.
    def __init__(self, wind_vec: np.ndarray = [0.0, 0.0, 0.0], Temperature: float = 20.0, 
                 Humidity: float = 0.3, Pressure: float = 101325.0):

        self.__wind_vector = wind_vec      #[m/s] 
        self.__temperature = Temperature   #[degrees Celsius]
        self.__humidity = Humidity         #[-]
        self.__pressure = Pressure         #[Pa]

        self.__rho_air = air_Density(self) #[kg / m^3]


    # ------------------------ Setting parameter functions ---------------------------
        
        # SANITY CHECKS?
        
    def set_Wind_Vector(self, wind_vector: np.ndarray):
        """
        Sets the wind vector for an exisiting weather status. The length of the vector is the speed of the wind [m/s] and
        the direction is the direction of the wind.
        """

        self.__wind_vector = wind_vector
        self.__rho_air = air_Density(self)

        return None
    
    def set_Temperature(self, temperature: float):
        """
        Sets the temperature (ºC) of an existing weather status
        """

        self.__temperature = temperature
        self.__rho_air = air_Density(self)

        return None
    
    def set_Humidity(self, humidity: float):
        """
        Sets the humidity (R%) of an existing weather status
        """

        self.__humidity = humidity
        self.__rho_air = air_Density(self)

        return None
    
    def set_Pressure(self, pressure: float):
        """
        Sets the atmospheric pressure (Pa) of an existing weather status
        """
        self.__pressure = pressure
        self.__rho_air = air_Density(self)

        return None

    def update_Online(self, latlon: np.ndarray):
        # Location must be given as [Latitude, Longitud]

        apiKey = 'f4e4e58c9c0c4492aeb144023231005'; 
        url = 'http://api.weatherapi.com/v1/current.json?key='+apiKey+'&q='+f"{latlon[0]}"+','+f"{latlon[1]}"+'&aqi=no'

        try: response = requests.get(url, timeout=5.0)
        except: 
            print("Weather API Connection error or timed-out")
            return False
    

        if response.status_code == 200:

            alpha = response.json()["current"]["wind_degree"]
            #print(alpha)
            wind_speed = response.json()["current"]["wind_kph"]*1000/3600
        
            self.__wind_vector = wind_speed*np.array([np.sin(alpha/180*np.pi), np.cos(alpha/180*np.pi), 0]) # [m/s]
            self.__temperature = response.json()["current"]["temp_c"];              # [%]
            self.__humidity = response.json()["current"]["humidity"];               # [ºC]
            self.__pressure = response.json()["current"]["pressure_mb"]*100;        # [Pa]

            self.__rho_air = air_Density(self)
    
            print("Online Weather loaded at: " + url)

        return True

    # ------------------------ Getting parameter functions ---------------------------
        
    def get_Wind_Vector(self) -> float:
        """
        Gets the wind vector of an exisiting weather status. The length of the vector is the speed of the wind [m/s] and
        the direction is the direction of the wind.
        """

        return self.__wind_vector
    
    def get_Wind_Direction(self) -> float:
        """
        Outputs the wind direction measured respect to the north in degrees (-180º to +180º).
        """

        speed = np.linalg.norm(self.__wind_vector)

        if 0.0 == speed:
            return 0.0

        dir = np.rad2deg(np.arccos(self.__wind_vector[1] / speed))
        if self.__wind_vector[0] < 0: dir = -dir

        return dir
    
    def get_Temperature(self) -> float:
        """
        Gets the temperature (ºC) of an existing weather status
        """

        return self.__temperature 
    
    def get_Humidity(self) -> float:
        """
        Gets the humidity (R%) of an existing weather status
        """

        return self.__humidity 
    
    def get_Pressure(self) -> float:
        """
        Gets the atmospheric pressure (Pa) of an existing weather status
        """
        
        return self.__pressure
    
    def get_Air_Density(self) -> float:
        """
        Gets the air density of an existing weather status
        """
        
        return self.__rho_air
    

# ------------------------ Generic functions ---------------------------

def air_Density(weather:Weather) -> float:
    """
    Computes the air density for an existing weather status.
                        (Celsius Degrees, R%, Pa) -> kg/m^3
    From https://www.omnicalculator.com/physics/air-density#how-to-calculate-the-air-density
    """
    # Vapor pressure [Pa]
    pv = 6.1078*10**(7.5*weather.get_Temperature()/(weather.get_Temperature()+273.3)) * weather.get_Humidity()

    # Air density [kg / m^3]
    return ((weather.get_Pressure() - pv)/(287.058*(weather.get_Temperature()+273.3)) + pv/(461.495*(weather.get_Temperature()+273.3)))


