from modules import files as F

which = "eil51"

F.transform_TSPLIB_File("./files/tsplib_maps/"+which+".vrp", "./files/"+which+".json")