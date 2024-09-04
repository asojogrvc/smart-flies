from modules import files as F

wlist = ["att48", "eil22", "eil23", "eil30", "eil33", "eil51"]
for which in wlist:
    F.transform_TSPLIB_File("./files/tsplib_maps/"+which+".vrp", "./files/"+which+".json")