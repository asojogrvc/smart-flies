from modules import solver as SO

route = [("A", "B"), ("C", "D"), ("B", "C"), ("D", "A"), ("G", "H"), ("H", "G"), ("J", "P"), ("P", "J"), ("U", "K"), ("K", "U")]

print(SO.list_Loops(route))