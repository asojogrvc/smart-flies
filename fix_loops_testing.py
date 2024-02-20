from modules import solver as SO

route =  [('B1', 'T4'), ('T4', 'T6'), ('T6', 'T10'), ('T10', 'T3'), ('T3', 'T1'), ('T1', 'B1'),
       ('T8', 'T4'), ('T4', 'T7'), ('T7', 'T11'), ('T11', 'T8'),
       ('T8','T11'), ('T11', 'T12'), ('T12', 'T8')]

modes = ["N", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "N"]

print(len(route))
print(len(modes))

route, modes = SO.fix_Route_Valid_Subloops(route, modes)

print(route, modes)
