from modules import solver as SO

route = [('tT5', 'B1'), ('B1', 'tS3_D'), ('tT1', 'tT2'), ('tT2', 'tT1'), ('tT3', 'tT4'), ('tT4', 'tT3'), ('tS1_D', 'tT5'), ('tS3_D', 'tS1_D')]

SO.list_Loops(route)