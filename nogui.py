# No GUI version

import sys

print(sys.argv)

args = sys.argv[1:]

if args[0] == "-help":
    print("hey")

exit()
