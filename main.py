# Line Inspection Planner (Python Edition)
# The entire thing can be packaged into an binary using "pyinstaller main.py  --icon="ico.png" --name PLI --noconfirm"

# TO DO (CRITICAL)
#   - Finish both solvers by adding the subroute elimination
#   - For both solvers, if Nav. speed ~ Insp. speed, routes might no longer be connected. If  Nav.speed >> Insp. speed, no problem
#       seems to arise. This indicates that an adequate subroute elimination IS needed for both.
#   - If waypoints are computed twice with or without different settings, some are duplicated wps are duplicated or bugged
#
# TO DO (MUST)
#   - Use binary instead and use each bit for a different thing
#   - Fix Bugs for disjoint power lines. For both solvers, this happens if the network has at least two disjoint branches
#       and are distant. For regular, it might have even more bugs.
#   - In regularSolver: Add subroute elimination constrains loop
#   - In regularSolver: All tower to tower are treated as inspection. FIX IT
#   - In regularSolver: What about tower to tower but just a move not inspection
#   - In regularSolver: It might be faster to not duplicate edges to indicate direction and compute
#       both weights matrices directly from just one double edge.
#   - Fix several bugs in the UI:
#        - Status flag not working
#   - requirements.txt or maybe not if pyinstaller is used
#   - Put weather group in a subwindow or in a schematic way (SEMIDONE). Careful with the secondary window colors
#   - Use the json package to output the yaml instead of raw text editing
#   - Fix latlon and UTM components ordering
#   - If sim trajectories are computed once, you need to exec the planner again to compute (you may change a missionsetting
#       that does not need a new solution)
#
# TO DO (OPTIONAL)
#   - Add correct UI scaling for smaller than 1080p displays. This might be done using a global parameter
#   - Start using any usable API for the Satellital Map plot
#   - Allow GUI to run while loading a file. This might be done be manually updating the GUI within the function
#   - Allow zoom changes by user
#   - Stablish altitude reference and make it explicit to the user
#   - Move some functions codes to betters locations
#   - Implement bateries as a separate entity from UAVs and link then.
#   - In the left dockable, only show the individual different models and maybe add an image.
#       In the verticalHeader for tableView, put the model of each UAV next to its id as "U1 | M200" or similar
#   - Does the N axis of UTM point towards North????

import modules.UI as UI
from PyQt6 import QtWidgets, QtGui
import os
import sys

# If running on Windows, tell the OS that this is an 'independent app'
# with its own app_id
if os.name == 'nt':
        import ctypes
        appid = 'usgrvcpli'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(appid)

# We use a def main if __main__ structure so we can avoid global parameters bugs
# and to enable Python to easily detect the main code as a script, not a module.

# Main Code
def main():

        # Create the app process.
        app = QtWidgets.QApplication(sys.argv)

        # Change the default os fontsize to something more suitable.
        app.setFont(QtGui.QFont(app.font().family(), 8)) 

        # Spawns the main app window that contains all the UI.
        ui = UI.MainWindow()
        # If app is closed, exit python
        sys.exit(app.exec())
        

if __name__ == "__main__":
        main()
    
