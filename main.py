# Smart Flies Planner
# The UI part can be packaged into an binary using "pyinstaller main.py  --icon="ico.png" --name smart-flies --noconfirm".
#
# TO DO (CRITICAL)
#   - Add limitations for fixed-wings. PATHS ARE DONE. WE NEED ONLY NEED TO IMPLEMENT IT INTO THE SOLVER.
#       * https://docs.deltaquad.com/deltaquad-pro-operations-manual-v2/
#       * https://docs.qgroundcontrol.com/master/en/qgc-user-guide/index.html
#     As of now, no battery of movement model is allowed. I might ask for the allowance to develop it.
#  
# TO DO (MUST)
#   - Delete SUP and SDOWN names. Instead, use T1->T2 and T2->T1 or similar.
#   - Fix Simulator and probably waypoints for cases where a tower is entered and exited twice. This create joint loops 
#       and the orderRoute could just close the base loop without entering the other one first.
#   - In abstract_dynamic_DFJ: Allow user to change the max size for the initial subtour elimination constrains (the one
#       that are added before solving the problem for the first time)
#
#
# TO DO (OPTIONAL)
#   - Add correct UI scaling for smaller than 1080p displays. This might be done using a global parameter
#   - Start using any usable API for the Satellital Map plot
#   - Allow GUI to run while loading a file. This might be done be manually updating the GUI within the function
#   - Allow zoom changes by user
#   - Stablish altitude reference and make it explicit to the user
#   - Implement bateries as a separate entity from UAVs and link then.
#   - In the left dockable add an image.
#   - In the verticalHeader for tableView, put the model of each UAV next to its id as "U1 | M200" or similar
#   - Use the yaml package to output the yaml instead of raw text editing
#   - Put weather group in a subwindow or in a schematic way (SEMIDONE). Careful with the secondary window colors

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
    
