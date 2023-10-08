#!/usr/bin/env python3
""" client dashboard gui"""

import sys
import os

# Add the path to the backend_gui module to the PYTHONPATH environment variable
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

try:
    from backend_gui import ClientGUI
except ImportError:
    print("Unable to import 'backend_gui' module")

if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
    
