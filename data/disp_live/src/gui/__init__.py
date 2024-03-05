#!/usr/bin/env python3
"""__init__.py """
ACTIVE_CAMERA = 'Camera 1'
button_camera_1_status = (lambda event: "normal" if ACTIVE_CAMERA=="Camera 1" else "disabled")(ACTIVE_CAMERA)
print(button_camera_1_status)