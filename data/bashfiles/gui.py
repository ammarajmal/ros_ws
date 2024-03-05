#!/usr/bin/env python3
"""python gui module for client side"""
import tkinter as tk
import backend as bk # import python module backend.py

# Create the main window
root = tk.Tk()
# root.geometry("300x250")
# root.__setattr__("width", 300)
root.title("Camera Dashboard")  # Set the title of the window

# Create the two frames
camera_frame = tk.Frame(root)
calib_frame = tk.Frame(root, background="gray")

camera_frame.grid(row=0, column=0)
calib_frame.grid(row=1, column=0)


CAMERA_ACTIVE = tk.StringVar()

TEXT = bk.cam_[bk.CAMERA_SELECT - 1]['name'] + ' Output'
print(TEXT)
camera_lbl = tk.Label(camera_frame, text=TEXT)
camera_btn = tk.Button(camera_frame, text="Start Camera View", command=bk.camera_event)


camera_lbl.grid(row=0, column=0, padx=0, pady=(10, 10))
camera_btn.grid(row=1, column=0, padx=0, pady=(10, 10))

# Create the Camera Calibration button inside calib_frame
camera_calibration_btn = tk.Button(calib_frame, text="Start Camera Calibration",
                                      command=bk.camera_calib_event)
camera_calibration_btn.grid(row=0, column=0, padx=10, pady=30)

# Create the Exit button
exit_button = tk.Button(root, text="Exit", command=root.destroy)
exit_button.grid(row=2, column=0, pady=10)


if __name__ == "__main__":
    # Start the main loop
    root.mainloop()
    