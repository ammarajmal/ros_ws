import tkinter as tk
import cv2
import customtkinter
from PIL import Image, ImageTk

customtkinter.set_appearance_mode("System")
customtkinter.set_default_color_theme("blue")

class ClientGUI(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        width = 800
        height = 600
        self.title("Camera Dashboard")
        self.geometry(f"{width}x{height}")

        self.grid_rowconfigure((0, 1, 2), weight=1)
        self.grid_columnconfigure(( 1), weight=0)

        # Sidebar frame and components
        self.sidebar_frame = customtkinter.CTkFrame(self, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=7, sticky="nsew")

        self.sidebar_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Selection", font=customtkinter.CTkFont(size=14, weight="normal"))
        self.sidebar_label.grid(row=0, column=0, padx=20, pady=(10, 10))

        self.sidebar_cam_view_btn = customtkinter.CTkButton(
            self.sidebar_frame, command=self.sidebar_button_event, text="View Camera")
        self.sidebar_cam_view_btn.grid(row=1, column=0, padx=20, pady=(10, 10))

        self.sidebar_cam_calib_btn = customtkinter.CTkButton(
            self.sidebar_frame, command=self.sidebar_button_event, text="Calibrate Camera")
        self.sidebar_cam_calib_btn.grid(row=2, column=0, padx=20, pady=(10, 10))

        self.appearance_mode_label = customtkinter.CTkLabel(
            self.sidebar_frame, text="Appearance Mode:")
        self.appearance_mode_label.grid(row=3, column=0, padx=20, pady=(10, 0))

        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            self.sidebar_frame, values=["Light", "Dark", "System"], command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=4, column=0, padx=20, pady=(10, 10))
        self.appearance_mode_optionemenu.set("Dark")

        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:")
        self.scaling_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(
            self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"], command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_optionemenu.set("100%")

        # Main Frame and components
        self.tabview = customtkinter.CTkTabview(self)
        self.tabview.add("Camera Feed")
        self.tabview.add("Camera Calibration")
        self.tabview.grid(row=0, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew")

        self.camera_view_canvas = customtkinter.CTkCanvas(self.tabview.tab("Camera Feed"), width=640, height=480)
        self.camera_view_canvas.grid(row=0, column=0, padx=20, pady=20)

        self.start_camera_btn = customtkinter.CTkButton(
            self.tabview.tab("Camera Feed"), text="Start Camera Display", command=self.start_camera_display)
        self.start_camera_btn.grid(row=1, column=0, padx=20, pady=20)

        self.label_tab_2 = customtkinter.CTkLabel(
            self.tabview.tab("Camera Calibration"), text="CTkLabel on Camera Calibration")
        self.label_tab_2.grid(row=0, column=0, padx=20, pady=20)

        self.exit_button = customtkinter.CTkButton(
            master=self, text="EXIT", command=self.exit_btn_event, fg_color="transparent", border_width=2, text_color=("gray10", "#DCE4EE"))
        self.exit_button.grid(row=1, column=1, padx=(20, 0), pady=(20, 20), sticky="ew")

        self.camera_running = False
        self.photo = None
        self.image_label = tk.Label(self.tabview.tab("Camera Feed"))
        self.image_label.grid(row=0, column=0, padx=20, pady=20)

    def start_camera_display(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            return

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open the camera.")
            return

        self.start_camera_btn.configure(text="Stop Camera Display", command=self.stop_camera_display)
        self.camera_running = True  
        self.update_camera_view()  

    def update_camera_view(self):
        if self.camera_running and hasattr(self, "cap") and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.stop_camera_display()
                return

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame))
            self.image_label.configure(image=self.photo)
            self.after(10, self.update_camera_view)  

    def stop_camera_display(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        self.camera_running = False  
        self.start_camera_btn.configure(text="Start Camera Display", command=self.start_camera_display)

    def sidebar_button_event(self):
        print("sidebar_button click")

    def exit_btn_event(self):
        print("exit_button click")
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        self.destroy()

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)

if __name__ == "__main__":
    app = ClientGUI()
    app.mainloop()
