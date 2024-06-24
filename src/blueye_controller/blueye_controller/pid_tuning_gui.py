import tkinter as tk
from tkinter import ttk
import json
import os

class PIDTuningGUI:
    PARAMS_FILE = "pid_params.json"

    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("PID Tuning")

        self.params = self.load_params()

        # Create a notebook with tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(expand=True, fill='both')

        # Create frames for homing and transit
        self.homing_frame = ttk.Frame(self.notebook)
        self.transit_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.homing_frame, text='Homing')
        self.notebook.add(self.transit_frame, text='Transit')

        # Create sliders for homing parameters
        self.create_slider(self.homing_frame, "Homing Surge Kp", self.params.get("homing_surge_kp", 0.8), 0.0, 2.0, self.update_homing_surge_kp)
        self.create_slider(self.homing_frame, "Homing Surge Ki", self.params.get("homing_surge_ki", 0.0), 0.0, 2.0, self.update_homing_surge_ki)
        self.create_slider(self.homing_frame, "Homing Surge Kd", self.params.get("homing_surge_kd", 0.25), 0.0, 2.0, self.update_homing_surge_kd)

        self.create_slider(self.homing_frame, "Homing Sway Kp", self.params.get("homing_sway_kp", 0.2), 0.0, 2.0, self.update_homing_sway_kp)
        self.create_slider(self.homing_frame, "Homing Sway Ki", self.params.get("homing_sway_ki", 0.0), 0.0, 2.0, self.update_homing_sway_ki)
        self.create_slider(self.homing_frame, "Homing Sway Kd", self.params.get("homing_sway_kd", 0.01), 0.0, 2.0, self.update_homing_sway_kd)

        self.create_slider(self.homing_frame, "Homing Yaw Kp", self.params.get("homing_yaw_kp", 1.0), 0.0, 2.0, self.update_homing_yaw_kp)
        self.create_slider(self.homing_frame, "Homing Yaw Ki", self.params.get("homing_yaw_ki", 0.01), 0.0, 2.0, self.update_homing_yaw_ki)
        self.create_slider(self.homing_frame, "Homing Yaw Kd", self.params.get("homing_yaw_kd", 2.5), 0.0, 5.0, self.update_homing_yaw_kd)

        # Create sliders for transit parameters
        self.create_slider(self.transit_frame, "Transit Surge Kp", self.params.get("transit_surge_kp", 0.6), 0.0, 100.0, self.update_transit_surge_kp)
        self.create_slider(self.transit_frame, "Transit Surge Ki", self.params.get("transit_surge_ki", 0.0), 0.0, 100.0, self.update_transit_surge_ki)
        self.create_slider(self.transit_frame, "Transit Surge Kd", self.params.get("transit_surge_kd", 0.2), 0.0, 100.0, self.update_transit_surge_kd)

        self.create_slider(self.transit_frame, "Transit Sway Kp", self.params.get("transit_sway_kp", 0.16), 0.0, 100.0, self.update_transit_sway_kp)
        self.create_slider(self.transit_frame, "Transit Sway Ki", self.params.get("transit_sway_ki", 0.0), 0.0, 100.0, self.update_transit_sway_ki)
        self.create_slider(self.transit_frame, "Transit Sway Kd", self.params.get("transit_sway_kd", 0.002), 0.0, 100.0, self.update_transit_sway_kd)

        self.create_slider(self.transit_frame, "Transit Yaw Kp", self.params.get("transit_yaw_kp", 0.75), 0.0, 2.0, self.update_transit_yaw_kp)
        self.create_slider(self.transit_frame, "Transit Yaw Ki", self.params.get("transit_yaw_ki", 0.01), 0.0, 2.0, self.update_transit_yaw_ki)
        self.create_slider(self.transit_frame, "Transit Yaw Kd", self.params.get("transit_yaw_kd", 2.0), 0.0, 5.0, self.update_transit_yaw_kd)

    def create_slider(self, parent, name, initial, min_val, max_val, callback):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        label = ttk.Label(frame, text=name)
        label.pack(side=tk.LEFT, padx=5, pady=5)
        
        slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL)
        slider.set(initial)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

        value_label = ttk.Label(frame, text=f"{initial:.2f}")
        value_label.pack(side=tk.RIGHT, padx=5, pady=5)
        
        slider.configure(command=lambda val, cb=callback, l=value_label: self.update_value(val, cb, l))

    def update_value(self, val, callback, value_label):
        value = float(val)
        value_label.config(text=f"{value:.2f}")
        callback(value)

    def update_homing_surge_kp(self, val):
        self.controller.homing_surge_pid.kp = float(val)
        self.params["homing_surge_kp"] = float(val)
        self.save_params()

    def update_homing_surge_ki(self, val):
        self.controller.homing_surge_pid.ki = float(val)
        self.params["homing_surge_ki"] = float(val)
        self.save_params()

    def update_homing_surge_kd(self, val):
        self.controller.homing_surge_pid.kd = float(val)
        self.params["homing_surge_kd"] = float(val)
        self.save_params()

    def update_homing_sway_kp(self, val):
        self.controller.homing_sway_pid.kp = float(val)
        self.params["homing_sway_kp"] = float(val)
        self.save_params()

    def update_homing_sway_ki(self, val):
        self.controller.homing_sway_pid.ki = float(val)
        self.params["homing_sway_ki"] = float(val)
        self.save_params()

    def update_homing_sway_kd(self, val):
        self.controller.homing_sway_pid.kd = float(val)
        self.params["homing_sway_kd"] = float(val)
        self.save_params()

    def update_homing_yaw_kp(self, val):
        self.controller.homing_yaw_pid.kp = float(val)
        self.params["homing_yaw_kp"] = float(val)
        self.save_params()

    def update_homing_yaw_ki(self, val):
        self.controller.homing_yaw_pid.ki = float(val)
        self.params["homing_yaw_ki"] = float(val)
        self.save_params()

    def update_homing_yaw_kd(self, val):
        self.controller.homing_yaw_pid.kd = float(val)
        self.params["homing_yaw_kd"] = float(val)
        self.save_params()

    def update_transit_surge_kp(self, val):
        self.controller.transit_surge_pid.kp = float(val)
        self.params["transit_surge_kp"] = float(val)
        self.save_params()

    def update_transit_surge_ki(self, val):
        self.controller.transit_surge_pid.ki = float(val)
        self.params["transit_surge_ki"] = float(val)
        self.save_params()

    def update_transit_surge_kd(self, val):
        self.controller.transit_surge_pid.kd = float(val)
        self.params["transit_surge_kd"] = float(val)
        self.save_params()

    def update_transit_sway_kp(self, val):
        self.controller.transit_sway_pid.kp = float(val)
        self.params["transit_sway_kp"] = float(val)
        self.save_params()

    def update_transit_sway_ki(self, val):
        self.controller.transit_sway_pid.ki = float(val)
        self.params["transit_sway_ki"] = float(val)
        self.save_params()

    def update_transit_sway_kd(self, val):
        self.controller.transit_sway_pid.kd = float(val)
        self.params["transit_sway_kd"] = float(val)
        self.save_params()

    def update_transit_yaw_kp(self, val):
        self.controller.transit_yaw_pid.kp = float(val)
        self.params["transit_yaw_kp"] = float(val)
        self.save_params()

    def update_transit_yaw_ki(self, val):
        self.controller.transit_yaw_pid.ki = float(val)
        self.params["transit_yaw_ki"] = float(val)
        self.save_params()

    def update_transit_yaw_kd(self, val):
        self.controller.transit_yaw_pid.kd = float(val)
        self.params["transit_yaw_kd"] = float(val)
        self.save_params()

    def save_params(self):
        with open(self.PARAMS_FILE, 'w') as f:
            json.dump(self.params, f)

    def load_params(self):
        if os.path.exists(self.PARAMS_FILE):
            with open(self.PARAMS_FILE, 'r') as f:
                return json.load(f)
        else:
            return {}

    def run(self):
        self.root.mainloop()
