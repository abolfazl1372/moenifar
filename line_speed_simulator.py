#!/usr/bin/env python3
"""
شبیه‌ساز کنترل سرعت خط تولید / Line Speed Control Simulator
برای تست عملکرد FB_LineSpeedControl
"""

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from datetime import datetime
import time
from collections import deque
import threading

class FB_LineSpeedControl:
    """شبیه‌سازی فانکشن‌بلاک کنترل سرعت خط"""
    
    # Constants
    MODE_MANUAL = 0
    MODE_HMI = 1
    MODE_AUTO = 2
    MIN_RAMP_TIME = 0.1
    SPEED_ZERO_THRESHOLD = 0.1
    
    def __init__(self):
        # Inputs
        self.enable = False
        self.reset = False
        self.mode = self.MODE_HMI
        self.setpoint_hmi = 50.0
        self.auto_setpoint = 75.0
        self.speed_up = False
        self.speed_down = False
        self.jog_forward = False
        self.jog_reverse = False
        self.stop_emergency = False
        self.safety_interlock_ok = True
        
        # Parameters
        self.max_speed = 100.0
        self.min_speed = 0.0
        self.jog_speed = 5.0
        self.ramp_up_time_normal_s = 5.0
        self.ramp_down_time_normal_s = 5.0
        self.ramp_down_time_emergency_s = 2.0
        self.manual_ramp_time_s = 1.0
        self.ramp_step = 1.0
        self.scan_cycle_ms = 100.0  # 100ms for simulation
        self.speed_tolerance = 0.5
        self.warning_threshold = 90.0
        
        # Outputs
        self.speed_out_mpm = 0.0
        self.speed_out_percent = 0.0
        self.acceleration_mps2 = 0.0
        self.is_running = False
        self.at_setpoint = False
        self.emergency_latched = False
        self.speed_warning = False
        self.ramp_active = False
        self.current_mode = "DISABLED"
        self.run_time_hours = 0.0
        self.total_distance_km = 0.0
        
        # Internal variables
        self.target_speed = 0.0
        self.previous_speed = 0.0
        self.speed_before_stop = 0.0
        self.manual_setpoint = 0.0
        self.first_scan = True
        self.mode_changed = False
        self.previous_mode = self.mode
        self.run_start_time = None
        self.distance_accumulator = 0.0
        
    def safe_divide(self, numerator, denominator):
        """تقسیم ایمن برای جلوگیری از خطای تقسیم بر صفر"""
        if abs(denominator) < 0.0001:
            return 0.0
        return numerator / denominator
    
    def limit(self, min_val, value, max_val):
        """محدود کردن مقدار بین حد پایین و بالا"""
        return max(min_val, min(value, max_val))
    
    def execute(self):
        """اجرای منطق کنترل - فراخوانی در هر سیکل"""
        
        # Initialization
        if self.first_scan:
            self.manual_setpoint = 0.0
            self.speed_out_mpm = 0.0
            self.previous_speed = 0.0
            self.first_scan = False
        
        # Reset errors
        if self.reset and self.emergency_latched:
            self.emergency_latched = False
            self.speed_before_stop = 0.0
        
        # Check enable and safety interlock
        if not self.enable or not self.safety_interlock_ok:
            if self.speed_out_mpm > self.SPEED_ZERO_THRESHOLD:
                self.speed_out_mpm -= self.safe_divide(self.speed_out_mpm, 
                    self.ramp_down_time_normal_s * 1000.0 / self.scan_cycle_ms)
            else:
                self.speed_out_mpm = 0.0
            self.is_running = False
            self.current_mode = "DISABLED"
            return
        
        # Emergency stop management
        if self.stop_emergency:
            if not self.emergency_latched and self.speed_out_mpm > self.SPEED_ZERO_THRESHOLD:
                self.speed_before_stop = self.speed_out_mpm
            
            self.speed_out_mpm -= self.safe_divide(self.speed_out_mpm,
                self.ramp_down_time_emergency_s * 1000.0 / self.scan_cycle_ms)
            
            if self.speed_out_mpm < self.SPEED_ZERO_THRESHOLD:
                self.speed_out_mpm = 0.0
                self.emergency_latched = True
                self.is_running = False
            
            self.current_mode = "EMERGENCY STOP"
            return
        
        # Check emergency latch
        if self.emergency_latched:
            self.speed_out_mpm = 0.0
            self.is_running = False
            self.current_mode = "E-STOP LATCHED"
            return
        
        # Jog management
        if self.jog_forward or self.jog_reverse:
            if self.jog_forward and not self.jog_reverse:
                self.target_speed = self.jog_speed
                self.current_mode = "JOG FORWARD"
            elif self.jog_reverse and not self.jog_forward:
                self.target_speed = -self.jog_speed
                self.current_mode = "JOG REVERSE"
            else:
                self.target_speed = 0.0
                self.current_mode = "JOG CONFLICT"
            ramp_time = self.manual_ramp_time_s
        else:
            # Mode change detection
            if self.mode != self.previous_mode:
                self.mode_changed = True
                self.previous_mode = self.mode
            else:
                self.mode_changed = False
            
            # Auto switch to manual with button press
            if (self.speed_up or self.speed_down) and self.mode != self.MODE_MANUAL:
                self.mode = self.MODE_MANUAL
                self.manual_setpoint = self.speed_out_mpm
                self.mode_changed = True
            
            # Process different modes
            if self.mode == self.MODE_MANUAL:
                if self.speed_up and not self.speed_down:
                    self.manual_setpoint += self.ramp_step
                elif self.speed_down and not self.speed_up:
                    self.manual_setpoint -= self.ramp_step
                
                self.manual_setpoint = self.limit(self.min_speed, self.manual_setpoint, self.max_speed)
                self.target_speed = self.manual_setpoint
                ramp_time = self.manual_ramp_time_s
                self.current_mode = "MANUAL"
                
            elif self.mode == self.MODE_HMI:
                self.target_speed = self.limit(self.min_speed, self.setpoint_hmi, self.max_speed)
                ramp_time = self.ramp_up_time_normal_s
                self.current_mode = "HMI CONTROL"
                
            elif self.mode == self.MODE_AUTO:
                self.target_speed = self.limit(self.min_speed, self.auto_setpoint, self.max_speed)
                ramp_time = self.ramp_up_time_normal_s
                self.current_mode = "AUTOMATIC"
            else:
                self.target_speed = 0.0
                self.current_mode = "INVALID MODE"
                ramp_time = self.ramp_up_time_normal_s
        
        # Calculate and apply speed ramp
        delta = self.target_speed - self.speed_out_mpm
        
        # Select ramp time based on direction
        if delta > 0:
            ramp_time = max(self.MIN_RAMP_TIME, self.ramp_up_time_normal_s)
        elif delta < 0:
            ramp_time = max(self.MIN_RAMP_TIME, self.ramp_down_time_normal_s)
        
        # Calculate ramp step
        if abs(delta) > self.speed_tolerance:
            step = self.safe_divide(delta, ramp_time * 1000.0 / self.scan_cycle_ms)
            self.speed_out_mpm += step
            self.ramp_active = True
        else:
            self.speed_out_mpm = self.target_speed
            self.ramp_active = False
        
        # Apply final limits
        self.speed_out_mpm = self.limit(self.min_speed, self.speed_out_mpm, self.max_speed)
        
        # Output calculations
        # Speed percentage
        if self.max_speed > 0:
            self.speed_out_percent = (self.speed_out_mpm / self.max_speed) * 100.0
        else:
            self.speed_out_percent = 0.0
        
        # Acceleration
        if self.scan_cycle_ms > 0:
            self.acceleration_mps2 = ((self.speed_out_mpm - self.previous_speed) / 60.0) / (self.scan_cycle_ms / 1000.0)
        else:
            self.acceleration_mps2 = 0.0
        
        self.previous_speed = self.speed_out_mpm
        
        # Set status flags
        self.is_running = abs(self.speed_out_mpm) > self.SPEED_ZERO_THRESHOLD
        self.at_setpoint = abs(self.speed_out_mpm - self.target_speed) <= self.speed_tolerance
        self.speed_warning = self.speed_out_percent >= self.warning_threshold
        
        # Runtime and distance calculations
        if self.is_running:
            if self.run_start_time is None:
                self.run_start_time = time.time()
            
            # Update runtime
            current_time = time.time()
            elapsed_seconds = current_time - self.run_start_time
            self.run_time_hours = elapsed_seconds / 3600.0
            
            # Calculate distance
            self.distance_accumulator += abs(self.speed_out_mpm) * self.scan_cycle_ms / 60000.0
            if self.distance_accumulator >= 1000.0:
                self.total_distance_km += 1.0
                self.distance_accumulator -= 1000.0
        else:
            self.run_start_time = None


class LineSpeedSimulator:
    """رابط گرافیکی شبیه‌ساز"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("شبیه‌ساز کنترل سرعت خط تولید / Line Speed Control Simulator")
        self.root.geometry("1200x800")
        
        # Controller instance
        self.controller = FB_LineSpeedControl()
        
        # Data for plotting
        self.time_data = deque(maxlen=100)
        self.speed_data = deque(maxlen=100)
        self.setpoint_data = deque(maxlen=100)
        self.acceleration_data = deque(maxlen=100)
        self.start_time = time.time()
        
        # Simulation control
        self.running = False
        self.simulation_thread = None
        
        # Create UI
        self.create_widgets()
        
        # Start update loop
        self.update_display()
        
    def create_widgets(self):
        """ایجاد رابط کاربری"""
        
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Control Panel
        control_frame = ttk.LabelFrame(main_frame, text="کنترل‌ها / Controls", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # Enable/Disable
        self.enable_var = tk.BooleanVar()
        ttk.Checkbutton(control_frame, text="فعال‌سازی / Enable", 
                       variable=self.enable_var).grid(row=0, column=0, sticky=tk.W)
        
        # Mode selection
        ttk.Label(control_frame, text="حالت / Mode:").grid(row=1, column=0, sticky=tk.W)
        self.mode_var = tk.IntVar(value=1)
        ttk.Radiobutton(control_frame, text="دستی/Manual", variable=self.mode_var, 
                       value=0).grid(row=1, column=1, sticky=tk.W)
        ttk.Radiobutton(control_frame, text="HMI", variable=self.mode_var, 
                       value=1).grid(row=1, column=2, sticky=tk.W)
        ttk.Radiobutton(control_frame, text="اتوماتیک/Auto", variable=self.mode_var, 
                       value=2).grid(row=1, column=3, sticky=tk.W)
        
        # Speed setpoints
        ttk.Label(control_frame, text="سرعت HMI:").grid(row=2, column=0, sticky=tk.W)
        self.hmi_speed_var = tk.DoubleVar(value=50.0)
        self.hmi_speed_scale = ttk.Scale(control_frame, from_=0, to=150, 
                                        variable=self.hmi_speed_var, orient=tk.HORIZONTAL)
        self.hmi_speed_scale.grid(row=2, column=1, columnspan=2, sticky=(tk.W, tk.E))
        self.hmi_speed_label = ttk.Label(control_frame, text="50.0 m/min")
        self.hmi_speed_label.grid(row=2, column=3)
        
        ttk.Label(control_frame, text="سرعت اتوماتیک:").grid(row=3, column=0, sticky=tk.W)
        self.auto_speed_var = tk.DoubleVar(value=75.0)
        self.auto_speed_scale = ttk.Scale(control_frame, from_=0, to=150, 
                                         variable=self.auto_speed_var, orient=tk.HORIZONTAL)
        self.auto_speed_scale.grid(row=3, column=1, columnspan=2, sticky=(tk.W, tk.E))
        self.auto_speed_label = ttk.Label(control_frame, text="75.0 m/min")
        self.auto_speed_label.grid(row=3, column=3)
        
        # Manual control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=4, column=0, columnspan=4, pady=10)
        
        self.speed_up_btn = ttk.Button(button_frame, text="▲ افزایش سرعت")
        self.speed_up_btn.grid(row=0, column=0, padx=5)
        
        self.speed_down_btn = ttk.Button(button_frame, text="▼ کاهش سرعت")
        self.speed_down_btn.grid(row=0, column=1, padx=5)
        
        self.jog_fwd_btn = ttk.Button(button_frame, text="► Jog جلو")
        self.jog_fwd_btn.grid(row=0, column=2, padx=5)
        
        self.jog_rev_btn = ttk.Button(button_frame, text="◄ Jog عقب")
        self.jog_rev_btn.grid(row=0, column=3, padx=5)
        
        # Emergency controls
        emergency_frame = ttk.Frame(control_frame)
        emergency_frame.grid(row=5, column=0, columnspan=4, pady=10)
        
        self.emergency_var = tk.BooleanVar()
        self.emergency_btn = ttk.Button(emergency_frame, text="🛑 توقف اضطراری", 
                                       command=self.toggle_emergency)
        self.emergency_btn.grid(row=0, column=0, padx=5)
        
        self.reset_btn = ttk.Button(emergency_frame, text="🔄 ریست", 
                                   command=self.reset_controller)
        self.reset_btn.grid(row=0, column=1, padx=5)
        
        # Safety interlock
        self.safety_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(emergency_frame, text="اینترلاک ایمنی / Safety OK", 
                       variable=self.safety_var).grid(row=0, column=2, padx=5)
        
        # Status Panel
        status_frame = ttk.LabelFrame(main_frame, text="وضعیت / Status", padding="10")
        status_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # Status displays
        self.status_labels = {}
        status_items = [
            ("سرعت فعلی / Current Speed:", "speed"),
            ("درصد سرعت / Speed %:", "percent"),
            ("شتاب / Acceleration:", "acceleration"),
            ("حالت عملیاتی / Mode:", "mode"),
            ("وضعیت / Status:", "status"),
            ("زمان کارکرد / Runtime:", "runtime"),
            ("مسافت / Distance:", "distance")
        ]
        
        for i, (label, key) in enumerate(status_items):
            ttk.Label(status_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=2)
            self.status_labels[key] = ttk.Label(status_frame, text="---", 
                                               font=("Arial", 10, "bold"))
            self.status_labels[key].grid(row=i, column=1, sticky=tk.W, pady=2)
        
        # Warning indicators
        self.warning_frame = ttk.Frame(status_frame)
        self.warning_frame.grid(row=len(status_items), column=0, columnspan=2, pady=10)
        
        self.warning_labels = {}
        warnings = [
            ("speed_warning", "⚠️ هشدار سرعت بالا"),
            ("ramp_active", "🔄 رمپ فعال"),
            ("emergency_latched", "🚨 قفل اضطراری"),
            ("at_setpoint", "✅ در نقطه تنظیم")
        ]
        
        for key, text in warnings:
            self.warning_labels[key] = ttk.Label(self.warning_frame, text=text, 
                                                foreground="gray")
            self.warning_labels[key].pack(side=tk.LEFT, padx=5)
        
        # Graph
        graph_frame = ttk.LabelFrame(main_frame, text="نمودار سرعت / Speed Graph", padding="10")
        graph_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), 
                         padx=5, pady=5)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel('زمان / Time (s)')
        self.ax.set_ylabel('سرعت / Speed (m/min)')
        self.ax.set_ylim(-10, 160)
        self.ax.grid(True, alpha=0.3)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Simulation control
        sim_frame = ttk.Frame(main_frame)
        sim_frame.grid(row=2, column=0, columnspan=2, pady=10)
        
        self.start_btn = ttk.Button(sim_frame, text="▶️ شروع شبیه‌سازی", 
                                   command=self.start_simulation)
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(sim_frame, text="⏸️ توقف شبیه‌سازی", 
                                  command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Bind button events
        self.speed_up_btn.bind("<ButtonPress-1>", lambda e: setattr(self.controller, 'speed_up', True))
        self.speed_up_btn.bind("<ButtonRelease-1>", lambda e: setattr(self.controller, 'speed_up', False))
        self.speed_down_btn.bind("<ButtonPress-1>", lambda e: setattr(self.controller, 'speed_down', True))
        self.speed_down_btn.bind("<ButtonRelease-1>", lambda e: setattr(self.controller, 'speed_down', False))
        self.jog_fwd_btn.bind("<ButtonPress-1>", lambda e: setattr(self.controller, 'jog_forward', True))
        self.jog_fwd_btn.bind("<ButtonRelease-1>", lambda e: setattr(self.controller, 'jog_forward', False))
        self.jog_rev_btn.bind("<ButtonPress-1>", lambda e: setattr(self.controller, 'jog_reverse', True))
        self.jog_rev_btn.bind("<ButtonRelease-1>", lambda e: setattr(self.controller, 'jog_reverse', False))
        
    def toggle_emergency(self):
        """تغییر وضعیت توقف اضطراری"""
        self.emergency_var.set(not self.emergency_var.get())
        self.controller.stop_emergency = self.emergency_var.get()
        if self.emergency_var.get():
            self.emergency_btn.configure(style="Emergency.TButton")
        else:
            self.emergency_btn.configure(style="TButton")
    
    def reset_controller(self):
        """ریست کنترلر"""
        self.controller.reset = True
        self.root.after(100, lambda: setattr(self.controller, 'reset', False))
    
    def start_simulation(self):
        """شروع شبیه‌سازی"""
        self.running = True
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        
        # Start simulation thread
        self.simulation_thread = threading.Thread(target=self.simulation_loop)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
    
    def stop_simulation(self):
        """توقف شبیه‌سازی"""
        self.running = False
        self.start_btn.configure(state=tk.NORMAL)
        self.stop_btn.configure(state=tk.DISABLED)
    
    def simulation_loop(self):
        """حلقه اصلی شبیه‌سازی"""
        while self.running:
            # Update controller inputs
            self.controller.enable = self.enable_var.get()
            self.controller.mode = self.mode_var.get()
            self.controller.setpoint_hmi = self.hmi_speed_var.get()
            self.controller.auto_setpoint = self.auto_speed_var.get()
            self.controller.safety_interlock_ok = self.safety_var.get()
            
            # Execute controller
            self.controller.execute()
            
            # Store data for plotting
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            self.speed_data.append(self.controller.speed_out_mpm)
            self.setpoint_data.append(self.controller.target_speed)
            self.acceleration_data.append(self.controller.acceleration_mps2)
            
            # Sleep for scan cycle
            time.sleep(self.controller.scan_cycle_ms / 1000.0)
    
    def update_display(self):
        """به‌روزرسانی نمایش"""
        # Update sliders labels
        self.hmi_speed_label.configure(text=f"{self.hmi_speed_var.get():.1f} m/min")
        self.auto_speed_label.configure(text=f"{self.auto_speed_var.get():.1f} m/min")
        
        # Update status displays
        self.status_labels["speed"].configure(
            text=f"{self.controller.speed_out_mpm:.1f} m/min")
        self.status_labels["percent"].configure(
            text=f"{self.controller.speed_out_percent:.1f}%")
        self.status_labels["acceleration"].configure(
            text=f"{self.controller.acceleration_mps2:.2f} m/s²")
        self.status_labels["mode"].configure(text=self.controller.current_mode)
        
        # Status text
        if self.controller.is_running:
            status_text = "در حال کار / Running"
            status_color = "green"
        else:
            status_text = "متوقف / Stopped"
            status_color = "red"
        self.status_labels["status"].configure(text=status_text, foreground=status_color)
        
        self.status_labels["runtime"].configure(
            text=f"{self.controller.run_time_hours:.2f} h")
        self.status_labels["distance"].configure(
            text=f"{self.controller.total_distance_km:.2f} km")
        
        # Update warnings
        warnings_map = {
            "speed_warning": (self.controller.speed_warning, "red"),
            "ramp_active": (self.controller.ramp_active, "orange"),
            "emergency_latched": (self.controller.emergency_latched, "red"),
            "at_setpoint": (self.controller.at_setpoint, "green")
        }
        
        for key, (condition, color) in warnings_map.items():
            if condition:
                self.warning_labels[key].configure(foreground=color)
            else:
                self.warning_labels[key].configure(foreground="gray")
        
        # Update graph
        if self.running and len(self.time_data) > 0:
            self.ax.clear()
            self.ax.plot(self.time_data, self.speed_data, 'b-', label='سرعت واقعی / Actual')
            self.ax.plot(self.time_data, self.setpoint_data, 'r--', label='نقطه تنظیم / Setpoint')
            self.ax.set_xlabel('زمان / Time (s)')
            self.ax.set_ylabel('سرعت / Speed (m/min)')
            self.ax.set_ylim(-10, max(160, max(self.speed_data) * 1.1) if self.speed_data else 160)
            self.ax.grid(True, alpha=0.3)
            self.ax.legend()
            self.canvas.draw()
        
        # Schedule next update
        self.root.after(100, self.update_display)


def main():
    """تابع اصلی"""
    root = tk.Tk()
    
    # Configure style
    style = ttk.Style()
    style.configure("Emergency.TButton", foreground="red")
    
    # Create and run simulator
    app = LineSpeedSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()