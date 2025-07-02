#!/usr/bin/env python3
"""
شبیه‌ساز ساده کنترل سرعت خط تولید (بدون نیاز به matplotlib)
Simple Line Speed Control Simulator (without matplotlib)
"""

import tkinter as tk
from tkinter import ttk
import time
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
        self.stop_emergency = False
        self.safety_interlock_ok = True
        
        # Parameters
        self.max_speed = 100.0
        self.min_speed = 0.0
        self.ramp_up_time_normal_s = 5.0
        self.ramp_down_time_normal_s = 5.0
        self.ramp_down_time_emergency_s = 2.0
        self.manual_ramp_time_s = 1.0
        self.ramp_step = 1.0
        self.scan_cycle_ms = 100.0
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
        self.manual_setpoint = 0.0
        self.first_scan = True
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
        """اجرای منطق کنترل"""
        
        # Initialization
        if self.first_scan:
            self.manual_setpoint = 0.0
            self.speed_out_mpm = 0.0
            self.previous_speed = 0.0
            self.first_scan = False
        
        # Reset errors
        if self.reset and self.emergency_latched:
            self.emergency_latched = False
        
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
        
        # Calculate and apply speed ramp
        delta = self.target_speed - self.speed_out_mpm
        
        if delta > 0:
            ramp_time = max(self.MIN_RAMP_TIME, self.ramp_up_time_normal_s)
        elif delta < 0:
            ramp_time = max(self.MIN_RAMP_TIME, self.ramp_down_time_normal_s)
        
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
        if self.max_speed > 0:
            self.speed_out_percent = (self.speed_out_mpm / self.max_speed) * 100.0
        else:
            self.speed_out_percent = 0.0
        
        if self.scan_cycle_ms > 0:
            self.acceleration_mps2 = ((self.speed_out_mpm - self.previous_speed) / 60.0) / (self.scan_cycle_ms / 1000.0)
        else:
            self.acceleration_mps2 = 0.0
        
        self.previous_speed = self.speed_out_mpm
        
        # Set status flags
        self.is_running = abs(self.speed_out_mpm) > self.SPEED_ZERO_THRESHOLD
        self.at_setpoint = abs(self.speed_out_mpm - self.target_speed) <= self.speed_tolerance
        self.speed_warning = self.speed_out_percent >= self.warning_threshold
        
        # Runtime calculations
        if self.is_running:
            if self.run_start_time is None:
                self.run_start_time = time.time()
            
            current_time = time.time()
            elapsed_seconds = current_time - self.run_start_time
            self.run_time_hours = elapsed_seconds / 3600.0
            
            self.distance_accumulator += abs(self.speed_out_mpm) * self.scan_cycle_ms / 60000.0
            if self.distance_accumulator >= 1000.0:
                self.total_distance_km += 1.0
                self.distance_accumulator -= 1000.0
        else:
            self.run_start_time = None


class SimpleLineSpeedSimulator:
    """رابط گرافیکی ساده شبیه‌ساز"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("شبیه‌ساز ساده کنترل سرعت خط / Simple Line Speed Simulator")
        self.root.geometry("800x600")
        
        # Controller instance
        self.controller = FB_LineSpeedControl()
        
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
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="شبیه‌ساز کنترل سرعت خط تولید", 
                               font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # Control Panel
        control_frame = ttk.LabelFrame(main_frame, text="کنترل‌ها / Controls", padding="10")
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Enable/Disable
        self.enable_var = tk.BooleanVar()
        ttk.Checkbutton(control_frame, text="فعال‌سازی / Enable", 
                       variable=self.enable_var).pack(anchor=tk.W)
        
        # Mode selection
        mode_frame = ttk.Frame(control_frame)
        mode_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(mode_frame, text="حالت / Mode:").pack(side=tk.LEFT, padx=5)
        self.mode_var = tk.IntVar(value=1)
        ttk.Radiobutton(mode_frame, text="دستی/Manual", variable=self.mode_var, 
                       value=0).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="HMI", variable=self.mode_var, 
                       value=1).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="اتوماتیک/Auto", variable=self.mode_var, 
                       value=2).pack(side=tk.LEFT, padx=5)
        
        # Speed setpoints
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="سرعت HMI:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.hmi_speed_var = tk.DoubleVar(value=50.0)
        self.hmi_speed_scale = ttk.Scale(speed_frame, from_=0, to=150, 
                                        variable=self.hmi_speed_var, orient=tk.HORIZONTAL)
        self.hmi_speed_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        self.hmi_speed_label = ttk.Label(speed_frame, text="50.0 m/min")
        self.hmi_speed_label.grid(row=0, column=2, padx=5)
        
        ttk.Label(speed_frame, text="سرعت Auto:").grid(row=1, column=0, sticky=tk.W, padx=5)
        self.auto_speed_var = tk.DoubleVar(value=75.0)
        self.auto_speed_scale = ttk.Scale(speed_frame, from_=0, to=150, 
                                         variable=self.auto_speed_var, orient=tk.HORIZONTAL)
        self.auto_speed_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5)
        self.auto_speed_label = ttk.Label(speed_frame, text="75.0 m/min")
        self.auto_speed_label.grid(row=1, column=2, padx=5)
        
        speed_frame.columnconfigure(1, weight=1)
        
        # Manual control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(pady=10)
        
        self.speed_up_btn = ttk.Button(button_frame, text="▲ افزایش سرعت")
        self.speed_up_btn.pack(side=tk.LEFT, padx=5)
        
        self.speed_down_btn = ttk.Button(button_frame, text="▼ کاهش سرعت")
        self.speed_down_btn.pack(side=tk.LEFT, padx=5)
        
        # Emergency controls
        emergency_frame = ttk.Frame(control_frame)
        emergency_frame.pack(pady=10)
        
        self.emergency_var = tk.BooleanVar()
        self.emergency_btn = tk.Button(emergency_frame, text="🛑 توقف اضطراری", 
                                      command=self.toggle_emergency, 
                                      bg="white", activebackground="red")
        self.emergency_btn.pack(side=tk.LEFT, padx=5)
        
        self.reset_btn = ttk.Button(emergency_frame, text="🔄 ریست", 
                                   command=self.reset_controller)
        self.reset_btn.pack(side=tk.LEFT, padx=5)
        
        self.safety_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(emergency_frame, text="اینترلاک ایمنی OK", 
                       variable=self.safety_var).pack(side=tk.LEFT, padx=5)
        
        # Status Panel
        status_frame = ttk.LabelFrame(main_frame, text="وضعیت / Status", padding="10")
        status_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Speed gauge (simple)
        gauge_frame = ttk.Frame(status_frame)
        gauge_frame.pack(pady=10)
        
        self.speed_gauge = ttk.Progressbar(gauge_frame, length=400, maximum=100, 
                                          mode='determinate', orient=tk.HORIZONTAL)
        self.speed_gauge.pack()
        
        self.speed_label = ttk.Label(gauge_frame, text="0.0 m/min (0.0%)", 
                                    font=("Arial", 14, "bold"))
        self.speed_label.pack()
        
        # Status grid
        status_grid = ttk.Frame(status_frame)
        status_grid.pack(fill=tk.X, pady=10)
        
        # Left column
        left_frame = ttk.Frame(status_grid)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        ttk.Label(left_frame, text="حالت:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.mode_label = ttk.Label(left_frame, text="---", font=("Arial", 10, "bold"))
        self.mode_label.grid(row=0, column=1, sticky=tk.W, padx=10, pady=2)
        
        ttk.Label(left_frame, text="شتاب:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.accel_label = ttk.Label(left_frame, text="0.0 m/s²", font=("Arial", 10, "bold"))
        self.accel_label.grid(row=1, column=1, sticky=tk.W, padx=10, pady=2)
        
        ttk.Label(left_frame, text="هدف:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.target_label = ttk.Label(left_frame, text="0.0 m/min", font=("Arial", 10, "bold"))
        self.target_label.grid(row=2, column=1, sticky=tk.W, padx=10, pady=2)
        
        # Right column
        right_frame = ttk.Frame(status_grid)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10)
        
        ttk.Label(right_frame, text="زمان کارکرد:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.runtime_label = ttk.Label(right_frame, text="0.00 h", font=("Arial", 10, "bold"))
        self.runtime_label.grid(row=0, column=1, sticky=tk.W, padx=10, pady=2)
        
        ttk.Label(right_frame, text="مسافت:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.distance_label = ttk.Label(right_frame, text="0.00 km", font=("Arial", 10, "bold"))
        self.distance_label.grid(row=1, column=1, sticky=tk.W, padx=10, pady=2)
        
        ttk.Label(right_frame, text="وضعیت:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.status_label = ttk.Label(right_frame, text="متوقف", font=("Arial", 10, "bold"))
        self.status_label.grid(row=2, column=1, sticky=tk.W, padx=10, pady=2)
        
        # Warning indicators
        warning_frame = ttk.Frame(status_frame)
        warning_frame.pack(pady=10)
        
        self.warning_labels = {}
        warnings = [
            ("speed_warning", "⚠️ سرعت بالا"),
            ("ramp_active", "🔄 رمپ فعال"),
            ("emergency_latched", "🚨 قفل اضطراری"),
            ("at_setpoint", "✅ در نقطه تنظیم")
        ]
        
        for key, text in warnings:
            self.warning_labels[key] = ttk.Label(warning_frame, text=text, 
                                                foreground="gray")
            self.warning_labels[key].pack(side=tk.LEFT, padx=10)
        
        # Simulation control
        sim_frame = ttk.Frame(main_frame)
        sim_frame.pack(pady=10)
        
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
        
    def toggle_emergency(self):
        """تغییر وضعیت توقف اضطراری"""
        self.emergency_var.set(not self.emergency_var.get())
        self.controller.stop_emergency = self.emergency_var.get()
        if self.emergency_var.get():
            self.emergency_btn.configure(bg="red")
        else:
            self.emergency_btn.configure(bg="white")
    
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
            
            # Sleep for scan cycle
            time.sleep(self.controller.scan_cycle_ms / 1000.0)
    
    def update_display(self):
        """به‌روزرسانی نمایش"""
        # Update sliders labels
        self.hmi_speed_label.configure(text=f"{self.hmi_speed_var.get():.1f} m/min")
        self.auto_speed_label.configure(text=f"{self.auto_speed_var.get():.1f} m/min")
        
        # Update speed gauge
        self.speed_gauge['value'] = self.controller.speed_out_percent
        self.speed_label.configure(
            text=f"{self.controller.speed_out_mpm:.1f} m/min ({self.controller.speed_out_percent:.1f}%)")
        
        # Update status labels
        self.mode_label.configure(text=self.controller.current_mode)
        self.accel_label.configure(text=f"{self.controller.acceleration_mps2:.2f} m/s²")
        self.target_label.configure(text=f"{self.controller.target_speed:.1f} m/min")
        self.runtime_label.configure(text=f"{self.controller.run_time_hours:.2f} h")
        self.distance_label.configure(text=f"{self.controller.total_distance_km:.2f} km")
        
        # Status text
        if self.controller.is_running:
            status_text = "در حال کار"
            status_color = "green"
        else:
            status_text = "متوقف"
            status_color = "red"
        self.status_label.configure(text=status_text, foreground=status_color)
        
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
        
        # Schedule next update
        self.root.after(100, self.update_display)


def main():
    """تابع اصلی"""
    root = tk.Tk()
    app = SimpleLineSpeedSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()