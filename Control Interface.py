import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import json
import os

class RobotArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("3 DOF Robot Arm Controller")
        self.root.geometry("800x600")
        self.root.configure(bg='#2c3e50')
        
        # Serial connection
        self.serial_connection = None
        self.is_connected = False
        
        # Current positions
        self.current_positions = {'base': 90, 'shoulder': 90, 'elbow': 90}
        
        # Saved positions
        self.saved_positions = self.load_positions()
        
        self.setup_ui()
        self.update_connection_status()
        
    def setup_ui(self):
        # Main frame
        main_frame = tk.Frame(self.root, bg='#2c3e50')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Connection frame
        self.setup_connection_frame(main_frame)
        
        # Control frame
        self.setup_control_frame(main_frame)
        
        # Preset positions frame
        self.setup_preset_frame(main_frame)
        
        # Log frame
        self.setup_log_frame(main_frame)
        
    def setup_connection_frame(self, parent):
        conn_frame = tk.LabelFrame(parent, text="Connection", bg='#34495e', fg='white', font=('Arial', 12, 'bold'))
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Port selection
        tk.Label(conn_frame, text="Port:", bg='#34495e', fg='white').grid(row=0, column=0, padx=5, pady=5, sticky='w')
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        # Refresh ports button
        tk.Button(conn_frame, text="Refresh", command=self.refresh_ports, 
                 bg='#3498db', fg='white', width=8).grid(row=0, column=2, padx=5, pady=5)
        
        # Connect/Disconnect button
        self.connect_btn = tk.Button(conn_frame, text="Connect", command=self.toggle_connection,
                                   bg='#27ae60', fg='white', width=10)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # Status label
        self.status_label = tk.Label(conn_frame, text="Disconnected", bg='#34495e', fg='#e74c3c', font=('Arial', 10, 'bold'))
        self.status_label.grid(row=0, column=4, padx=10, pady=5)
        
        self.refresh_ports()
        
    def setup_control_frame(self, parent):
        control_frame = tk.LabelFrame(parent, text="Joint Control", bg='#34495e', fg='white', font=('Arial', 12, 'bold'))
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Base control
        self.setup_joint_control(control_frame, "Base", 0, 'base')
        
        # Shoulder control
        self.setup_joint_control(control_frame, "Shoulder", 1, 'shoulder')
        
        # Elbow control
        self.setup_joint_control(control_frame, "Elbow", 2, 'elbow')
        
        # Control buttons
        btn_frame = tk.Frame(control_frame, bg='#34495e')
        btn_frame.grid(row=3, column=0, columnspan=4, pady=10)
        
        tk.Button(btn_frame, text="Home Position", command=self.move_to_home,
                 bg='#f39c12', fg='white', width=12).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Stop All", command=self.stop_all,
                 bg='#e74c3c', fg='white', width=12).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Get Positions", command=self.get_current_positions,
                 bg='#9b59b6', fg='white', width=12).pack(side=tk.LEFT, padx=5)
        
    def setup_joint_control(self, parent, name, row, joint_key):
        # Label
        tk.Label(parent, text=f"{name}:", bg='#34495e', fg='white', font=('Arial', 10, 'bold')).grid(
            row=row, column=0, padx=5, pady=5, sticky='w')
        
        # Slider
        slider = tk.Scale(parent, from_=0, to=180, orient=tk.HORIZONTAL, 
                         bg='#34495e', fg='white', highlightbackground='#34495e',
                         length=300, command=lambda val, j=joint_key: self.on_slider_change(j, val))
        slider.set(90)
        slider.grid(row=row, column=1, padx=5, pady=5)
        
        # Position entry
        entry_var = tk.StringVar(value="90")
        entry = tk.Entry(parent, textvariable=entry_var, width=5)
        entry.grid(row=row, column=2, padx=5, pady=5)
        entry.bind('<Return>', lambda e, j=joint_key, v=entry_var: self.on_entry_change(j, v))
        
        # Move button
        tk.Button(parent, text="Move", 
                 command=lambda j=joint_key, v=entry_var: self.move_joint(j, v.get()),
                 bg='#2ecc71', fg='white', width=6).grid(row=row, column=3, padx=5, pady=5)
        
        # Store references
        setattr(self, f"{joint_key}_slider", slider)
        setattr(self, f"{joint_key}_entry", entry_var)
        
    def setup_preset_frame(self, parent):
        preset_frame = tk.LabelFrame(parent, text="Preset Positions", bg='#34495e', fg='white', font=('Arial', 12, 'bold'))
        preset_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Preset buttons frame
        btn_frame = tk.Frame(preset_frame, bg='#34495e')
        btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Save current position
        save_frame = tk.Frame(btn_frame, bg='#34495e')
        save_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        tk.Label(save_frame, text="Name:", bg='#34495e', fg='white').pack(side=tk.LEFT, padx=2)
        self.preset_name_var = tk.StringVar()
        tk.Entry(save_frame, textvariable=self.preset_name_var, width=15).pack(side=tk.LEFT, padx=2)
        tk.Button(save_frame, text="Save Position", command=self.save_current_position,
                 bg='#16a085', fg='white').pack(side=tk.LEFT, padx=5)
        
        # Preset positions list
        self.preset_listbox = tk.Listbox(preset_frame, height=4, bg='#ecf0f1', selectmode=tk.SINGLE)
        self.preset_listbox.pack(fill=tk.X, padx=5, pady=5)
        
        # Preset control buttons
        preset_btn_frame = tk.Frame(preset_frame, bg='#34495e')
        preset_btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Button(preset_btn_frame, text="Move to Selected", command=self.move_to_preset,
                 bg='#3498db', fg='white').pack(side=tk.LEFT, padx=5)
        tk.Button(preset_btn_frame, text="Delete Selected", command=self.delete_preset,
                 bg='#e74c3c', fg='white').pack(side=tk.LEFT, padx=5)
        
        self.update_preset_list()
        
    def setup_log_frame(self, parent):
        log_frame = tk.LabelFrame(parent, text="Communication Log", bg='#34495e', fg='white', font=('Arial', 12, 'bold'))
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, bg='#ecf0f1', fg='#2c3e50')
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
            
    def toggle_connection(self):
        if not self.is_connected:
            self.connect_to_arduino()
        else:
            self.disconnect_from_arduino()
            
    def connect_to_arduino(self):
        try:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
                
            self.serial_connection = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            
            self.is_connected = True
            self.update_connection_status()
            self.log_message(f"Connected to {port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
            self.log_message(f"Connection failed: {str(e)}")
            
    def disconnect_from_arduino(self):
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None
            
        self.is_connected = False
        self.update_connection_status()
        self.log_message("Disconnected")
        
    def update_connection_status(self):
        if self.is_connected:
            self.status_label.config(text="Connected", fg='#27ae60')
            self.connect_btn.config(text="Disconnect", bg='#e74c3c')
        else:
            self.status_label.config(text="Disconnected", fg='#e74c3c')
            self.connect_btn.config(text="Connect", bg='#27ae60')
            
    def send_command(self, command):
        if not self.is_connected or not self.serial_connection:
            messagebox.showwarning("Warning", "Not connected to Arduino")
            return False
            
        try:
            self.serial_connection.write(f"{command}\n".encode())
            self.log_message(f"Sent: {command}")
            return True
        except Exception as e:
            self.log_message(f"Send error: {str(e)}")
            return False
            
    def on_slider_change(self, joint, value):
        getattr(self, f"{joint}_entry").set(value)
        
    def on_entry_change(self, joint, entry_var):
        try:
            value = int(entry_var.get())
            value = max(0, min(180, value))  # Clamp to valid range
            getattr(self, f"{joint}_slider").set(value)
            entry_var.set(str(value))
        except ValueError:
            pass
            
    def move_joint(self, joint, value):
        try:
            angle = int(value)
            angle = max(0, min(180, angle))
            
            command_map = {'base': 'b', 'shoulder': 's', 'elbow': 'e'}
            command = f"{command_map[joint]}{angle}"
            
            if self.send_command(command):
                self.current_positions[joint] = angle
                
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid angle (0-180)")
            
    def move_to_home(self):
        if self.send_command("h"):
            # Update sliders to home position
            for joint in ['base', 'shoulder', 'elbow']:
                getattr(self, f"{joint}_slider").set(90)
                getattr(self, f"{joint}_entry").set("90")
                self.current_positions[joint] = 90
                
    def stop_all(self):
        # Send current positions to stop movement
        for joint, angle in self.current_positions.items():
            command_map = {'base': 'b', 'shoulder': 's', 'elbow': 'e'}
            self.send_command(f"{command_map[joint]}{angle}")
            
    def get_current_positions(self):
        if self.send_command("p"):
            pass  # Arduino will send back current positions
            
    def save_current_position(self):
        name = self.preset_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter a name for the position")
            return
            
        # Get current slider positions
        position = {
            'base': self.base_slider.get(),
            'shoulder': self.shoulder_slider.get(),
            'elbow': self.elbow_slider.get()
        }
        
        self.saved_positions[name] = position
        self.save_positions()
        self.update_preset_list()
        self.preset_name_var.set("")
        self.log_message(f"Saved position: {name}")
        
    def move_to_preset(self):
        selection = self.preset_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a preset position")
            return
            
        name = self.preset_listbox.get(selection[0])
        if name in self.saved_positions:
            position = self.saved_positions[name]
            
            # Update sliders
            for joint, angle in position.items():
                getattr(self, f"{joint}_slider").set(angle)
                getattr(self, f"{joint}_entry").set(str(angle))
                
            # Send commands
            command_map = {'base': 'b', 'shoulder': 's', 'elbow': 'e'}
            for joint, angle in position.items():
                self.send_command(f"{command_map[joint]}{angle}")
                
            self.log_message(f"Moving to preset: {name}")
            
    def delete_preset(self):
        selection = self.preset_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a preset position to delete")
            return
            
        name = self.preset_listbox.get(selection[0])
        if messagebox.askyesno("Confirm", f"Delete preset '{name}'?"):
            del self.saved_positions[name]
            self.save_positions()
            self.update_preset_list()
            self.log_message(f"Deleted preset: {name}")
            
    def update_preset_list(self):
        self.preset_listbox.delete(0, tk.END)
        for name in self.saved_positions.keys():
            self.preset_listbox.insert(tk.END, name)
            
    def load_positions(self):
        try:
            if os.path.exists("robot_positions.json"):
                with open("robot_positions.json", "r") as f:
                    return json.load(f)
        except Exception as e:
            print(f"Error loading positions: {e}")
        return {}
        
    def save_positions(self):
        try:
            with open("robot_positions.json", "w") as f:
                json.dump(self.saved_positions, f, indent=2)
        except Exception as e:
            print(f"Error saving positions: {e}")
            
    def log_message(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
    def on_closing(self):
        if self.is_connected:
            self.disconnect_from_arduino()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = RobotArmController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()