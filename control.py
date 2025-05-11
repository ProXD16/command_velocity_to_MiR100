import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import threading
import csv
import tkinter as tk
from tkinter import ttk

class MiR100ControllerGUI:
    def __init__(self, csv_file):
        rospy.init_node('mir100_trajectory_controller', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback) #Chay Mo Phong
        # rospy.Subscriber('robot_pose', PoseStamped, self.pose_callback) #Chay Thuc Te
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.twist_msg = Twist()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_linear_vel = 0.0
        self.odom_angular_vel = 0.0
        self.cmd_linear_vel = 0.0
        self.cmd_angular_vel = 0.0
        self.position_history = {'x': [], 'y': [], 'theta': []}

        self.velocity_history = {
            'time': [],
            'cmd_linear': [], 
            'cmd_angular': [],
            'odom_linear': [],
            'odom_angular': []
        }
        self.start_time = None
        self.trajectory_data = pd.read_csv(csv_file)
        print(f"Đã đọc {len(self.trajectory_data)} dòng dữ liệu từ file CSV")
        self.is_executing = False
        self.is_closing = False
        self.setup_gui()
    
    def setup_gui(self):
        """Thiết lập giao diện Tkinter"""
        self.root = tk.Tk()
        self.root.title("MiR100 Robot Controller")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        control_frame = ttk.Frame(main_frame, padding="5")
        control_frame.pack(fill=tk.X, pady=5)
        self.start_button = ttk.Button(control_frame, text="Start Trajectory", command=self.start_trajectory)
        self.start_button.pack(side=tk.LEFT, padx=5)
        self.stop_button = ttk.Button(control_frame, text="Stop Trajectory", command=self.stop_trajectory)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        self.stop_button["state"] = "disabled"
        self.save_button = ttk.Button(control_frame, text="Save Trajectory", command=self.save_trajectory)
        self.save_button.pack(side=tk.LEFT, padx=5)
        self.clear_button = ttk.Button(control_frame, text="Clear Plot", command=self.clear_plot)
        self.clear_button.pack(side=tk.LEFT, padx=5)
        info_frame = ttk.LabelFrame(main_frame, text="Robot Information", padding="5")
        info_frame.pack(fill=tk.X, pady=5)
        self.position_label = ttk.Label(info_frame, text="Position: X=0.00 m, Y=0.00 m")
        self.position_label.pack(side=tk.LEFT, padx=10)
        self.orientation_label = ttk.Label(info_frame, text="Orientation: θ=0.00 rad")
        self.orientation_label.pack(side=tk.LEFT, padx=10)
        velocity_frame = ttk.LabelFrame(main_frame, text="Velocity Information", padding="5")
        velocity_frame.pack(fill=tk.X, pady=5)
        self.cmd_vel_label = ttk.Label(velocity_frame, text="Command: Linear=0.00 m/s, Angular=0.00 rad/s")
        self.cmd_vel_label.pack(side=tk.LEFT, padx=10)
        self.odom_vel_label = ttk.Label(velocity_frame, text="Odometry: Linear=0.00 m/s, Angular=0.00 rad/s")
        self.odom_vel_label.pack(side=tk.LEFT, padx=10)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=5)
        trajectory_frame = ttk.Frame(self.notebook)
        self.notebook.add(trajectory_frame, text='Trajectory Plot')
        velocity_plot_frame = ttk.Frame(self.notebook)
        self.notebook.add(velocity_plot_frame, text='Velocity Feedback From Odom')
        self.fig_traj, self.ax_traj = plt.subplots(figsize=(8, 6))
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.set_title('MiR100 Trajectory')
        self.ax_traj.grid(True)
        self.line_traj, = self.ax_traj.plot([], [], 'b-', label='Robot Path')
        self.ax_traj.legend()
        self.ax_traj.set_xlim(0, 20)
        self.ax_traj.set_ylim(0, 20)
        self.canvas_traj = FigureCanvasTkAgg(self.fig_traj, master=trajectory_frame)
        self.canvas_traj.draw()
        self.canvas_traj.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.fig_vel, self.ax_vel = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        self.ax_vel[0].set_ylabel('Linear Velocity (m/s)')
        self.ax_vel[0].set_title('Linear Velocity Comparison')
        self.ax_vel[0].grid(True)
        self.line_cmd_linear, = self.ax_vel[0].plot([], [], 'r-', label='Command')
        self.line_odom_linear, = self.ax_vel[0].plot([], [], 'g-', label='Odometry')
        self.ax_vel[0].legend()
        self.ax_vel[1].set_xlabel('Time (s)')
        self.ax_vel[1].set_ylabel('Angular Velocity (rad/s)')
        self.ax_vel[1].set_title('Angular Velocity Comparison')
        self.ax_vel[1].grid(True)
        self.line_cmd_angular, = self.ax_vel[1].plot([], [], 'r-', label='Command')
        self.line_odom_angular, = self.ax_vel[1].plot([], [], 'g-', label='Odometry')
        self.ax_vel[1].legend()
        self.canvas_vel = FigureCanvasTkAgg(self.fig_vel, master=velocity_plot_frame)
        self.canvas_vel.draw()
        self.canvas_vel.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.status_bar = ttk.Label(self.root, text="Sẵn sàng", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        self.update_gui()
    
    def update_gui(self):
        if not self.is_closing:
            self.position_label.config(text=f"Position: X={self.current_x:.6f} m, Y={self.current_y:.6f} m")
            self.orientation_label.config(text=f"Orientation: θ={self.current_theta:.6f} rad")
            self.cmd_vel_label.config(text=f"Command: Linear={self.cmd_linear_vel:.6f} m/s, Angular={self.cmd_angular_vel:.6f} rad/s")
            self.odom_vel_label.config(text=f"Odometry: Linear={self.odom_linear_vel:.6f} m/s, Angular={self.odom_angular_vel:.6f} rad/s")
            if len(self.position_history['x']) > 1:
                self.line_traj.set_xdata(self.position_history['x'])
                self.line_traj.set_ydata(self.position_history['y'])
                self.ax_traj.relim()
                self.ax_traj.autoscale_view()
                self.canvas_traj.draw_idle()
            if len(self.velocity_history['time']) > 1:
                self.line_cmd_linear.set_xdata(self.velocity_history['time'])
                self.line_cmd_linear.set_ydata(self.velocity_history['cmd_linear'])
                self.line_odom_linear.set_xdata(self.velocity_history['time'])
                self.line_odom_linear.set_ydata(self.velocity_history['odom_linear'])
                self.line_cmd_angular.set_xdata(self.velocity_history['time'])
                self.line_cmd_angular.set_ydata(self.velocity_history['cmd_angular'])
                self.line_odom_angular.set_xdata(self.velocity_history['time'])
                self.line_odom_angular.set_ydata(self.velocity_history['odom_angular'])
                for ax in self.ax_vel:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_vel.draw_idle()
            self.root.after(100, self.update_gui)
    #Chay Mo Phong
    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_theta = yaw
        self.position_history['x'].append(self.current_x)
        self.position_history['y'].append(self.current_y)
        self.position_history['theta'].append(self.current_theta)
    
    #Chay Thuc Te
    # def pose_callback(self,msg):
    #     self.current_x = msg.pose.position.x
    #     self.current_y = msg.pose.position.y
    #     orientation_q = msg.pose.orientation
    #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #     roll, pitch, yaw = euler_from_quaternion(orientation_list)
    #     self.current_theta = yaw
    #     self.position_history['x'].append(self.current_x)
    #     self.position_history['y'].append(self.current_y)
    #     self.position_history['theta'].append(self.current_theta)

    
    def odom_callback(self, msg):
        self.odom_linear_vel = msg.twist.twist.linear.x
        self.odom_angular_vel = msg.twist.twist.angular.z
        if self.start_time is not None:
            current_time = time.time() - self.start_time
            self.velocity_history['time'].append(current_time)
            self.velocity_history['cmd_linear'].append(self.cmd_linear_vel)
            self.velocity_history['cmd_angular'].append(self.cmd_angular_vel)
            self.velocity_history['odom_linear'].append(self.odom_linear_vel)
            self.velocity_history['odom_angular'].append(self.odom_angular_vel)
    
    def start_trajectory(self):
        if not self.is_executing:
            self.is_executing = True
            self.start_button["state"] = "disabled"
            self.stop_button["state"] = "normal"
            self.status_bar.config(text="Đang thực thi quỹ đạo...")
            self.start_time = time.time()
            self.velocity_history = {
                'time': [],
                'cmd_linear': [], 
                'cmd_angular': [],
                'odom_linear': [],
                'odom_angular': []
            }
            self.trajectory_thread = threading.Thread(target=self.execute_trajectory)
            self.trajectory_thread.daemon = True
            self.trajectory_thread.start()
    
    def stop_trajectory(self):
        self.is_executing = False
        self.start_button["state"] = "normal"
        self.stop_button["state"] = "disabled"
        self.status_bar.config(text="Quỹ đạo đã dừng")
        self.start_time = None
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.vel_pub.publish(self.twist_msg)
        self.cmd_linear_vel = 0.0
        self.cmd_angular_vel = 0.0
    
    def execute_trajectory(self):
        rate = rospy.Rate(100)  
        print("Bắt đầu thực thi quỹ đạo...")
        
        # Sử dụng tên cột từ CSV: 'Time', 'Velocity', 'Omega'
        for index, row in self.trajectory_data.iterrows():
            if not self.is_executing or rospy.is_shutdown() or self.is_closing:
                break
                
            target_time = row['Time']  # Sử dụng tên cột 'Time'
            linear_vel = row['Velocity']  # Sử dụng tên cột 'Velocity'
            angular_vel = row['Omega']  # Sử dụng tên cột 'Omega'
            
            self.twist_msg.linear.x = linear_vel
            self.twist_msg.angular.z = angular_vel
            self.cmd_linear_vel = linear_vel
            self.cmd_angular_vel = angular_vel
            self.vel_pub.publish(self.twist_msg)
            
            print("STT: " + str(index))
            print(f"Time: {target_time:.6f}s, Linear vel: {linear_vel:.6f} m/s, Angular vel: {angular_vel:.6f} rad/s")
            print(f"Robot tại vị trí: ({self.current_x:.6f}, {self.current_y:.6f}), hướng: {self.current_theta:.6f}")
            print("-" * 50) 
            rate.sleep()
            
        if self.is_executing:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.vel_pub.publish(self.twist_msg)
            self.cmd_linear_vel = 0.0
            self.cmd_angular_vel = 0.0
            self.root.after(0, lambda: self.status_bar.config(text="Hoàn thành quỹ đạo!"))
            self.root.after(0, lambda: self.start_button.config(state="normal"))
            self.root.after(0, lambda: self.stop_button.config(state="disabled"))
            self.start_time = None
            print("Hoàn thành quỹ đạo!")
        self.is_executing = False
    
    def save_trajectory(self):
        if len(self.position_history['x']) == 0:
            self.status_bar.config(text="Không có dữ liệu để lưu!")
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        traj_filename = f"mir100_trajectory_{timestamp}.csv"
        with open(traj_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'theta'])
            for i in range(len(self.position_history['x'])):
                writer.writerow([
                    self.position_history['x'][i],
                    self.position_history['y'][i],
                    self.position_history['theta'][i]
                ])
        if len(self.velocity_history['time']) > 0:
            vel_filename = f"mir100_velocity_{timestamp}.csv"
            with open(vel_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['time', 'cmd_linear', 'cmd_angular', 'odom_linear', 'odom_angular'])
                for i in range(len(self.velocity_history['time'])):
                    writer.writerow([
                        self.velocity_history['time'][i],
                        self.velocity_history['cmd_linear'][i],
                        self.velocity_history['cmd_angular'][i],
                        self.velocity_history['odom_linear'][i],
                        self.velocity_history['odom_angular'][i]
                    ])
            self.status_bar.config(text=f"Đã lưu dữ liệu quỹ đạo và vận tốc")
            print(f"Đã lưu quỹ đạo thực tế vào file {traj_filename}")
            print(f"Đã lưu dữ liệu vận tốc vào file {vel_filename}")
        else:
            self.status_bar.config(text=f"Đã lưu dữ liệu quỹ đạo")
            print(f"Đã lưu quỹ đạo thực tế vào file {traj_filename}")
    
    def clear_plot(self):
        self.position_history = {'x': [], 'y': [], 'theta': []}
        self.line_traj.set_data([], [])
        self.canvas_traj.draw_idle()
        self.velocity_history = {
            'time': [],
            'cmd_linear': [], 
            'cmd_angular': [],
            'odom_linear': [],
            'odom_angular': []
        }
        self.line_cmd_linear.set_data([], [])
        self.line_odom_linear.set_data([], [])
        self.line_cmd_angular.set_data([], [])
        self.line_odom_angular.set_data([], [])
        self.canvas_vel.draw_idle()
        self.status_bar.config(text="Đã xóa đồ thị")
    
    def on_closing(self):
        if self.is_executing:
            self.stop_trajectory()
        self.is_closing = True
        self.root.destroy()
        rospy.signal_shutdown("Ứng dụng đã đóng")
    
    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    try:
        csv_file = "velocity_omega_profile.csv"
        print(f"Đọc dữ liệu từ file: {csv_file}")
        controller = MiR100ControllerGUI(csv_file)
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Lỗi: {e}")