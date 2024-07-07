import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

# Global variables to store animation, canvas widget, last position, and last orientation
anim_global = None
canvas_widget = None
last_position = None
last_orientation = None
waypoints = []

def differential_drive_simulation(left_speed, left_radius, right_speed, right_radius, body_radius, initial_x, initial_y, initial_orientation, waypoints):
    # Convert angles to radians
    initial_orientation_rad = np.radians(initial_orientation)

    # Initialize position and orientation
    x = initial_x
    y = initial_y
    theta = initial_orientation_rad

    # If the last position and orientation are stored, use them for initialization
    if last_position is not None and last_orientation is not None:
        x = last_position[0]
        y = last_position[1]
        theta = np.radians(last_orientation)

    # Initialize position lists for plotting the path
    x_plot = [x]
    y_plot = [y]

    # Simulation time
    total_time = 10  # e.g., the simulation runs for 10 seconds
    dt = 0.1  # Time interval
    times = np.arange(0, total_time, dt)

    # Plotting
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='datalim')
    ax.invert_yaxis()  # Invert y-axis to match common coordinate system

    # Add fixed grid
    ax.set_xticks(np.arange(0, 21, 1))
    ax.set_yticks(np.arange(0, 21, 1))
    ax.grid(True)

    # Plot static lines between initial position and waypoints
    points = [(x, y)] + [(wx, wy) for wx, wy, wpsi in waypoints]
    for i in range(len(points) - 1):
        ax.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], 'k--')  # Dashed line

    # Function to update the robot's position and orientation
    def update(frame):
        nonlocal x, y, theta, left_speed, right_speed

        if not waypoints:
            return  # No waypoints to follow

        # Get the current target waypoint
        target_x, target_y, target_psi = waypoints[0]
        target_psi_rad = np.radians(target_psi)

        # Simple proportional controller for steering towards the target
        K_linear = 1.0  # Linear speed gain
        K_angular = 4.0  # Angular speed gain

        # Compute the error in position
        dx_target = target_x - x
        dy_target = target_y - y

        # Compute the distance and angle to the target
        distance_to_target = np.sqrt(dx_target**2 + dy_target**2)
        angle_to_target = np.arctan2(dy_target, dx_target)

        # Compute the error in orientation
        angle_error = angle_to_target - theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize the angle error

        # Control law
        v = K_linear * distance_to_target
        omega = K_angular * angle_error

        # Calculate the translational speed (linear) of each wheel
        v_left = v - omega * body_radius / 2
        v_right = v + omega * body_radius / 2

        # Convert wheel speeds to angular speeds
        left_speed = v_left / left_radius
        right_speed = v_right / right_radius

        # Calculate the change in position and orientation
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = omega * dt

        # Update position and orientation
        x += dx
        y += dy
        theta += dtheta

        # Check if the waypoint has been reached
        if distance_to_target < 0.1:
            waypoints.pop(0)  # Remove the reached waypoint

        # Store position for plotting the path
        x_plot.append(x)
        y_plot.append(y)

        # Plot the robot's position and path
        ax.clear()
        ax.plot(x_plot, y_plot, 'b-')

        # Draw static lines again (as the axes are cleared on each update)
        for i in range(len(points) - 1):
            ax.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], 'k--')

        # Draw a square representing the robot
        robot_size = 0.5  # Size of the robot square
        corners = np.array([
            [-robot_size / 2, -robot_size / 2],
            [robot_size / 2, -robot_size / 2],
            [robot_size / 2, robot_size / 2],
            [-robot_size / 2, robot_size / 2],
            [-robot_size / 2, -robot_size / 2]
        ])

        # Rotate and translate the square according to the robot's position and orientation
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        transformed_corners = np.dot(corners, rotation_matrix.T) + np.array([x, y])

        # Draw the square
        ax.plot(transformed_corners[:, 0], transformed_corners[:, 1], 'r-')

        # Draw the waypoints
        for wx, wy, wpsi in waypoints:
            ax.plot(wx, wy, 'go')  # Green dot for the waypoint position
            waypoint_orientation_arrow_length = 1.0
            wpsi_rad = np.radians(wpsi)
            ax.arrow(wx, wy, waypoint_orientation_arrow_length * np.cos(wpsi_rad), waypoint_orientation_arrow_length * np.sin(wpsi_rad), color='g', head_width=0.2, head_length=0.3)

        # Set plot limits
        ax.set_xlim(0, 20)
        ax.set_ylim(0, 20)

        # Add fixed grid
        ax.set_xticks(np.arange(0, 21, 1))
        ax.set_yticks(np.arange(0, 21, 1))
        ax.grid(True)

    # Animation
    anim = FuncAnimation(fig, update, frames=times, repeat=False, blit=False)
    return fig, anim

def add_waypoint():
    try:
        wx = float(waypoint_x_entry.get())
        wy = float(waypoint_y_entry.get())
        wpsi = float(waypoint_psi_entry.get())
        waypoints.append((wx, wy, wpsi))
        waypoints_listbox.insert(tk.END, f"Waypoint: X={wx}, Y={wy}, Psi={wpsi}")
        waypoint_x_entry.delete(0, tk.END)
        waypoint_y_entry.delete(0, tk.END)
        waypoint_psi_entry.delete(0, tk.END)
    except ValueError:
        messagebox.showerror("Error", "Input harus berupa angka")

def start_simulation():
    global anim_global, canvas_widget, waypoints
    if anim_global is not None:
        messagebox.showinfo("Info", "Simulasi sudah berjalan.")
        return

    if not waypoints:
        messagebox.showerror("Error", "Tambahkan setidaknya satu waypoint.")
        return

    try:
        left_speed = float(left_speed_entry.get())
        left_radius = float(left_radius_entry.get())
        right_speed = float(right_speed_entry.get())
        right_radius = float(right_radius_entry.get())
        body_radius = float(body_radius_entry.get())
        initial_x = float(initial_x_entry.get())
        initial_y = float(initial_y_entry.get())
        initial_orientation = float(initial_orientation_entry.get())

        fig, anim = differential_drive_simulation(left_speed, left_radius, right_speed, right_radius, body_radius, initial_x, initial_y, initial_orientation, waypoints.copy())
        canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(fill=tk.BOTH, expand=True)

        # Store animation object in global variable
        anim_global = anim

        # Ensure animation remains active until plt.show() is called
        anim._start()

    except ValueError:
        messagebox.showerror("Error", "Input harus berupa angka")

def reset_simulation():
    global anim_global, canvas_widget, last_position, last_orientation, waypoints
    if anim_global is None:
        messagebox.showinfo("Info", "Tidak ada simulasi yang berjalan.")
        return

    anim_global.event_source.stop()
    anim_global = None

    plt.close('all')
    canvas_widget.destroy()

    # Reset input values
    left_speed_entry.delete(0, tk.END)
    left_radius_entry.delete(0, tk.END)
    right_speed_entry.delete(0, tk.END)
    right_radius_entry.delete(0, tk.END)
    body_radius_entry.delete(0, tk.END)
    initial_x_entry.delete(0, tk.END)
    initial_y_entry.delete(0, tk.END)
    initial_orientation_entry.delete(0, tk.END)

    # Reset waypoint list
    waypoints.clear()
    waypoints_listbox.delete(0, tk.END)

    # Reset last position and orientation
    last_position = None
    last_orientation = None

def stop_simulation():
    global anim_global, last_position, last_orientation
    if anim_global is None:
        messagebox.showinfo("Info", "Tidak ada simulasi yang berjalan.")
        return

    # Save last position and orientation before stopping the simulation
    last_position = (float(initial_x_entry.get()), float(initial_y_entry.get()))
    last_orientation = float(initial_orientation_entry.get())

    anim_global.event_source.stop()

def continue_simulation():
    global anim_global
    if anim_global is None:
        messagebox.showinfo("Info", "Tidak ada simulasi yang berhenti.")
        return
    anim_global.event_source.start()

def exit_program():
    global anim_global
    if anim_global is not None:
        anim_global.event_source.stop()
    window.destroy()

# GUI Setup
window = tk.Tk()
window.title("Path Following & Path Planning")

input_frame = tk.Frame(window, padx=20, pady=20)
input_frame.pack(side=tk.LEFT, fill=tk.Y)

tk.Label(input_frame, text="Kecepatan Roda Kiri:").pack()
left_speed_entry = tk.Entry(input_frame)
left_speed_entry.pack()

tk.Label(input_frame, text="Radius Roda Kiri:").pack()
left_radius_entry = tk.Entry(input_frame)
left_radius_entry.pack()

tk.Label(input_frame, text="Kecepatan Roda Kanan:").pack()
right_speed_entry = tk.Entry(input_frame)
right_speed_entry.pack()

tk.Label(input_frame, text="Radius Roda Kanan:").pack()
right_radius_entry = tk.Entry(input_frame)
right_radius_entry.pack()

tk.Label(input_frame, text="Radius Body:").pack()
body_radius_entry = tk.Entry(input_frame)
body_radius_entry.pack()

tk.Label(input_frame, text="Posisi X Awal:").pack()
initial_x_entry = tk.Entry(input_frame)
initial_x_entry.pack()

tk.Label(input_frame, text="Posisi Y Awal:").pack()
initial_y_entry = tk.Entry(input_frame)
initial_y_entry.pack()

tk.Label(input_frame, text="Orientasi Awal (derajat):").pack()
initial_orientation_entry = tk.Entry(input_frame)
initial_orientation_entry.pack()

waypoint_frame = tk.Frame(window, padx=20, pady=20)
waypoint_frame.pack(side=tk.LEFT, fill=tk.Y)

tk.Label(waypoint_frame, text="Waypoint X:").pack()
waypoint_x_entry = tk.Entry(waypoint_frame)
waypoint_x_entry.pack()

tk.Label(waypoint_frame, text="Waypoint Y:").pack()
waypoint_y_entry = tk.Entry(waypoint_frame)
waypoint_y_entry.pack()

tk.Label(waypoint_frame, text="Waypoint Orientasi (derajat):").pack()
waypoint_psi_entry = tk.Entry(waypoint_frame)
waypoint_psi_entry.pack()

add_waypoint_button = tk.Button(waypoint_frame, text="Tambah Waypoint", command=add_waypoint)
add_waypoint_button.pack()

waypoints_listbox = tk.Listbox(waypoint_frame)
waypoints_listbox.pack(fill=tk.BOTH, expand=True)

plot_frame = tk.Frame(window)
plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

start_button = tk.Button(window, text="Mulai Simulasi", command=start_simulation)
start_button.pack(side=tk.BOTTOM)

reset_button = tk.Button(window, text="Reset", command=reset_simulation)
reset_button.pack(side=tk.BOTTOM)

stop_button = tk.Button(window, text="Stop", command=stop_simulation)
stop_button.pack(side=tk.BOTTOM)

continue_button = tk.Button(window, text="Lanjutkan Simulasi", command=continue_simulation)
continue_button.pack(side=tk.BOTTOM)

exit_button = tk.Button(window, text="Keluar", command=exit_program)
exit_button.pack(side=tk.BOTTOM)

window.geometry("800x600+100+100")
window.mainloop()
