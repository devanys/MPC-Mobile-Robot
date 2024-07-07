import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from scipy.optimize import minimize

# Global variables to store animation and canvas widget
anim_global = None
canvas_widget = None
waypoints = []

def mpc_control(x, y, theta, waypoints, body_radius, dt, prediction_horizon):
    def objective(u):
        cost = 0.0
        x_pred, y_pred, theta_pred = x, y, theta
        for i in range(prediction_horizon):
            v = u[i]
            omega = u[prediction_horizon + i]
            x_pred += v * np.cos(theta_pred) * dt
            y_pred += v * np.sin(theta_pred) * dt
            theta_pred += omega * dt

            target_x, target_y, _ = waypoints[0]
            cost += (x_pred - target_x)**2 + (y_pred - target_y)**2
        return cost

    def constraints(u):
        return []

    u0 = [0.0] * (2 * prediction_horizon)
    bounds = [(-1, 1)] * (2 * prediction_horizon)
    result = minimize(objective, u0, bounds=bounds, constraints={'type': 'ineq', 'fun': constraints})
    return result.x[0], result.x[prediction_horizon]

def differential_drive_simulation(body_radius, initial_x, initial_y, initial_orientation, waypoints):
    initial_orientation_rad = np.radians(initial_orientation)

    x = initial_x
    y = initial_y
    theta = initial_orientation_rad

    x_plot = [x]
    y_plot = [y]

    total_time = 20
    dt = 0.1
    times = np.arange(0, total_time, dt)
    prediction_horizon = 10

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='datalim')
    ax.invert_yaxis()

    ax.set_xticks(np.arange(0, 21, 1))
    ax.set_yticks(np.arange(0, 21, 1))
    ax.grid(True)

    # Draw the static planned path once
    points = [(x, y)] + [(wx, wy) for wx, wy, wpsi in waypoints]
    for i in range(len(points) - 1):
        ax.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], 'k--')

    def update(frame):
        nonlocal x, y, theta

        if not waypoints:
            return

        v, omega = mpc_control(x, y, theta, waypoints, body_radius, dt, prediction_horizon)

        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = omega * dt

        x += dx
        y += dy
        theta += dtheta

        if np.sqrt((waypoints[0][0] - x)**2 + (waypoints[0][1] - y)**2) < 0.1:
            waypoints.pop(0)

        x_plot.append(x)
        y_plot.append(y)

        ax.clear()
        ax.plot(x_plot, y_plot, 'b-')

        # Re-draw the static planned path
        for i in range(len(points) - 1):
            ax.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], 'k--')

        robot_size = 0.5
        corners = np.array([
            [-robot_size / 2, -robot_size / 2],
            [robot_size / 2, -robot_size / 2],
            [robot_size / 2, robot_size / 2],
            [-robot_size / 2, robot_size / 2],
            [-robot_size / 2, -robot_size / 2]
        ])

        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        transformed_corners = np.dot(corners, rotation_matrix.T) + np.array([x, y])

        ax.plot(transformed_corners[:, 0], transformed_corners[:, 1], 'r-')

        for wx, wy, wpsi in waypoints:
            ax.plot(wx, wy, 'go')
            waypoint_orientation_arrow_length = 1.0
            wpsi_rad = np.radians(wpsi)
            ax.arrow(wx, wy, waypoint_orientation_arrow_length * np.cos(wpsi_rad), waypoint_orientation_arrow_length * np.sin(wpsi_rad), color='g', head_width=0.2, head_length=0.3)

        ax.set_xlim(0, 20)
        ax.set_ylim(0, 20)

        ax.set_xticks(np.arange(0, 21, 1))
        ax.set_yticks(np.arange(0, 21, 1))
        ax.grid(True)

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

def run_simulation():
    global anim_global, canvas_widget, waypoints

    if anim_global is not None:
        messagebox.showinfo("Info", "Simulasi sudah berjalan.")
        return

    if not waypoints:
        messagebox.showerror("Error", "Tambahkan setidaknya satu waypoint.")
        return

    try:
        body_radius = float(body_radius_entry.get())
        initial_x = float(initial_x_entry.get())
        initial_y = float(initial_y_entry.get())
        initial_orientation = float(initial_orientation_entry.get())

        fig, anim = differential_drive_simulation(body_radius, initial_x, initial_y, initial_orientation, waypoints.copy())
        canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(fill=tk.BOTH, expand=True)

        anim_global = anim
        anim._start()
    except ValueError:
        messagebox.showerror("Error", "numbers input only")

def exit_program():
    global anim_global
    if anim_global is not None:
        anim_global.event_source.stop()
    window.destroy()

# GUI Setup
window = tk.Tk()
window.title("MPC")

input_frame = tk.Frame(window, padx=20, pady=20)
input_frame.pack(side=tk.LEFT, fill=tk.Y)

tk.Label(input_frame, text="Radius Body:").pack()
body_radius_entry = tk.Entry(input_frame)
body_radius_entry.pack()

tk.Label(input_frame, text="Position X:").pack()
initial_x_entry = tk.Entry(input_frame)
initial_x_entry.pack()

tk.Label(input_frame, text="Position Y:").pack()
initial_y_entry = tk.Entry(input_frame)
initial_y_entry.pack()

tk.Label(input_frame, text="Orientation (degree):").pack()
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

tk.Label(waypoint_frame, text="Waypoint Orientation (derajat):").pack()
waypoint_psi_entry = tk.Entry(waypoint_frame)
waypoint_psi_entry.pack()

add_waypoint_button = tk.Button(waypoint_frame, text="Add Waypoint(s)", command=add_waypoint)
add_waypoint_button.pack()

waypoints_listbox = tk.Listbox(waypoint_frame)
waypoints_listbox.pack(fill=tk.BOTH, expand=True)

plot_frame = tk.Frame(window)
plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

run_button = tk.Button(window, text="Run", command=run_simulation)
run_button.pack(side=tk.BOTTOM)

exit_button = tk.Button(window, text="Quit", command=exit_program)
exit_button.pack(side=tk.BOTTOM)

window.geometry("800x600+100+100")
window.mainloop()
