# MPC Mobile Robot Simulation GUI
![Screenshot 2024-06-06 070415](https://github.com/devanys/MPC-Mobile-Robot/assets/145944367/2f3df067-8bc3-4405-bb69-5b681eb5cb57)

![Screenshot 2024-06-06 055753](https://github.com/devanys/MPC-Mobile-Robot/assets/145944367/8afd10e4-9ff9-42ef-aa45-3f25b9f7f9c7)


https://github.com/user-attachments/assets/4d9d5e96-6b7f-46c4-a4c8-ba7a913346b8



The provided code is a Python script that creates a graphical user interface (GUI) using `tkinter` for simulating a robot's movement based on Model Predictive Control (MPC). The robot follows a series of user-defined waypoints. The GUI allows users to input parameters and waypoints, then visualize the robot's path using Matplotlib.

## Global Variables
- `anim_global`: Stores the animation object.
- `canvas_widget`: Stores the Matplotlib canvas widget.
- `waypoints`: List of waypoints, where each waypoint is a tuple of (x, y, orientation).

## Functions

### `mpc_control(x, y, theta, waypoints, body_radius, dt, prediction_horizon)`
This function computes the control inputs (velocity `v` and angular velocity `omega`) using MPC.
- **Objective Function:** Minimizes the cost of deviating from the next waypoint.
- **Constraints:** No specific constraints in this implementation.
- **Optimization:** Uses `scipy.optimize.minimize` to find the optimal control inputs.

### `differential_drive_simulation(body_radius, initial_x, initial_y, initial_orientation, waypoints)`
This function sets up the simulation and visualization of the robot's movement.
- **Initial Setup:** Converts initial orientation to radians and initializes position and orientation.
- **Matplotlib Plot:** Sets up the plot and draws the static planned path.
- **Update Function:** Updates the robot's position and orientation based on the control inputs and plots the new position.

### `add_waypoint()`
Adds a waypoint to the list and updates the GUI listbox with the new waypoint.

### `run_simulation()`
Runs the simulation by calling `differential_drive_simulation` and displaying the animation in the GUI.
- Checks if the simulation is already running.
- Validates user inputs.
- Starts the animation if inputs are valid.

### `exit_program()`
Stops the animation and closes the GUI window.

## GUI Setup
The GUI is created using `tkinter`:
- **Input Frame:** Contains entries for the robot's body radius, initial position, and orientation.
- **Waypoint Frame:** Contains entries for adding waypoints (x, y, orientation) and a listbox to display the added waypoints.
- **Plot Frame:** Area where the Matplotlib plot is displayed.
- **Buttons:** Buttons to add waypoints, run the simulation, and quit the program.

## Example Run
When the script is run, the user can:
1. Enter the body radius, initial position, and orientation of the robot.
2. Add waypoints to the list.
3. Click "Run" to start the simulation.
4. Visualize the robot's movement and its path towards the waypoints.
5. Click "Quit" to exit the program.

## references
- **J. B. Rawlings and D. Q. Mayne**, “Model Predictive Control: Theory and Design”, Madison, WI: Nob Hill Publishing, 2009.
- **F. Borrelli, A. Bemporad, and M. Morari**, “Predictive Control for Linear and Hybrid Systems”, Cambridge, UK: Cambridge University Press, 2017.
- **D. Q. Mayne, J. B. Rawlings, C. V. Rao, and P. O. M. Scokaert**, "Constrained model predictive control: Stability and optimality,“ vol. 36, no. 6, pp. 789-814, Jun. 2000, doi: 10.1016/S0005-1098(99)00214-9.
- **Mathlab's Understanding MPC** https://youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEHww8&si=w7yuGim3kMNhnvUn
