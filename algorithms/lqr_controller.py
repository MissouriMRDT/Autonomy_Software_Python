#
# Mars Rover Design Team
# mpc_controller.py
#
# Created on Mar 4, 2023
# Updated on Mar 4, 2023
#

import do_mpc
import numpy as np

# Define system parameters
L = 1.0  # wheelbase
R = 0.5  # wheel radius
v_max = 1.0  # maximum linear velocity
w_max = np.pi / 2  # maximum angular velocity
N = 10  # prediction horizon
dt = 0.1  # time step

# Define the model
model_type = "continuous"  # use continuous model
model = do_mpc.model.Model(model_type)

# Define state variables
x = model.set_variable(var_type="_x", var_name="x", shape=(3, 1))
y = model.set_variable(var_type="_x", var_name="y", shape=(3, 1))
theta = model.set_variable(var_type="_x", var_name="theta", shape=(3, 1))

# Define input variables
vl = model.set_variable(var_type="_u", var_name="vl")
vr = model.set_variable(var_type="_u", var_name="vr")

# Define parameters
params = model.set_variable(var_type="_p", var_name="params", shape=(3, 1))
params[0, :] = v_max
params[1, :] = w_max
params[2, :] = L

# Define dynamics
r = R / 2 * (vl + vr)
x_dot = r * np.cos(theta)
y_dot = r * np.sin(theta)
theta_dot = R / L * (vr - vl)
model.set_rhs("x", x_dot)
model.set_rhs("y", y_dot)
model.set_rhs("theta", theta_dot)

# Define stage cost
cost = x**2 + y**2 + theta**2 + vl**2 + vr**2
model.set_expression("cost", cost)

# Setup model.
model.setup()

# Define controller
mpc = do_mpc.controller.MPC(model)

# Set up optimization problem
setup_mpc = {"n_horizon": N, "t_step": dt, "n_robust": 1, "store_full_solution": True}
mpc.set_param(**setup_mpc)

# Set up constraints
vl_max = 1000.0
vr_max = 1000.0
mpc.bounds["lower", "_u", "vl"] = -vl_max
mpc.bounds["upper", "_u", "vl"] = vl_max
mpc.bounds["lower", "_u", "vr"] = -vr_max
mpc.bounds["upper", "_u", "vr"] = vr_max

# Set up reference tracking
mpc.x_ref = np.zeros((3, N + 1))
mpc.u_ref = np.zeros((2, N))

# Set up optimizer
mpc.setup()

# Define simulation parameters
N_sim = 100
t_step_sim = 0.1
t_sim = np.arange(0, N_sim * t_step_sim, t_step_sim)

# Define initial state
x0 = np.array([[0], [0], [np.pi / 2]])

# Simulate the system using the MPC controller
x_sim = np.zeros((3, N_sim + 1))
u_sim = np.zeros((2, N_sim))
for i in range(N_sim):
    # Set the current state
    mpc.x0 = x0

    # Solve the optimization problem and obtain the optimal control inputs
    mpc.solve()
    u = mpc.u_pred[0, :]

    # Apply the control inputs to the system
    vl = u[0]
    vr = u[1]
    r = R / 2 * (vl + vr)
