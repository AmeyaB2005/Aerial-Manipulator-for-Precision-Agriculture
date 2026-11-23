import mujoco
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('tdcr_arm.xml')
data = mujoco.MjData(model)

# 1. Analytical Stiffness
# In our MJCF model, each joint has a stiffness defined by 'stiffness' attribute.
# For N joints in series, the equivalent rotational stiffness is:
# 1 / K_eq = sum(1 / K_i)
hinge_x_stiffnesses = [model.jnt_stiffness[i] for i in range(model.njnt) if model.jnt_type[i] == 3 and "bend_x" in model.joint(i).name]

if hinge_x_stiffnesses:
    K_eq_analytical = 1.0 / sum(1.0 / k for k in hinge_x_stiffnesses)
else:
    K_eq_analytical = 0.0

print("====================================================")
print("             TDCR STIFFNESS ANALYSIS                ")
print("====================================================")
print("--- Analytical Model Stiffness ---")
print(f"Number of bending joints: {len(hinge_x_stiffnesses)}")
print(f"Stiffness per joint segment: {hinge_x_stiffnesses[0] if hinge_x_stiffnesses else 0.0} N-m/rad")
print(f"Equivalent overall bending stiffness: {K_eq_analytical:.4f} N-m/rad\n")

# 2. Experimental Measurement in Simulation
# We apply a constant tension to one tendon, let it settle to equilibrium, 
# and measure the ratio of applied bending moment to the bending deflection.
print("--- Experimental Bending Test in MuJoCo ---")
test_force = 30.0  # Newtons
R_tendon = 0.01766 # Tendon radius in meters

# Apply constant tension to the NE tendon
data.actuator('pull_NE').ctrl[0] = test_force

# Run simulation steps until the system reaches static equilibrium (velocities are near zero)
settled = False
for _ in range(5000):
    mujoco.mj_step(model, data)
    # Check if kinetic energy is very low (system has settled)
    if np.linalg.norm(data.qvel) < 1e-5:
        settled = True
        break

# Measure the total bending rotation of the top disk relative to the base
# We sum the joint angles of all segments along the X and Y axes
theta_x = sum(data.joint(f"bend_x_{i}").qpos[0] for i in [1, 2, 3]) + data.joint("bend_x_top").qpos[0]
theta_y = sum(data.joint(f"bend_y_{i}").qpos[0] for i in range(1, 4)) + data.joint("bend_y_top").qpos[0]
theta_total = np.sqrt(theta_x**2 + theta_y**2)

# The bending moment applied by the NE tendon is:
# M = F * R_tendon
moment = test_force * R_tendon

# Bending stiffness: K = M / theta
if theta_total > 0:
    K_measured = moment / theta_total
    print(f"Applied Tendon Force: {test_force:.2f} N")
    print(f"Bending Moment (F * R_tendon): {moment:.4f} N-m")
    print(f"Measured Bending Angle: {theta_total:.4f} rad ({np.degrees(theta_total):.2f} degrees)")
    print(f"Experimentally Measured Stiffness (M/theta): {K_measured:.4f} N-m/rad")
    print("----------------------------------------------------")
    print(f"Difference (Analytical vs Experimental): {abs(K_measured - K_eq_analytical):.4f} N-m/rad")
else:
    print("No bending deflection detected.")
print("====================================================")
