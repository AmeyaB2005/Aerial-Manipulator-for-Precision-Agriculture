import mujoco
import mujoco.viewer
import time
import math

model = mujoco.MjModel.from_xml_path('tdcr_arm.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        step_start = time.time()
        elapsed = time.time() - start_time
        
        # Calculate oscillating tension for the cables using sine waves (pulled harder with amplitude of 35)
        amplitude = 35.0
        tension_NE = max(0, amplitude * math.sin(elapsed * 2))
        tension_SE = max(0, amplitude * math.sin(elapsed * 2 + math.pi/2))
        tension_SW = max(0, amplitude * math.sin(elapsed * 2 + math.pi))
        tension_NW = max(0, amplitude * math.sin(elapsed * 2 + 3*math.pi/2))
        
        # Apply tension to the actuators
        data.actuator('pull_NE').ctrl[0] = tension_NE
        data.actuator('pull_SE').ctrl[0] = tension_SE
        data.actuator('pull_SW').ctrl[0] = tension_SW
        data.actuator('pull_NW').ctrl[0] = tension_NW

        mujoco.mj_step(model, data)
        viewer.sync()

        # Enforce real-time execution
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
