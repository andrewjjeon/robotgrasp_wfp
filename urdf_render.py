import pybullet as p
import pybullet_data
import time

# connect to pybullet GUI
physics_client = p.connect(p.GUI)

# set additional search path to find PyBullet's default assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())

urdf_path = "/home/andrewjjeon/FoundationPose/demo_data/panda.urdf"
robot_id = p.loadURDF(urdf_path, useFixedBase=True)

# Set the camera position for better viewing
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,    # adjust distance as needed for a full view
    cameraYaw=30,          # set yaw angle for a better side view
    cameraPitch=-30,       # set pitch to view from above
    cameraTargetPosition=[0, 0, 0.5]
)

# set gravity (optional, as this is a static render)
p.setGravity(0, 0, -9.81)

# wait for a few seconds to display the model
time.sleep(5)

# disconnect the PyBullet client
p.disconnect()
