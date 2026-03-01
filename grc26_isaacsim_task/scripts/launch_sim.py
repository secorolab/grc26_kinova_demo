from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension

# Enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Get your package's USD path via ament
import ament_index_python.packages as ament
package_share = ament.get_package_share_directory('isaacsim_grc26')
usd_path = f"{package_share}/assets/grc26_demo.usd"
print(f"Loading USD from: {usd_path}")

# Open the stage directly — preserves OmniGraphs, physics, TF publishers
omni.usd.get_context().open_stage(usd_path)

# Wait for stage to fully load
simulation_app.update()
simulation_app.update()

world = World()
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
