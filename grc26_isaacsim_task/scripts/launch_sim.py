from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension

# Enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.ros2.sim_control")
simulation_app.update()

# Get the USD path
from pathlib import Path
package_dir = Path(__file__).parents[1]
usd_path = Path.joinpath(package_dir, "assets", "grc26_demo.usd").as_posix()
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
