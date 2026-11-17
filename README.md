## Documentation
- [ROS2 Concepts](docs/ros2-concepts.md)

## Requirements
- **Ubuntu 24.04** (Noble) with **ROS 2 Jazzy**. This applies to both the Pi and the laptop.
- The colcon workspace is `ros2_ws/` in this repo, and the packages are in `ros2_ws/src/`. All commands below are run from the repo root.

## Network: ROS_DOMAIN_ID
All machines on this robot use **`ROS_DOMAIN_ID=24`** (our group number). This keeps us apart from other teams on the same network. A machine with a different ID, or with none set (the default is 0), won't see our topics.

Add these lines to `~/.bashrc` on **every** machine (Pi and laptop). Replace `<path-to-repo>` with the full path to your clone, for example `~/Robotics-Studio`:
```bash
source /opt/ros/jazzy/setup.bash
source <path-to-repo>/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
```
Check it in a new terminal with `echo $ROS_DOMAIN_ID`, which should print `24`.

## Which packages go where
| Package | Pi (robot) | Laptop |
|---|---|---|
| `digirunner_interfaces` | ✅ | ✅ |
| `digirunner_description` | ✅ | ✅ |
| `digirunner_bringup` | ✅ | ✅ |
| `digirunner_hardware` | ✅ | — |
| `digirunner_control` | ✅ | — |
| `digirunner_ui` | ✅ | — |
| `digirunner_policy` | ✅ | — |

The laptop is only used for monitoring, RViz and launching. It needs the message definitions, the URDF and the launch files, but none of the hardware drivers.

## Build
**Pi (everything):**
```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

**Laptop (interfaces, description, bringup only):**
```bash
cd ros2_ws
rosdep install --from-paths src/digirunner_interfaces src/digirunner_description src/digirunner_bringup --ignore-src -y
colcon build --symlink-install --packages-select digirunner_interfaces digirunner_description digirunner_bringup
source install/setup.bash
```
