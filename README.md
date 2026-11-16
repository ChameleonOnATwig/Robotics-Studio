# Robotics Studio: DigiRunner

A small bipedal robot running ROS 2. A Raspberry Pi 4B runs the robot nodes (bus servos via the BusLinker, GY-521 IMU, touchscreen), and a laptop on the same network runs RViz, the `ros2` CLI and training.

## Requirements

- **ROS 2 Jazzy** on **Ubuntu 24.04**, on both the Pi and the laptop.
- `colcon` and `rosdep` (`sudo apt install python3-colcon-common-extensions python3-rosdep`).

## Workspace setup

The ROS 2 workspace lives in [`ros2_ws/`](ros2_ws), and every package is under `ros2_ws/src/`. The commands in this repo and in the docs assume the workspace is at `~/ros2_ws`, so clone the repo and link it there:

```bash
git clone https://github.com/ChameleonOnATwig/Robotics-Studio.git ~/Robotics-Studio
ln -s ~/Robotics-Studio/ros2_ws ~/ros2_ws
```

Build:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Run `source ~/ros2_ws/install/setup.bash` in every new terminal (or add it to `~/.bashrc`, below) before any `ros2` command will find these packages. `build/`, `install/` and `log/` are git-ignored.

### Which machine builds what

| Package | Pi | Laptop |
|---|:-:|:-:|
| `digirunner_interfaces` | ✓ | ✓ |
| `digirunner_description` | ✓ | ✓ |
| `digirunner_bringup` | ✓ | ✓ |
| `digirunner_hardware` | ✓ | |
| `digirunner_control` | ✓ | |
| `digirunner_policy` | ✓ | |
| `digirunner_ui` | ✓ | |

The laptop needs the interfaces (so the CLI knows the `Walk` and `ServoStatus` types), the description (so RViz can load the model) and bringup (launch files). Build only those with:

```bash
colcon build --symlink-install --packages-up-to digirunner_interfaces digirunner_description digirunner_bringup
```

## Networking: `ROS_DOMAIN_ID`

This robot uses **`ROS_DOMAIN_ID=57`**. The Pi and the laptop must use the same number, and it keeps us from seeing (or commanding) a classmate's robot on the shared Wi-Fi. If another team is already on 57, pick a new number in the safe range 0–101 and update it here.

Add these lines to `~/.bashrc` on **both** the Pi and the laptop:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=57
```

Check it from a new terminal with `echo $ROS_DOMAIN_ID`. If the two machines can ping each other but `ros2 topic list` doesn't show the other side's topics, check that both have the same domain ID and that the network allows multicast (see [DDS and ROS_DOMAIN_ID](docs/ros2-concepts.md#dds-and-ros_domain_id)).

## Documentation

- [ROS2 Concepts](docs/ros2-concepts.md)
