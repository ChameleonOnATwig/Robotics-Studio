# ROS 2 Concepts for Digi-Runner

Nov 12, 2026 · Maximilian Shah

These are the ROS 2 pieces a small bipedal robot actually uses. Each entry says what it is, why it matters, and how it would be used on this robot, plus the basic `ros2` CLI commands where the CLI is the simple way in.

**Hardware assumed in the examples:** Raspberry Pi 4B, bus servos driven through the Bus Linker (serial), a GY-521 (MPU-6050) IMU on I²C, and the 5" Touchscreen IPS LCD Display.

**Commands assume** ROS 2 Jazzy (the LTS release for Ubuntu 24.04, which runs on the Pi 4). Most also work on Humble. Names in angle brackets or under `digirunner_*` are placeholders for your own packages and nodes. Before any command will find your code, source the workspace with `source ~/ros2_ws/install/setup.bash`.

## Core building blocks

Nodes talking over topics, services and actions are the skeleton of every ROS 2 robot. Everything else in this doc builds on them.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="images/node-graph-dark.png">
  <img alt="Example node graph for the robot" src="images/node-graph.png">
</picture>

Ovals are nodes (shaded ones run on the laptop), rectangles are topics, the diamond is a service and the hexagon is an action. Each block holds its interface type, and arrows show which way data flows. The node and topic names are the examples used throughout this doc; `rqt_graph` draws the live version of this picture for your real system. The touchscreen only talks to /walk: its Stop button cancels the walk goal, so the robot stands still instead of collapsing. A hard e-stop (torque off) comes from a physical power switch or a ros2 service call from the laptop, so it still works if the gait node has crashed. The gait node can also cut torque itself when it detects a fall. The double oval is the controller slot: either classical_controller or policy_controller fills it (see Packages), and the rest of this doc calls whichever one is running gait_node.

### Nodes

**What:** A node is a single program (a process) that does one job.

**Why:** You can write, test, restart and swap each job on its own. A crash in one node doesn't take down the robot.

**Example:** `imu_node` reads the MPU-6050 over I²C. `servo_driver_node` talks to the Bus Linker over serial. `gait_node` generates walking motions, and `screen_ui_node` runs the touchscreen: it draws the face and turns taps into /walk requests, like start and stop.

```bash
ros2 run digirunner_hardware imu_node      # start a node: ros2 run <package> <executable>
ros2 node list                        # see every running node
ros2 node info /imu_node              # its publishers, subscribers, services, actions
```

### Topics

**What:** A topic is a named channel for a continuous stream of data, such as `/imu/data` or `/joint_states`.

**Why:** Nodes don't need to know about each other. They only need to agree on a topic name and a message type.

**Example:** The IMU node streams orientation and acceleration on `/imu/data` about 100 times a second.

```bash
ros2 topic list -t                    # all topics, with their message types
ros2 topic echo /imu/data             # print messages as they arrive
ros2 topic hz /imu/data               # measure the publish rate
ros2 topic info /imu/data -v          # who publishes and subscribes, and their QoS
```

### Publishers / Subscribers

**What:** A publisher sends messages on a topic, and a subscriber receives them. A topic can have many of each.

**Why:** This is the main "many-to-many, fire-and-forget" way nodes communicate. It suits sensor data and commands sent over and over.

**Example:** `imu_node` publishes `/imu/data`, and the balance controller and a logger both subscribe to it. The gait node publishes target joint angles, and the servo driver subscribes and sends them to the servos.

Publishers and subscribers are written in code (`create_publisher` / `create_subscription` in rclpy). The CLI can stand in for either end while testing:

```bash
# act as a publisher: send one command by hand
ros2 topic pub --once /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, -0.4, 0.2]}"
# publish repeatedly at 10 Hz (Ctrl+C to stop)
ros2 topic pub --rate 10 /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, -0.4, 0.2]}"
# act as a subscriber: ros2 topic echo (above)
```

### Interfaces (messages, services, actions)

**What:** Interfaces define the data structures nodes exchange: `.msg` for topics, `.srv` for request/response, and `.action` for long-running goals.

**Why:** They're strongly typed, so every node agrees on the format. Use the standard ones (`sensor_msgs/Imu`, `sensor_msgs/JointState`, `std_srvs/SetBool`) wherever you can, because tools like RViz already understand them.

**Example:** `sensor_msgs/JointState` for servo positions. A custom `ServoStatus.msg` for voltage, temperature and load, since the bus servos report those.

```bash
ros2 interface show sensor_msgs/msg/Imu     # fields of a type
ros2 interface list                          # every type installed
ros2 interface package std_srvs              # types in one package
ros2 interface proto sensor_msgs/msg/JointState   # a fill-in-the-blanks template for ros2 topic pub
```

Custom interfaces aren't made from the CLI. Put them in their own CMake package (`ros2 pkg create --build-type ament_cmake --license MIT digirunner_interfaces`), add files like `msg/ServoStatus.msg`, and list them in `rosidl_generate_interfaces(...)` in that package's `CMakeLists.txt`.

### Services

**What:** A service is a request → response call, like a function call between nodes. It's one-off and the caller waits for the answer.

**Why:** It's good for quick commands that need a confirmation, and wasteful for streaming data.

**Example:** `/servos/set_torque` turns servo torque on or off (to go limp safely). `/imu/calibrate` zeros the gyro while the robot stands still.

```bash
ros2 service list -t                          # services and their types
ros2 service type /servos/set_torque
ros2 service call /servos/set_torque std_srvs/srv/SetBool "{data: false}"   # go limp
```

### Actions

**What:** An action is a long-running goal. While it runs, the action server sends feedback, and the client can cancel it.

**Why:** Some tasks take seconds. You want progress updates and the ability to stop partway, which topics and services don't give you.

**Example:** A `/walk` action with the goal "take 10 steps." It sends feedback ("step 4 of 10") and is cancelled if the IMU detects a fall. `/stand_up` and `/sit_down` fit the same pattern.

```bash
ros2 action list -t
ros2 action info /walk                        # its server and clients
ros2 action send_goal /walk digirunner_interfaces/action/Walk "{steps: 10}" --feedback
# Ctrl+C while a goal is running cancels it
```

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="images/topic-service-action-dark.png">
  <img alt="Topic vs service vs action message timing" src="images/topic-service-action.png">
</picture>

Time runs downward. A service caller waits for its one reply, while an action client keeps receiving feedback until the result arrives, and can cancel along the way.

### Parameters

**What:** Parameters are configuration values that belong to a node. You can load them from a YAML file at startup and change them while the node runs.

**Why:** You can tune the robot without recompiling, which matters a lot for gaits.

**Example:** Step height, step length, cycle time, PID gains, the servo port (`/dev/ttyUSB0`) and per-servo calibration offsets, all in `gait_params.yaml`.

```bash
ros2 param list /gait_node
ros2 param get /gait_node step_height
ros2 param set /gait_node step_height 0.02          # change it live
ros2 param dump /gait_node > gait_params.yaml       # save the current values to a file
ros2 run digirunner_control gait_node --ros-args --params-file gait_params.yaml
ros2 run digirunner_control gait_node --ros-args -p step_height:=0.02   # set one at launch
```

### Timers, callbacks and executors

**What:** A node's work runs in callbacks, which fire when a message arrives or a timer ticks. The executor decides when those callbacks run.

**Why:** This is how you get a steady control loop instead of a `while True` loop that blocks everything else.

**Example:** A 50 Hz timer in `gait_node` computes the next joint positions and publishes them on each tick.

These are set up in code (`create_timer(0.02, self.step)`), not from the CLI. To check the loop really runs at 50 Hz, use `ros2 topic hz /joint_commands`.

### QoS (Quality of Service)

**What:** QoS settings control delivery: reliable vs. best-effort, how many old messages to keep, and whether a late subscriber gets the last message.

**Why:** For fast sensor data the newest value matters more than guaranteed delivery. For one-time configuration you want guaranteed delivery.

**Example:** `/imu/data` uses best-effort with depth 1, so the balance loop never works on stale readings. A `/robot_mode` topic uses reliable delivery.

```bash
ros2 topic info /imu/data -v                       # shows each endpoint's QoS
ros2 topic echo /imu/data --qos-profile sensor_data   # match a best-effort publisher
```

If `echo` prints nothing on a topic you know is publishing, the cause is usually a QoS mismatch (a reliable subscriber on a best-effort publisher).

## Organizing and running code

Your code lives in packages inside a workspace, gets built with colcon, and runs from launch files.

### Packages

**What:** A package is the unit of ROS code: a folder with a `package.xml` (metadata and dependencies) plus Python (`ament_python`) or C++ (`ament_cmake`) code.

**Why:** Packages keep the project modular and let the build system resolve dependencies.

**Example:** This robot uses seven packages, split by what depends on what rather than one per node. Topics, services and actions don't get packages; only the custom interface types do.

```text
ros2_ws/src/
├── digirunner_interfaces/   (CMake)          Walk.action, ServoStatus.msg
├── digirunner_description/  (CMake)          URDF, SolidWorks meshes, RViz config
├── digirunner_hardware/     (C++)            imu_node, servo_driver_node
├── digirunner_control/      (C++)            classical_controller
├── digirunner_policy/       (C++ or Python)  policy_controller + exported .onnx model
├── digirunner_ui/           (Python)         screen_ui_node
└── digirunner_bringup/      (Python)         launch files, parameter YAMLs
```

| Package | Language | Holds | Why it's separate |
| --- | --- | --- | --- |
| `digirunner_interfaces` | CMake (no code) | `Walk.action`, `ServoStatus.msg` | Every other package depends on it; the build generates both C++ and Python versions of each type |
| `digirunner_description` | CMake (no code) | URDF/xacro, meshes, RViz config | The robot, the simulator and the laptop all load it; only changes when the CAD changes |
| `digirunner_hardware` | C++ | `imu_node`, `servo_driver_node` | Byte-level serial and I²C work, close to C; only runs on the Pi; steady timing matters most here |
| `digirunner_control` | C++ | `classical_controller` | Your hand-designed control; C++ is the industry standard for real-time control |
| `digirunner_policy` | C++ or Python | `policy_controller`, the trained `.onnx` model | Needs a neural-net runtime (ONNX Runtime) nothing else uses |
| `digirunner_ui` | Python | `screen_ui_node` | Needs a GUI library (pygame) nothing else uses; GUI code gains nothing from C++ |
| `digirunner_bringup` | Python | Launch files, parameter YAMLs | Ties everything together; tuning never touches node code |

**One controller slot, two controllers.** `classical_controller` and `policy_controller` follow the same contract: `/imu/data` and `/joint_states` in, `/joint_commands` out. Only one runs at a time, chosen by a launch argument, and nothing else in the system changes. That lets you compare your own control design against the learned one on the same robot.

```bash
ros2 launch digirunner_bringup robot.launch.py controller:=classical
ros2 launch digirunner_bringup robot.launch.py controller:=policy
```

**Training lives outside the workspace.** The neural net is trained in Python (PyTorch) in a simulator such as MuJoCo, usually over millions of attempts, then exported to ONNX and copied into `digirunner_policy`. Accurate masses and inertias from the SolidWorks export make the simulated robot match the real one, so the policy transfers better.

**Nodes you don't write.** `robot_state_publisher`, `rviz2` and the `ros2` CLI come with ROS (`sudo apt install ros-jazzy-robot-state-publisher`). List them as dependencies in `digirunner_bringup/package.xml`; they don't live in your workspace.

**What goes where.** The Pi builds every package. The laptop only needs `digirunner_interfaces`, `digirunner_description` and `digirunner_bringup` (RViz needs the model, and the CLI needs the `Walk` type), plus the training code.

**License.** Every package uses the **MIT** license: a `LICENSE` file at the repo root, and `<license>MIT</license>` in each `package.xml` (the `--license MIT` flag below fills that in). Using one license everywhere means there's never a question about what applies where.

```bash
cd ~/ros2_ws/src
ros2 pkg create digirunner_interfaces --build-type ament_cmake --license MIT --dependencies std_msgs action_msgs
ros2 pkg create digirunner_description --build-type ament_cmake --license MIT
ros2 pkg create digirunner_hardware --build-type ament_cmake --license MIT --dependencies rclcpp rclcpp_lifecycle sensor_msgs std_msgs std_srvs digirunner_interfaces
ros2 pkg create digirunner_control --build-type ament_cmake --license MIT --dependencies rclcpp rclcpp_action sensor_msgs std_msgs std_srvs digirunner_interfaces
ros2 pkg create digirunner_policy --build-type ament_cmake --license MIT --dependencies rclcpp rclcpp_action sensor_msgs std_msgs std_srvs digirunner_interfaces
ros2 pkg create digirunner_ui --build-type ament_python --license MIT --dependencies rclpy std_msgs digirunner_interfaces
ros2 pkg create digirunner_bringup --build-type ament_python --license MIT --dependencies launch launch_ros robot_state_publisher xacro
cd .. && colcon build --symlink-install       # builds digirunner_interfaces first automatically
source install/setup.bash                      # makes the new packages visible in this terminal
ros2 pkg list | grep digirunner                # check all seven are found
ros2 pkg executables digirunner_hardware       # runnable nodes in a package (empty until you add nodes)
```

**Why these dependencies.** Each one is there because the package's nodes use something from it:

| Dependency | Provides | Needed by |
| --- | --- | --- |
| `rclcpp` / `rclpy` | ROS client library for C++ / Python: nodes, publishers, subscribers, timers, parameters | Every package with nodes |
| `rclcpp_action` | Action servers and clients in C++ (Python's `rclpy` includes them) | Both controllers, which serve `/walk` |
| `rclcpp_lifecycle` | Lifecycle (managed) nodes in C++ | Hardware, if the servo driver is a lifecycle node; drop it otherwise |
| `sensor_msgs` | `Imu`, `JointState` | Hardware (publishes both), controllers (subscribe to both) |
| `std_msgs` | `Float64MultiArray` (`/joint_commands`), `String` (`/robot_mode`), `Header` | Hardware, controllers, UI, interfaces |
| `std_srvs` | `SetBool` (`/servos/set_torque`) | Hardware (serves it), controllers (call it on a fall) |
| `action_msgs` | Building blocks every action definition needs | Interfaces |
| `digirunner_interfaces` | Your `Walk` action and `ServoStatus` message | Hardware, controllers, UI |
| `launch`, `launch_ros`, `robot_state_publisher`, `xacro` | What the launch files start and use | Bringup |

`digirunner_interfaces` also needs two lines added to its `package.xml` by hand, because `ros2 pkg create` can't add them: `<buildtool_depend>rosidl_default_generators</buildtool_depend>` and `<member_of_group>rosidl_interface_packages</member_of_group>`. `rviz2` is left out of bringup on purpose, so the Pi doesn't have to install it. `--dependencies` only pre-fills `package.xml` (and `CMakeLists.txt` for C++), so you can add more later by editing those files. Put the package name right after `create`: `--dependencies` takes every word after it as a dependency, so a name at the end gets swallowed and the command fails with "the following arguments are required: package_name".

### Workspace and colcon

**What:** The workspace (`~/ros2_ws/src/...`) holds your packages. `colcon build` compiles all of them, and sourcing `install/setup.bash` makes them available in your terminal.

**Why:** It's the standard build-and-run workflow.

**Example:** After editing the gait node, rebuild only that package and relaunch.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y     # install dependencies listed in package.xml files
colcon build --symlink-install                      # build all (Python edits apply without rebuilding)
colcon build --packages-select digirunner_control        # build one package
source install/setup.bash                           # needed in every new terminal
```

Add the `source` line to `~/.bashrc` on the Pi so you don't forget it.

### Launch files

**What:** A launch file is a Python (or XML/YAML) script that starts many nodes at once, with their parameters and remappings.

**Why:** You don't want to open seven terminals every time you turn the robot on.

**Example:** `robot.launch.py` starts the IMU, servo driver, robot_state_publisher, gait and screen UI nodes, loading `gait_params.yaml`. A second `sim.launch.py` starts the simulated version instead.

```bash
ros2 launch digirunner_bringup robot.launch.py
ros2 launch digirunner_bringup robot.launch.py --show-args     # list the arguments it accepts
ros2 launch digirunner_bringup robot.launch.py use_sim:=true   # pass an argument
```

Launch files themselves are written in a text editor and placed in the package's `launch/` folder (and installed through `setup.py` or `CMakeLists.txt`).

### Remapping and namespaces

**What:** These rename a node's topics, or group them under a prefix, when the node starts.

**Why:** You can reuse one node for several similar things without editing its code.

**Example:** Run the same leg-controller node twice, once as `/left_leg` and once as `/right_leg`.

```bash
ros2 run digirunner_control leg_controller --ros-args -r __ns:=/left_leg
ros2 run digirunner_control leg_controller --ros-args -r __ns:=/right_leg
# rename one topic, or the node itself
ros2 run digirunner_hardware imu_node --ros-args -r imu/data:=/body/imu -r __node:=body_imu
```

## Robot model and geometry

The URDF is the bridge from the SolidWorks assembly to ROS. Visualization, kinematics, simulation and ros2_control all read it.

### URDF / Xacro

**What:** A URDF is an XML description of the robot: links (rigid parts), joints (axes and limits), masses, inertias and meshes. Xacro adds macros and variables on top.

**Why:** Almost everything else in ROS uses it.

**Example:** Export it from the SolidWorks assembly with the community `sw_urdf_exporter` add-in. It officially targets older SolidWorks versions, so check it with yours. It pulls masses and inertias from your material settings. Because the legs mirror each other, Xacro lets you define one leg and instantiate it for left and right. The passive ankle becomes a revolute joint with ±10° limits.

```bash
xacro digirunner.urdf.xacro > digirunner.urdf        # expand xacro into plain URDF
check_urdf digirunner.urdf                      # parse it and print the link tree (liburdfdom-tools)
ros2 launch urdf_tutorial display.launch.py model:=$PWD/digirunner.urdf   # quick look in RViz with sliders
```

### TF2 (transforms)

**What:** TF2 tracks the coordinate frames (`base_link`, `left_foot`, `imu_link`...) and how they relate to each other over time.

**Why:** It answers "where is the left foot relative to the body right now?", which gait and balance math needs constantly.

**Example:** Look up `left_foot` relative to `base_link` to check the center of mass is over the support foot. The IMU's position in the floor tray gives the fixed `base_link` → `imu_link` transform.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="images/tf-tree-dark.png">
  <img alt="TF frame tree from base_link down to each foot" src="images/tf-tree.png">
</picture>

`tf2_echo base_link left_foot` walks this tree for you, combining every joint angle along the left leg into one transform.

```bash
ros2 run tf2_ros tf2_echo base_link left_foot      # print one transform live
ros2 run tf2_tools view_frames                     # write frames.pdf showing the whole tree
# publish a fixed transform by hand (metres, radians)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0.03 --z -0.06 \
    --frame-id base_link --child-frame-id imu_link
```

The `imu_link` numbers above are placeholders. Measure them from the IMU pocket's position relative to wherever you put `base_link`.

### robot_state_publisher / joint_state_publisher

**What:** `robot_state_publisher` reads the URDF and `/joint_states` and publishes every TF frame. `joint_state_publisher_gui` gives you sliders that fake joint states.

**Why:** Together they turn servo angles into a full 3D pose of the robot.

**Example:** Before any hardware is wired, drag the sliders to confirm the SolidWorks export moves correctly: joint axes point the right way and the limits are right.

```bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro digirunner.urdf.xacro)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2                                   # add RobotModel and TF displays, Fixed Frame = base_link
```

## Control

ros2_control is the standard way to connect controllers to motors. A plain custom driver node is also fine for a class project.

### ros2_control

**What:** A framework that separates the **hardware interface** (how to read and write your specific servos) from **controllers** (reusable logic such as `joint_trajectory_controller`).

**Why:** You write the Bus Linker driver once as a hardware plugin. The same controllers then run on both the real robot and the simulator.

**Example:** A `BusServoSystem` hardware plugin (C++) reads positions from the serial bus and writes position commands. A `joint_trajectory_controller` plays gait trajectories through it, and `joint_state_broadcaster` publishes `/joint_states`.

```bash
ros2 control list_hardware_interfaces          # what the hardware plugin exposes
ros2 control list_controllers                  # loaded controllers and their state
ros2 run controller_manager spawner joint_state_broadcaster      # load + start a controller
ros2 run controller_manager spawner joint_trajectory_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller   # stop sending commands
```

The simpler alternative: a Python `servo_driver_node` that subscribes to `/joint_commands`, writes to the servos, and publishes `/joint_states`. ros2_control mainly pays off if you also simulate the robot.

### Lifecycle (managed) nodes

**What:** A lifecycle node moves through explicit states: unconfigured → inactive → active (and back).

**Why:** You get controlled startup and shutdown.

**Example:** The servo driver only enables torque once it's activated, after the IMU node reports it's ready. That keeps the robot from twitching on power-up.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="images/lifecycle-dark.png">
  <img alt="Lifecycle states for the servo driver" src="images/lifecycle.png">
</picture>

Each arrow is one `ros2 lifecycle set` command. Deactivating drops back to Inactive, so the robot goes limp without the node exiting.

```bash
ros2 lifecycle nodes                           # which nodes are managed
ros2 lifecycle get /servo_driver               # current state
ros2 lifecycle set /servo_driver configure     # open the serial port, load offsets
ros2 lifecycle set /servo_driver activate      # enable torque, start commanding
ros2 lifecycle set /servo_driver deactivate    # go limp without killing the node
```

## Tools: debugging, testing and data

These tools show what the robot is doing and let you replay it afterwards. Run the heavy ones (RViz, Gazebo, plotting) on your laptop, not the Pi.

### Bag files (rosbag2)

**What:** `ros2 bag record` saves topic data to disk with timestamps. `ros2 bag play` replays it as if it were live.

**Why:** Walking tests are short and usually end in a fall. A recording lets you analyze it afterwards and test new code against old data.

**Example:** Record IMU, joint states and commands on every walk attempt. Afterwards, plot IMU pitch over time to see exactly when it started tipping, or replay the bag into a new balance node.

```bash
ros2 bag record -o walk_test_01 /imu/data /joint_states /joint_commands
ros2 bag record -a -o everything          # every topic (large files)
ros2 bag info walk_test_01                # duration, topics, message counts
ros2 bag play walk_test_01                # replay at real speed
ros2 bag play walk_test_01 --rate 0.25 --loop   # quarter speed, repeating
```

### RViz2

**What:** A 3D visualizer for robot models, TF frames, IMU orientation and markers.

**Why:** You see what the robot thinks is happening.

**Example:** Show the URDF moving live with the real servos, an arrow for IMU orientation, and a marker for the estimated center of mass.

```bash
rviz2                                    # set up displays, then File > Save Config
rviz2 -d ~/ros2_ws/src/digirunner_bringup/config/digirunner.rviz   # reopen a saved layout
```

### Gazebo (simulation)

**What:** A physics simulator that loads your URDF. Jazzy pairs with Gazebo Harmonic.

**Why:** You can test gaits without breaking parts or draining batteries. Accurate masses and inertias from SolidWorks make it much more realistic.

**Example:** Tune step timing in simulation first, then move the same parameters to the real robot.

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf        # start an empty world
ros2 run ros_gz_sim create -topic robot_description -name digirunner   # spawn the robot (needs robot_state_publisher running)
```

Sensors and controllers inside Gazebo connect to ROS through `ros_gz_bridge` and `gz_ros2_control`. Those are set up in launch files rather than typed by hand.

### CLI introspection and rqt

**What:** The `ros2` CLI (used throughout this doc) plus GUI tools: `rqt_graph` (node and topic map), `rqt_plot` and PlotJuggler (live plots), and `rqt_console` (log messages).

**Why:** They're how you find out why something isn't working.

**Example:** `ros2 topic hz /imu/data` confirms the IMU really runs at 100 Hz. `rqt_graph` shows whether the gait node is actually connected to the servo driver.

```bash
rqt_graph                                # who talks to whom
ros2 run rqt_plot rqt_plot /imu/data/linear_acceleration/x
ros2 run plotjuggler plotjuggler         # better plots; also opens bag files
ros2 run rqt_console rqt_console         # filter log messages by node and severity
ros2 doctor --report                     # check the install and network setup
```

## Networking

### DDS and ROS_DOMAIN_ID

**What:** DDS is the middleware underneath ROS 2. It lets nodes find each other automatically across machines on the same network. `ROS_DOMAIN_ID` (0–101 is the safe range) isolates groups of robots on a shared network.

**Why:** You can run RViz, Gazebo and PlotJuggler on your laptop while the Pi runs the robot nodes, with no extra code.

**Example:** Give the robot a unique domain ID, so you don't see a classmate's robot on the school Wi-Fi (or command it by accident).

```bash
export ROS_DOMAIN_ID=42        # same number on the Pi and the laptop; add to ~/.bashrc on both
ros2 multicast receive         # run on one machine...
ros2 multicast send            # ...and this on the other, to test discovery
ros2 daemon stop               # clear stale node/topic lists after changing network settings
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # Jazzy: keep traffic on this machine only
```

If the Pi and laptop can ping each other but can't see each other's topics, check that both use the same domain ID, and that the network allows multicast. Some campus Wi-Fi networks block it.

## Design rules of thumb

The sections above say what each piece is. This one covers how to decide between them when you design your own system.

### What becomes its own node

The main rule: **one node per shared bus or device.** What decides it is what's shared, not how many devices there are: six servos on one serial bus are one resource, so they get one node. Inside that rule, split by rate, by swappability and by failure isolation.

**Make it a separate node when:**

- **It owns a separate device or bus.** The IMU (I²C) and the servos (serial) get separate nodes, so a hung I²C read can't stall servo commands.
- **It runs at a different rate.** IMU at 100 Hz, gait at 50 Hz, screen at about 10 Hz. Separate nodes let each run at its own pace.
- **You'd want to swap or reuse it.** With the servo driver separate, you can replace it with a simulator or a bag replay and the gait node never knows.
- **You want to restart or debug it alone.** Relaunch the gait node 50 times while tuning and leave the drivers running.
- **A crash shouldn't take everything down.** If `screen_ui_node` crashes, the robot should keep standing.

**Keep it in one node when:**

- **The parts share one resource**, like the servo bus.
- **They run in lockstep on tightly shared state**, such as gait generation and balance correction adjusting the same joint targets each cycle. Splitting them adds latency and jitter between two halves of one calculation.
- **They'd exchange lots of data at high rates** for no benefit. Every topic hop costs time and CPU, which matters on a Pi.

Inside one node you can still organize the code into separate classes (for example a `BusServo` class for the protocol and a node class for ROS). That gives clean code without extra processes.

**Worked example: why servo reads and writes stay in one node.** The 6 HTD-35H servos share one half-duplex serial bus at 115200 baud, reached through the Bus Linker's single USB port. The second Bus Linker port is the same bus, so it adds no bandwidth.

- **Only one process can safely open the serial port.** With two nodes, one fails to open it, or both succeed and their bytes interleave.
- **Half-duplex means taking turns.** A read is a request plus a reply. If a writer sends a move command mid-reply, packets collide. One node can run a fixed schedule, like "write all 6, then read 2."
- **Both use the same state.** Calibration offsets, IDs, limits and torque status live in one place.

From outside, the node still looks split: it subscribes to `/joint_commands`, publishes `/joint_states` and offers `/servos/set_torque`.

**Why not one node per servo?** It looks more robust, but it isn't:

- **Six nodes can't share one port.** They'd hit the same port-ownership and collision problems as splitting reads and writes, six times over. Taking turns would need a seventh node, which is just the servo driver again.
- **A biped can't walk on 5 of 6 joints.** When a servo fails, the right response is to stop safely, not to keep the others running.
- **Hardware faults usually hit several servos.** Each leg's 3 servos are daisy-chained, so a loose plug, power sag or shorted cable takes out the whole chain whatever the node layout.
- **Separate processes guard against software crashes, not hardware faults.** Six copies of the same driver share the same bugs.
- **Timing gets worse.** Six nodes on their own timers command the joints slightly out of step, adding jitter between legs that must move together.

Put the robustness inside the one driver instead: give each servo read a short timeout so one silent servo doesn't stall the others, track each servo's health in `ServoStatus`, and go to a safe state (torque off, lifecycle Inactive) when one stops responding.

One node per device *would* fit if each servo had its own connection, such as its own serial port or PWM pin with nothing shared.

| Node | Why it's separate |
| --- | --- |
| `servo_driver_node` | Sole owner of the serial bus; does all reads and writes |
| `imu_node` | Different bus (I²C) and rate (100 Hz) |
| `gait_node` | Your logic; swappable, restarted constantly while tuning. Holds balance too until that grows big enough to test alone |
| `screen_ui_node` | Owns the touchscreen (display + touch). Non-critical, so its crash shouldn't matter; it only sends /walk requests |
| `robot_state_publisher` | Standard ROS node; no reason to rewrite it |

If you later want separate nodes without separate processes, ROS 2 **component containers** run several nodes in one process.

### Topic, service or action?

| If the communication is... | Use | Robot example |
| --- | --- | --- |
| Continuous data, or a command sent over and over | Topic | `/imu/data`, `/joint_commands` |
| A quick one-off command that needs a yes/no answer | Service | `/servos/set_torque` |
| A task that takes seconds and needs progress or cancelling | Action | `/walk`, `/stand_up` |

- **Don't use a service for streaming data.** Calling it 50 times a second adds a round trip each time and blocks the caller.
- **Keep service handlers fast**, well under a second. Anything longer should be an action.
- **In Python, call services asynchronously** (`call_async`) from inside a callback. A blocking `call` from a callback can deadlock the node.

### Topic design

- **Use a standard message type when one fits.** RViz, PlotJuggler and other people's code already understand `sensor_msgs/msg/Imu` and `sensor_msgs/msg/JointState`.
- **Use SI units everywhere:** meters, radians, seconds. The HTD-35H speaks 0–1000 for 0–240°; convert at the servo driver and nowhere else, so the rest of the system only sees radians.
- **Fill in the header.** Set `header.stamp` (when the data was measured) and `header.frame_id` (e.g. `imu_link`) so TF and bag replay can use it.
- **One meaning per topic.** Don't send "walk speed" and "mode" on the same topic; give each its own.
- **Write topic names without a leading slash in code** (`imu/data`, not `/imu/data`), so namespaces and remapping work. Use lowercase snake_case.
- **Decide what happens when commands stop.** If `/joint_commands` goes quiet (a crashed gait node, a dropped Wi-Fi link), the servo driver should notice after a timeout and hold position or go limp.

### Choosing QoS

| Data | QoS | Why |
| --- | --- | --- |
| Sensor streams (`/imu/data`) | Best effort, depth 1 (the `sensor_data` profile) | Only the newest reading matters |
| Commands (`/joint_commands`) | Reliable, depth 1 | Every command should arrive, but never a backlog of stale ones |
| State a late joiner needs (`robot_description`, mode) | Reliable + transient local | A node that starts later still gets the last value |

If a subscriber gets nothing from a topic that is clearly publishing, check QoS first: a reliable subscriber can't receive from a best-effort publisher.

### Callbacks and timing

- **Never block inside a callback.** No `time.sleep()`, no waiting loops. A blocked callback stops every other callback in that node.
- **One timer drives the control loop.** Subscription callbacks just store the latest message; the timer reads those values and does the work once per cycle.
- **Use the node's clock** (`self.get_clock().now()`), not the computer's. Then the same code works in simulation and bag replay with `use_sim_time`.
- **Log with `self.get_logger().info(...)`, not `print`.** Log messages show up in `rqt_console` and in bag files.

### Parameter or hard-coded?

- **Make it a parameter** if you'll tune it, or if it differs between the real robot and simulation: gains, step sizes, rates, servo offsets, serial port.
- **Hard-code it** if it's a fact of physics or the protocol: packet headers, the servo's 0–1000 range.
- **Keep parameter YAML files in `digirunner_bringup/config`** under version control, so you can see what changed between a good walk and a bad one.

### Frames and TF

- **Follow the standard axis convention:** x forward, y left, z up, right-handed (ROS REP 103). Set this up in the SolidWorks export, or every downstream tool will look wrong.
- **Exactly one source per transform.** If both the URDF and a hand-run `static_transform_publisher` publish `base_link` → `imu_link`, TF behaves erratically.
- **Put fixed offsets (like the IMU) in the URDF** as fixed joints, rather than in separate static publishers.

### Package boundaries

- **Split packages by dependency, not one per node.** Several nodes can share a package.
- **Interfaces get their own package**, because every other package depends on them.
- **The robot description gets its own package**, because both the real robot and the simulator load it.

### Testing habits

- **Record a bag on every hardware test** and name it by date or test number. Most bugs are found by replaying the failure, not by watching it live.
- **Test new code with torque off, or the robot hanging in the air,** before letting it stand.
