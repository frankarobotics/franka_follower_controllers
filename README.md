# ROS 2 Follower Controllers for Franka Robots

This package provides follower controllers for Franka robots.
The purpose of these controllers is the control of Franka robots with a low-frequency control signal.
Examples are teleoperation devices (gello, space mouse) or AI models (VLA, VML, imitation learning etc.), which are usually publishing control signals at less than 1kHz.
The follower controller will listen to a specified topic and match the published positions while maintaining the required 1kHz control loop.

Currently, the following controllers are available:
    - `joint_follower_controller`: Follows absolute joint positions

## State Topic

Each controller publishes its current state on `~/state` (`std_msgs/String`, transient local QoS):

- **INACTIVE**: Controller is not active (published on configure and deactivate)
- **SYNCING**: Controller is active and moving to the start position (only when `sync_after_activation: true`)
- **FOLLOWING**: Controller is active and following the target joint states

## Launch a Controller

You can launch a single controller with:

```bash
ros2 launch franka_follower_controllers follower.launch.py config_file:=/path/to/your/config_file.yaml
```

## Config file

This is an example of a configuration file:

```yaml
# List of controllers that you want to load
# That is usually a single follower controller and some supporting controllers that you might to want to switch to at some point
controller_names:
    - joint_follower_controller

# If `true` the first controller in the `controller_names` list is activated automatically
# If `false` no controller will be activated and you will need to activate one using calls to the switch_controller service
activate_controller: false

# Name of the topic the target joint states or endeffector positions are published on
target_topic_name: "leader/joint_states"

# If true, the follower controller will use a syncing procedure after activation
sync_after_activation: true

# IP of the robot you want the controller to connect to
robot_ip: "192.168.19.12"

# Set a namespace for your controllers
namespace: ""

# Target position for the move_to_position_controller
reset_position: [0.0, -0.78539816339, 0.0, -2.35619449019, 0.0, -1.57079632679, 0.78539816339]

# Parameters for joint impedance parameters you might to want to tune
k_gains: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
d_gains: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0]

# Select the correct urdf for your Franka robot
urdf_file: "fr3/fr3.urdf.xacro"

# Prefix for the joint names sent to tf2
arm_prefix: ""
```