## Unreleased

- Updated parameter name `arm_id` to `robot_type` to stay compatible with newer `franka_ros2` versions
- removed some arguments given to `franka.launch.py` since they are no longer required in the new version of `franka_ros2`
- During container builds, the correct dependecies of `franka_ros2` are determined automatically
- **Breaking Change** Going forward, this repo will only support `franka_ros2` with versions `v3.3.0+` (`jazzy`branch) and `v2.4.0+` (`humble` branch)


## [0.2.0] - 2026-05-27

- Add controller state publishing (syncing, following, inactive)
- Fix namespace passing to gravity controller and avoid hardcoded arm_id
- Add multi-robot support to follower launch file
- Update follower_controllers.yaml control gains

## [0.1.0] - 2026-02-24

Initial release of franka_follower_controllers package
Includes the following features:
    - dev and application container
    - joint_follower_controller