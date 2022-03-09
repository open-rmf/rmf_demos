# rmf\_demo\_tasks

This package provides scripts used in the RMF demos to demonstrate the tasks that may be requested of a robot.

## Legacy Tasks

This package offers some task scripts that can be used with `rmf_demos` simulation robots. To run these tasks:
- `dispatch_clean`

  This script submits a cleaning task. The `-cs` flag takes in the desired cleaning zone as an argument.

  Example for Hotel world:
  ```
  ros2 run rmf_demos_tasks dispatch_clean -cs clean_lobby --use_sim_time
  ```
- `dispatch_delivery`

  This script submits a delivery task.
    - `-p` flag takes in the start waypoint for the delivery pickup
    - `-pd` flag takes in the payload dispenser
    - `-d` flag takes in the finish waypoint for the delivery dropoff
    - `-di` flag takes in the payload ingestor

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_delivery -p pantry -pd coke_dispenser -d hardware_2 -di coke_ingestor --use_sim_time
  ```
- `dispatch_loop`

  This script submits a loop task. It takes in a start `-s` and finish `-f` waypoint, as well as an optional loop number `-n` that defaults to 1.

  Example for Clinic world:
  ```
  ros2 run rmf_demos_tasks dispatch_loop -s L1_right_nurse_center -f L1_left_nurse_center -n 2 --use_sim_time
  ```

Do remember to add the `--use_sim_time` flag when running these tasks in simulation.

## Flexible Tasks

The new task system allows users to construct and submit their own tasks in a more flexible manner, such as having multiple stops in a loop or cleaning task. This package contains some helpful scripts that demonstrate how to compose such tasks:
- `direct_patrol`

  This task is similar to `dispatch_loop`, but it allows users to specify a robot to perform the patrol task. On top of the start and finish waypoints, you will need to provide a fleet name `-F` and robot name `-R`.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks direct_patrol -F tinyRobot -R tinyRobot1 -s pantry -f lounge -n 2 --use_sim_time
  ```
- `dispatch_multi_clean`

  This script provides users the option of requesting a specific robot to clean multiple cleaning zones within a single task. You will need to specify the desired cleaning zones `-z`.

  Example for Hotel world:
  ```
  ros2 run rmf_demos_tasks dispatch_multi_clean -F cleanerBotA -R cleanerBotA_2 -z clean_lobby clean_waiting_area --use_sim_time
  ```
- `dispatch_multi_delivery`

  This script allows users to perform multiple deliveries at one go. It takes in information about the pickup `-p` and dropoff `-d` activities in the order you submit them. For each activity, you will need to provide the place, handler, payload SKU and quantity separated by a comma.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_multi_delivery -p pantry,coke_dispenser,coke,1 -d hardware_2,coke_ingestor,coke,1 --use_sim_time
- `dispatch_multi_stop`

  This script takes in multiple places `-p` for the robot to travel to.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_multi_stop -p supplies pantry coe --use_sim_time
  ```
- `dispatch_teleop`

  This script demonstrates how to compose a task with the `PerformAction` feature. You can ask a robot to go to a starting point `-s` to execute an action, then return control to RMF when the action is completed.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_teleop -F tinyRobot -R tinyRobot1 -s coe --use_sim_time
  ```
- `teleop_action`

  This script demonstrates sending a teleop action with a particular set of coordinates you would like your robot to follow. To test it, launch the Office world and run the above command for `dispatch_teleop` to bring `tinyRobot1` to `coe`. Then,
  ```
  ros2 run rmf_demos_tasks teleop_action -F tinyRobot -R tinyRobot1
  ```
  If you are running this in Gazebo, you will need to delete the room chair models for the simulation physics to run smoothly.

  When the robot is done, you can end the teleop action by publishing the following message:
  ```
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  tinyRobot, robot_name: tinyRobot1, mode: {mode: 0}}' --once
  ```

## Quality Declaration

This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](./QUALITY_DECLARATION.md) for more details.
