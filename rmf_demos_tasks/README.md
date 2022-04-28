# rmf\_demo\_tasks

This package provides scripts used in the RMF demos to demonstrate the tasks that may be requested of a robot.

## Flexible Tasks

The new task system allows users to construct and submit their own tasks in a more flexible manner, such as having multiple stops in a loop or specifying a robot for a cleaning task. This package contains some helpful scripts that demonstrate how to compose such tasks:
- `dispatch_patrol`

  This task is similar to `dispatch_patrol`, but it allows users to specify a robot to perform the patrol task. On top of the start and finish waypoints, you will need to provide a fleet name `-F` and robot name `-R`.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -F tinyRobot -R tinyRobot1 -p lounge -n 2 --use_sim_time
  ```

  or multiple places for the robot to travel to:
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -p supplies pantry coe --use_sim_time
  ```
- `dispatch_clean`

  This script submits a cleaning task. The `-cs` flag takes in the desired cleaning zone as an argument. You may also choose to specify a robot to perform the cleaning task by providing a fleet name `-F` and robot name `-R`.

  Example for Hotel world:
  ```
  ros2 run rmf_demos_tasks dispatch_clean -cs clean_lobby -F cleanerBotA -R cleanerBotA_1 --use_sim_time
  ```
- `dispatch_action`

  This script demonstrates how to compose a task with the `PerformAction` teleop feature. You can ask a robot to go to a starting point `-s` to execute an action, `-a` indicates the name of the action (in this case the name is "teleop").

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_action -F tinyRobot -R tinyRobot1 -a teleop -s coe --use_sim_time
  ```

  After completing your teleop action, you will need to publish the following message to return control to RMF:
  ```
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  tinyRobot, robot_name: tinyRobot1, mode: {mode: 0}}' --once
  ```

  This script also takes a series of starting points `-s` as arguments and commands a robot to perform multiple actions at the given points.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_action -a teleop -s coe supplies pantry --use_sim_time
  ```

  The robot will move to each of these places and RMF will relinquish control. Similar to `dispatch_action` earlier, you may perform your desired action (e.g. using the provided `teleop_action` script) and end the action by publishing a `ModeRequest` to `/action_execution_notice`. The robot will then move to the next point to perform the next action.
- `dispatch_delivery`

  This script allows users to perform multiple deliveries at one go. It takes in information about the pickup `-p` and dropoff `-d` activities in the order you submit them. For each activity, you will need to provide the place, handler, payload SKU and quantity separated by a comma.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_delivery -p pantry,coke_dispenser,coke,1 pantry,coke_dispenser_2,coke,1 -d hardware_2,coke_ingestor,coke,1 coe,coke_ingestor_2,coke,1 --use_sim_time
- `dispatch_patrol`

  This script takes in multiple places `-p` for the robot to travel to.

  Example for Office world:
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -p supplies pantry coe --use_sim_time
  ```

- `cancel_task`
  Cancel a task by specifying its task ID:
  
  Example: Run dispatch a `patrol` task to the newly launced `office_world`.
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge --use_sim_time
  ```

  Then try cancel the submitted task with the id.
  ```bash
  ros2 run rmf_demos_tasks dispatch_action -id patrol.dispatch-0
  ```

## Additional Scripts

- `teleop_action`

  This script demonstrates sending a teleop action with a particular set of coordinates `-p` you would like your robot to follow in a given map `-m`. To test it, launch the Office world and run the above `dispatch_action` task to bring `tinyRobot1` to `coe`. Then,
  ```
  ros2 run rmf_demos_tasks teleop_action -F tinyRobot -R tinyRobot1 -p 5.3,-4.9 3.3,-8.9 5.3,-9.9 7.3,-6.1 5.3,-4.9 -m L1
  ```
  If you are running this in Gazebo, you will need to delete the room chair models for the simulation physics to run smoothly.

  You can end the action by publishing a `ModeRequest` via the `/action_execution_notice` topic as demonstrated above.

## Quality Declaration

This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](./QUALITY_DECLARATION.md) for more details.
