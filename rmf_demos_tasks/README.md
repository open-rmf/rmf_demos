# rmf\_demo\_tasks

This package provides scripts used in the RMF demos to demonstrate the tasks that may be requested of a robot.

## Flexible Tasks

The new task system allows users to construct and submit their own tasks in a more flexible manner, such as having multiple stops in a loop or specifying a robot for a cleaning task. This package contains some helpful scripts that demonstrate how to compose such tasks:

- **dispatch_patrol**

  This task allows users to dispatch a robot to perform the patrol task. On top of the start and finish waypoints, you can provide a fleet name `-F` and robot name `-R`.

  Example for Office world, dispatch the task to a specific robot.
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -F tinyRobot -R tinyRobot1 -p lounge -n 2 --use_sim_time
  ```

  or multiple places for the robot to travel to:
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -p supplies pantry coe --use_sim_time
  ```

- **dispatch_clean**

  This script submits a cleaning task. The `-cs` flag takes in the desired cleaning zone as an argument. You may also choose to specify a robot to perform the cleaning task by providing a fleet name `-F` and robot name `-R`.

  Example for Hotel world:
  ```
  ros2 run rmf_demos_tasks dispatch_clean -cs clean_lobby -F cleanerBotA -R cleanerBotA_1 --use_sim_time
  ```

- **dispatch_action**

  This script demonstrates how to compose a task with the `PerformAction` teleop feature. You can ask a robot to go to a starting point `-s` to execute an action, `-a` indicates the name of the action.

  Example for Office world. The example below shows an action name of "teleop".
  ```
  ros2 run rmf_demos_tasks dispatch_action -F tinyRobot -R tinyRobot1 -a teleop -s coe --use_sim_time
  ```

  After completing your teleop action, you will need to publish the following message to return control to RMF:
  ```
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  tinyRobot, robot_name: tinyRobot1, mode: {mode: 0}}' --once
  ```

  This script also takes a series of starting points `-s` as arguments and commands a robot to perform the same specified action at the given points.

  Example for Office world. Run a total of 3 "teleop" actions at coe, supplies and pantry waypoints:
  ```
  ros2 run rmf_demos_tasks dispatch_action -a teleop -s coe supplies pantry --use_sim_time
  ```

  The robot will move to each of these places and RMF will relinquish control. Similar to `dispatch_action` earlier, you may perform your desired action (e.g. using the provided `teleop_robot` script) and end the action by publishing a `ModeRequest` to `/action_execution_notice`. The robot will then move to the next point to perform the next action.

- **dispatch_delivery**

  This script allows users to perform deliveries with one or more pickup and drop-off locations. It takes in information about the pickup `-p` and dropoff `-d` activities in the order you submit them. For each activity, you will need to provide the place, handler, payload SKU and quantity separated by a comma.

  Example for Office world:
  ```bash
  ros2 run rmf_demos_tasks dispatch_delivery \
  -p pantry pantry \
  -ph coke_dispenser coke_dispenser_2 \
  -d hardware_2 coe \
  -dh coke_ingestor coke_ingestor_2 \
  -pp coke,1 coke,1 \
  -dp coke,1 coke,1 \
  --use_sim_time
  ```

- **cancel_task**
  Cancel a task by specifying its task ID:
  
  Example: Run dispatch a `patrol` task to the newly launced `office_world`.
  ```
  ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge --use_sim_time
  ```

  Then try cancel the submitted task with the id.
  ```bash
  ros2 run rmf_demos_tasks cancel_task -id patrol.dispatch-0
  ```

## Additional Scripts

- **office_teleop_robot**
  This script demonstrates the capability of `teleop` perform action task. Imagine when the robot executes a teleop action, the user would like to move the robot around. To simulate this behaviour, we will show this scenario in office world.
  
  Launch the Office world and run the above `dispatch_action` task to bring `tinyRobot1` to `coe`. During perform action, the fleetadapter give up control of the robot. To simulate a teleop operation, we will run:
  ```
  ros2 launch rmf_demos office_teleop_robot.launch.xml
  ```
  
  The `office_teleop_robot` script directly controls the robot motion in simulation without going through the fleet adapter.

  If you are running this in Gazebo, you will need to delete the room chair models for the simulation physics to run smoothly.

  You can end the action by publishing a `ModeRequest` via the `/action_execution_notice` topic as demonstrated above.

- **dispatch_go_to_place**
  Similar to `dispatch_patrol`, you can dispatch the robot to go to a specified waypoint. The useful side of `dispatch_go_to_place` is the ability to specify the orientation when the robot reaches the destination, with the arg `-o`. Additionally, you may also choose to specify a robot with `-f` and `-R`.

  Example in office world:
  ```
  ros2 run rmf_demos_tasks  dispatch_go_to_place -p lounge -o 105 --use_sim_time
  ```

## Quality Declaration

This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](./QUALITY_DECLARATION.md) for more details.
