
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
import json

from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile

from rmf_task_msgs.srv import SubmitTask, GetTaskList, CancelTask
from rmf_task_msgs.msg import TaskDescription, TaskSummary
from rmf_task_msgs.msg import TaskType, Delivery, Loop
from rmf_fleet_msgs.msg import FleetState, RobotMode


###############################################################################


class DispatcherClient(Node):
    def __init__(self):
        super().__init__('api_client')
        self.submit_task_srv = self.create_client(SubmitTask, '/submit_task')
        self.cancel_task_srv = self.create_client(CancelTask, '/cancel_task')
        self.get_tasks_srv = self.create_client(GetTaskList, '/get_tasks')

        qos_profile = QoSProfile(depth=20)

        # to show robot states
        self.fleet_state_subscription = self.create_subscription(
            FleetState, 'fleet_states', self.fleet_state_cb,
            qos_profile=qos_profile)
        self.fleet_states_dict = {}

        self.active_tasks_cache = []
        self.terminated_tasks_cache = []

        # just check one srv endpoint
        while not self.submit_task_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Dispatcher node is not avail, waiting...')

    def fleet_state_cb(self, msg: FleetState):
        fleet_name = msg.name
        self.fleet_states_dict[fleet_name] = msg.robots

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def ros_time(self) -> int:
        return self.get_clock().now().to_msg().sec

    def submit_task_request(self, req_json) -> (str, str):
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID

        Args:
            req_json: task description in json format
        Returns:
            task_id, error_msg: if submission failed
        """

        # Convert a task json to rmf_task_msg form
        print("Submit Task Request!")
        desc_msg, err_msg = self.__convert_task_description(req_json)
        if desc_msg is None:
            return "", err_msg

        req_msg = SubmitTask.Request()
        req_msg.description = desc_msg
        req_msg.requester = "api-server"

        try:
            future = self.submit_task_srv.call_async(req_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                self.get_logger().warn('/submit_task srv call failed')
            elif not response.success:
                self.node.get_logger().error(
                    'Dispatcher node failed to accept task')
            else:
                self.get_logger().info(
                    f'New Dispatch task_id {response.task_id}')
                if response.task_id:
                    return response.task_id, ""
        except Exception as e:
            self.get_logger().error('Error! Submit Srv failed %r' % (e,))
        return "", "Dispatcher server failed in accepting the task"

    def cancel_task_request(self, task_id) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        """
        print("Canceling Task Request!")
        req = CancelTask.Request()
        req.task_id = task_id
        try:
            future = self.cancel_task_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                self.get_logger().warn('/cancel_task srv call failed')
            else:
                self.get_logger().info(
                    f'Cancel Task, success? {response.success}')
                return response.success
        except Exception as e:
            self.get_logger().error('Error! Cancel Srv failed %r' % (e,))
        return False

    def get_task_status(self):
        """
        Get all task status - This fn will trigger a ros srv call to acquire
        all submitted tasks to dispatcher node. Fn returns an object of tasks
        """
        req = GetTaskList.Request()
        try:
            future = self.get_tasks_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                # self.get_logger().debug('/get_tasks srv call failed')
                return self.active_tasks_cache + self.terminated_tasks_cache
            else:
                # self.get_logger().info(f'Get Task, success? \
                #   {response.success}')
                active_tasks = self.__convert_task_status_msg(
                    response.active_tasks, False)
                terminated_tasks = self.__convert_task_status_msg(
                    response.terminated_tasks, True)
                self.active_tasks_cache = active_tasks
                self.terminated_tasks_cache = terminated_tasks
                return active_tasks + terminated_tasks
        except Exception as e:
            self.get_logger().error('Error! GetTasks Srv failed %r' % (e,))
        return []  # empty list

    def get_robot_states(self):
        """
        This function will return an aggregated list of robot states to the
        front end UI when a ajax GET is requested.
        """
        agg_robot_states = []
        for fleet_name, robot_states in self.fleet_states_dict.items():
            robots = self.__convert_robot_states_msg(fleet_name, robot_states)
            agg_robot_states = agg_robot_states + robots
        return agg_robot_states

###############################################################################

    def __convert_task_status_msg(self, task_summaries, is_done=True):
        """
        convert task summary msg and return a jsonify-able task status obj
        """
        states_enum = {
            TaskSummary.STATE_QUEUED: "Queued",
            TaskSummary.STATE_ACTIVE: "Active/Executing",
            TaskSummary.STATE_COMPLETED: "Completed",
            TaskSummary.STATE_FAILED: "Failed",
            TaskSummary.STATE_CANCELED: "Cancelled",
            TaskSummary.STATE_PENDING: "Pending",
        }
        type_enum = {
            TaskType.TYPE_STATION: "Station",
            TaskType.TYPE_LOOP: "Loop",
            TaskType.TYPE_DELIVERY: "Delivery",
            TaskType.TYPE_CHARGE_BATTERY: "Charging",
            TaskType.TYPE_CLEAN: "Clean",
            TaskType.TYPE_PATROL: "Patrol",
        }

        status_list = []
        rclpy.spin_once(self, timeout_sec=0.0)
        now = self.get_clock().now().to_msg().sec  # only use sec
        for task in task_summaries:
            desc = task.task_profile.description
            status = {}
            status["task_id"] = task.task_id
            status["state"] = states_enum[task.state]
            status["done"] = is_done
            status["fleet_name"] = task.fleet_name
            status["robot_name"] = task.robot_name
            status["task_type"] = type_enum[desc.task_type.type]
            status["priority"] = desc.priority.value
            status["submited_start_time"] = desc.start_time.sec
            status["start_time"] = task.start_time.sec     # only use sec
            status["end_time"] = task.end_time.sec         # only use sec

            if status["task_type"] == "Clean":
                status["description"] = desc.clean.start_waypoint
            elif status["task_type"] == "Loop":
                status["description"] = desc.loop.start_name + " --> " + \
                    desc.loop.finish_name + " x" + str(desc.loop.num_loops)
            elif status["task_type"] == "Delivery":
                status["description"] = desc.delivery.pickup_place_name + \
                    " --> " + desc.delivery.dropoff_place_name
            elif status["task_type"] == "Charging":
                status["description"] = "Back to Charging Station"

            # Current hack to generate a progress percentage
            duration = abs(task.end_time.sec - task.start_time.sec)
            # TOD0: is done shouldnt be indicative at all
            if is_done or task.state == TaskSummary.STATE_COMPLETED:
                status["progress"] = "100%"
            elif (duration == 0 or
                  task.state == TaskSummary.STATE_QUEUED or
                  task.state == TaskSummary.STATE_CANCELED):
                status["progress"] = "0%"
            else:
                percent = int(100*(now - task.start_time.sec)/float(duration))
                if (percent < 0):
                    status["progress"] = "0%"
                elif (percent > 100):
                    status["progress"] = "Delayed"
                else:
                    status["progress"] = f"{percent}%"
            status_list.insert(0, status)  # insert front
        return status_list

    def __get_robot_assignment(self, robot_name):
        assigned_tasks = []
        assigned_task_ids = []
        for task in self.active_tasks_cache:
            if task["robot_name"] == robot_name:
                assigned_tasks.append(task)
        assigned_tasks.sort(key=lambda x: x.get('start_time'))
        for task in assigned_tasks:
            assigned_task_ids.append(task["task_id"])
        return assigned_task_ids

    def __convert_robot_states_msg(self, fleet_name, robot_states):
        """
        convert robot states msg to a jsonify-able robot_states
        """
        bots = []
        mode_enum = {
            RobotMode.MODE_IDLE: "Idle-0",
            RobotMode.MODE_CHARGING: "Charging-1",
            RobotMode.MODE_MOVING: "Moving-2",
            RobotMode.MODE_PAUSED: "Paused-3",
            RobotMode.MODE_WAITING: "Waiting-4",
            RobotMode.MODE_EMERGENCY: "Emengency-5",
            RobotMode.MODE_GOING_HOME: "GoingHome-6",
            RobotMode.MODE_DOCKING: "Dock/Clean-7",
            RobotMode.MODE_ADAPTER_ERROR: "AdpterError-8"
        }
        for bot in robot_states:
            state = {}
            state["robot_name"] = bot.name
            state["fleet_name"] = fleet_name
            state["mode"] = mode_enum[bot.mode.mode]
            state["battery_percent"] = bot.battery_percent
            state["location_x"] = bot.location.x
            state["location_y"] = bot.location.y
            state["location_yaw"] = bot.location.yaw
            state["level_name"] = bot.location.level_name
            state["assignments"] = self.__get_robot_assignment(bot.name)
            bots.append(state)
        return bots

    def __convert_task_description(self, task_json):
        """
        Convert a json task req format to rmf_task_msgs/TaskDescription.
        :note: The 'start time' here is the "Duration" from now.
        """
        task_desc = TaskDescription()
        print(task_json)

        try:
            if (("task_type" not in task_json) or
                ("start_time" not in task_json) or
                    ("description" not in task_json)):
                raise Exception("Key value is incomplete")

            if ("priority" in task_json):
                priority = int(task_json["priority"])
                if (priority < 0):
                    raise Exception("Priority value is less than 0")
                task_desc.priority.value = priority
            else:
                task_desc.priority.value = 0

            desc = task_json["description"]
            if task_json["task_type"] == "Clean":
                task_desc.task_type.type = TaskType.TYPE_CLEAN
                task_desc.clean.start_waypoint = desc["cleaning_zone"]
            elif task_json["task_type"] == "Loop":
                task_desc.task_type.type = TaskType.TYPE_LOOP
                loop = Loop()
                loop.num_loops = int(desc["num_loops"])
                loop.start_name = desc["start_name"]
                loop.finish_name = desc["finish_name"]
                task_desc.loop = loop
            elif task_json["task_type"] == "Delivery":
                task_desc.task_type.type = TaskType.TYPE_DELIVERY
                delivery = Delivery()
                delivery.pickup_place_name = desc["pickup_place_name"]
                delivery.pickup_dispenser = desc["pickup_dispenser"]
                delivery.dropoff_ingestor = desc["dropoff_ingestor"]
                delivery.dropoff_place_name = desc["dropoff_place_name"]
                task_desc.delivery = delivery
            else:
                raise Exception("Invalid TaskType")

            # Calc start time, convert min to sec: TODO better representation
            rclpy.spin_once(self, timeout_sec=0.0)
            ros_start_time = self.get_clock().now().to_msg()
            ros_start_time.sec += int(task_json["start_time"]*60)
            task_desc.start_time = ros_start_time
        except KeyError as ex:
            return None, f"Missing Key value in json body: {ex}"
        except Exception as ex:
            return None, str(ex)
        return task_desc, ""
