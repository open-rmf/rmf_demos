import uuid
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.time import Time
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from rmf_dispenser_msgs.msg import DispenserRequest
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_dispenser_msgs.msg import DispenserState

class Dispenser(Node):
  def __init__(self):
    super().__init__('dispenser_node')
    self.get_logger().info('Dispenser node started...')

    self.dispenser_state = DispenserState()
    # self.dispenser_state.guid = str(uuid.uuid1())
    name = self.declare_parameter('dispenser_name').value
    if not name:
      raise Exception('[dispenser_name] parameter is not specified for mock dispenser')

    print(f'Launching a mock dispenser with the name [{name}]')

    self.dispenser_state.guid = name
    self.dispenser_state.mode = self.dispenser_state.IDLE

    self.responses = {}
    self.finish_timer = None
    self.count = 0
    self.timer_period = 1.0

    self.timer = self.create_timer(
        self.timer_period,
        self.timer_cb)

    self.create_subscription(
        DispenserRequest,
        'dispenser_requests',
        self.dispenser_requests_cb,
        10)

    self.dispenser_results_pub = self.create_publisher(
        DispenserResult,
        'dispenser_results',
        qos_profile = qos_profile_system_default)

    self.dispenser_states_pub = self.create_publisher(
        DispenserState,
        'dispenser_states',
        qos_profile = qos_profile_system_default)


  def publish_response(self, request_guid):
    response = self.responses[request_guid]
    response.time = self.get_clock().now().to_msg()
    self.dispenser_results_pub.publish(response)


  def finish_request(self):
      finished_request_guid = self.dispenser_state.request_guid_queue.pop(0)
      print(f'Finished dispenser request [{finished_request_guid}]')
      self.responses[finished_request_guid].status = DispenserResult.SUCCESS
      self.publish_response(finished_request_guid)

      if not self.dispenser_state.request_guid_queue:
        self.finish_timer.cancel()
        self.finish_timer = None
        return

      print(f'Beginning dispenser request [{self.dispenser_state.request_guid_queue[0]}]')


  def dispenser_requests_cb(self, msg):

    if msg.target_guid != self.dispenser_state.guid:
      return

    if msg.request_guid in self.responses.keys():
      print(f'Republishing result for [{msg.request_guid}]')
      self.publish_response(msg.request_guid)
      return

    print(f'Registering request [{msg.request_guid}]')
    # cache new request
    self.dispenser_state.request_guid_queue.append(msg.request_guid)

    response = DispenserResult()
    response.request_guid = msg.request_guid
    response.source_guid = self.dispenser_state.guid
    response.status = response.ACKNOWLEDGED

    self.responses[msg.request_guid] = response

    if not self.finish_timer:
      print(f'Beginning dispenser request [{msg.request_guid}]')
      self.finish_timer = self.create_timer(5, self.finish_request)


  def timer_cb(self):
    if not self.dispenser_state.request_guid_queue:
      self.dispenser_state.mode = self.dispenser_state.IDLE
    else:
      self.dispenser_state.mode = self.dispenser_state.BUSY

    time = self.get_clock().now()
    self.dispenser_state.time = time.to_msg()
    self.dispenser_states_pub.publish(self.dispenser_state)

def main(args=None):
  rclpy.init(args=args)
  dispenser = Dispenser()
  rclpy.spin(dispenser)
  rclpy.shutdown()

if __name__ == "__main__":
  main(sys.argv)
