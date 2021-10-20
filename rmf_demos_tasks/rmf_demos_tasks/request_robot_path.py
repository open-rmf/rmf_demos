#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import yaml
import time
import argparse
import math
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import PathRequest, Location

# Simple script to find a path to some waypoint
# Only for moving robots/vehicles without relying on a task paradigm
# Should not be used in production
class RobotPathRequester:

    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-p', '--package_name',
                            help='Name of package to look for navgraph files')
        parser.add_argument('-n', '--nav_graph_file', help='Navgraph yaml')
        parser.add_argument('-r', '--robot_name', help='Name of robot')
        parser.add_argument('-s', '--start', help='Starting waypoint (label)')
        parser.add_argument('-e', '--end', help='Ending waypoint (label)')
        parser.add_argument('-i', '--task-id', help='Task ID', default='',
                            type=str)

        args = parser.parse_args(argv[1:])
        self.robot_name = args.robot_name
        self.start_wp = args.start
        self.end_wp = args.end
        self.task_id = args.task_id
        self.package_share_dir = get_package_share_directory(args.package_name)
        self.nav_graph_file = args.nav_graph_file

        self.node = rclpy.create_node('robot_path_requester_node')
        self.publisher = self.node.create_publisher(
            PathRequest, 'robot_path_requests', 10)

    def main(self):
        nav_graph_filename = self.package_share_dir + '/' + self.nav_graph_file
        print("Loading nav_graph_filename: " + nav_graph_filename)

        nav_graph_yaml = None
        with open(nav_graph_filename) as f:
            nav_graph_yaml = yaml.load(f)

        levels_yaml = nav_graph_yaml['levels']
        L1_yaml = levels_yaml['L1']
        lanes_yaml = L1_yaml['lanes']

        vertices_yaml = L1_yaml['vertices']
        start_vertex_index = -1
        end_vertex_index = -1
        index = 0

        for vertex in vertices_yaml:
            vertex_metadata = vertex[2]
            if vertex_metadata['name'] is not None:
                if vertex_metadata['name'] == self.start_wp:
                    start_vertex_index = index
                if vertex_metadata['name'] == self.end_wp:
                    end_vertex_index = index
            index = index + 1

        print("searching path from {} to {}".format(
            start_vertex_index, end_vertex_index))
        # do an exhaustive search
        routes = []
        for lane in lanes_yaml:
            if lane[0] == start_vertex_index:
                routes.append([lane[0], lane[1]])

        complete_routes = []
        while len(routes) != 0:
            route = routes[-1]
            routes.pop()
            route_last_wp_idx = route[-1]

            if route_last_wp_idx == end_vertex_index:
                complete_routes.append(route)
                break

            # search for any connectors that haven't been in the route
            for lane in lanes_yaml:
                if lane[0] == route_last_wp_idx:
                    candidate_wp = lane[1]
                    # print("considering: {} {}".format(lane[0], lane[1]))
                    if candidate_wp == end_vertex_index:
                        route.append(candidate_wp)
                        complete_routes.append(route)
                        break
                    else:
                        next_wp_backtracks = False
                        for wp in route:
                            if wp == candidate_wp:
                                # print("hit backtrack!")
                                next_wp_backtracks = True
                                break
                        if next_wp_backtracks is False:
                            new_route = route.copy()
                            new_route.append(candidate_wp)

                            print("new route: ")
                            print(new_route)
                            routes.append(new_route)
                            print(len(routes))

        print("full route:")
        print(complete_routes)

        if len(complete_routes) == 0:
            print("could not find route!")
            return
        final_route = complete_routes[0]

        path_request = PathRequest()
        path_request.fleet_name = ""
        path_request.robot_name = self.robot_name
        path_request.task_id = self.task_id

        index = 0
        for wp in final_route:
            print(vertices_yaml[wp][2]['name'])
            
            location = Location()
            location.x = vertices_yaml[wp][0]
            location.y = vertices_yaml[wp][1]
            
            if index < (len(final_route) - 1):
                next_wp = final_route[index + 1]
                next_location_x = vertices_yaml[next_wp][0]
                next_location_y = vertices_yaml[next_wp][1]

                diff_x = next_location_x - location.x
                diff_y = next_location_y - location.y
                diff_len = math.sqrt(diff_x * diff_x + diff_y * diff_y)
                location.yaw = math.atan2(diff_y / diff_len, diff_x / diff_len)
            else:
                prev_location = path_request.path[-1]
                # location.yaw = prev_location.yaw
                location.yaw = -1.57 # test only

            location.level_name = "L1"
            path_request.path.append(location)
            index += 1

        time.sleep(0.5)
        self.publisher.publish(path_request)
        time.sleep(0.5)
        rclpy.shutdown()

        self.node.get_logger().info(
          'Robot path request between {} (#{}) and {} (#{}), submitted to {}'
          .format(self.start_wp, start_vertex_index,
                  self.end_wp, end_vertex_index,
                  path_request.robot_name))


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    path_requester = RobotPathRequester(args_without_ros)
    path_requester.main()


if __name__ == '__main__':
    main(sys.argv)

