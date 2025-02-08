import rclpy, sys, os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints

class ClientFollowPoints(Node):

    def __init__(self):
        super().__init__('client_follow_points')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')

    def send_points(self, points):
        msg = FollowWaypoints.Goal()
        msg.poses = points

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("-----------------------------")
            print("arrived at goal position!!!!!")
            print("-----------------------------")
            sys.exit(1)
        
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Error: No result value provided.")
        sys.exit(1)
    
    try:
        result = int(sys.argv[1])
    except ValueError:
        print("Error: Invalid result value.")
        sys.exit(1)
    
    # result 값에 따른 위치 설정
    waypoints = {
        0: (0.0, 0.0),
        1: (1.2, -0.0),
        2: (0.65, -0.5),
        3: (-0.6, 0.0),
        4: (-1.1, -0.15),
        5: (-1.8, 0.2)
    }
    
    value_x, value_y = waypoints.get(result, (0.0, 0.0))  # 기본값 (0,0) 설정
    
    node = ClientFollowPoints()
    print('client inited')

    rgoal = PoseStamped()
    rgoal.header.frame_id = "map"
    rgoal.header.stamp.sec = 0
    rgoal.header.stamp.nanosec = 0
    rgoal.pose.position.z = 0.0
    rgoal.pose.position.x = value_x
    rgoal.pose.position.y = value_y
    rgoal.pose.orientation.w = 1.0
    print(rgoal)
    mgoal = [rgoal]
    
    node.send_points(mgoal)

    rclpy.spin(node)

if __name__ == "__main__":
    main()
