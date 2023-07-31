import rclpy
import requests
import json
from typing import Tuple
from math import sin, cos, radians
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage, NavigateToPose_GetResult_Response, NavigateToPose_SendGoal_Response
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus



X = Y = Theta = float
Position = Tuple[X, Y, Theta]


class Turtlebot4_Commander(BasicNavigator):

    def __init__(self, 
                 turtlebot4_id: int = 0,
                 hostName: str = "192.168.0.204",
                 serverPort: int = 8080
                ):
    
        super().__init__("turtlebot4_commander")

        # self.timer = self.create_timer(0.5, self.commander)

        self.turtlebot4_id: int = turtlebot4_id
        self.hostName: str = hostName
        self.serverPort: int = serverPort
        self.isExecuting: bool = False
        self.pose: PoseStamped = None

        self.commander()


    def commander(self):

        while True:

            if self.isExecuting:

        
            data = self.get_request()
            position = data.get("position")
            pose = self.getPoseStamped(position)
            self.send_goal(pose)
        

    def get_request(self):
        """Get the next position to navigate to."""
        url = f"http://{self.hostName}:{self.serverPort}"
        params = {"agent_id": self.turtlebot4_id}

        r = requests.get(url, params=params)

        return json.loads(r.content.decode())
    
    def post_request(self, position: PoseStamped, status: str) -> None:
        """Post the current position to the server."""
        url = f"http://{self.hostName}:{self.serverPort}"

        data = {"agent_id": self.turtlebot4_id,
                "position": [position],
                "status": status}
        
        json_data = json.dumps(data)

        r = requests.post(url, data=json_data)


    def getPoseStamped(self, position: Position) -> PoseStamped:
        """
        Fill and return a PoseStamped message.

        :param position: A list consisting of the x and y positions for the Pose. e.g [0.5, 1.2]
        :param rotation: Rotation of the pose about the Z axis in degrees.
        :return: PoseStamped message
        """
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        # Convert Z rotation to quaternion
        pose.pose.orientation.z = sin(radians(position[2]) / 2)
        pose.pose.orientation.w = cos(radians(position[2]) / 2)

        return pose
        
    def send_goal(self, pose: Position):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
            str(pose.pose.position.y) + '...')
        
        self.isExecuting =  True

        self.nav_to_pose_client.wait_for_server()

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle: NavigateToPose_SendGoal_Response = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.isExecuting = False
            return

        self.get_logger().info('Goal accepted :)')
        

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def feedback_callback(self, msg: NavigateToPose_FeedbackMessage):

        self.debug('Received action feedback message')
        feedback = msg.feedback
        self.pose = feedback.current_pose
        position = feedback.current_pose.pose._position
        orientation = feedback.current_pose.pose.orientation
        eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
        eda = feedback.distance_remaining
        
        self.info("Pose: " + str([position.x, position.y]) + " eta: " + str(eta) + " eda: " + str(eda))
        self.post_request(self.pose, "executing")


    def get_result_callback(self, future):

        reponse: NavigateToPose_GetResult_Response = future.result()
        status = reponse.status
        
        status_string = None
        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                status_string = "Succeeded"
            case GoalStatus.STATUS_ABORTED:
                status_string = "Aborted"
            case GoalStatus.STATUS_CANCELED:
                status_string = "Cancelled"
            case GoalStatus.STATUS_UNKNOWN:
                status_string = "Unknown"

        self.post_request(self.pose, status_string)
        self.info("Goal Status: " + status_string)
           


def main(args=None):

    rclpy.init(args=args)
    commander = Turtlebot4_Commander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        print("Shutting Down")

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()