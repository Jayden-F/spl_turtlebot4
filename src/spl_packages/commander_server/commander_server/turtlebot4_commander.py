#!/usr/bin/python3

import argparse
import json
import os
import time
from math import cos, radians, sin
from typing import Tuple

import rclpy
import requests
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import (
    NavigateToPose_FeedbackMessage, NavigateToPose_GetResult_Response,
    NavigateToPose_SendGoal_Response)
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

X = Y = Theta = float
Position = Tuple[X, Y, Theta]


class Turtlebot4_Commander(Node):
    def __init__(
        self,
        turtlebot4_id: int = 0,
        hostName: str = "192.168.0.204",
        serverPort: int = 8080,
    ):
        super().__init__("turtlebot4_commander")

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self._amclPoseCallback,
            amcl_pose_qos,
        )
        self.turtlebot4_id: int = turtlebot4_id
        self.hostName: str = hostName
        self.serverPort: int = serverPort
        self.isExecuting: bool = False
        self.pose: PoseStamped = None
        self.timestep: int = 0
        print(
            f"Starting Commander Server:\nRobot ID: {self.turtlebot4_id}\nWebserver: {hostName}:{serverPort}"
        )
        self.run()

    def run(self):

        while True:
            if self.isExecuting:
                time.sleep(0.2)
                continue

            data = self.get_request()
            position = data.get("position")
            self.timestep = data.get("timestep")

            pose: PoseStamped = self.getPoseStamped(position)
            self.send_goal(pose)

    def get_request(self):
        """Get the next position to navigate to."""
        url = f"http://{self.hostName}:{self.serverPort}"
        params = {"agent_id": self.turtlebot4_id}

        r = None
        while r is None:
            try:
                r = requests.get(url, params=params)
            except requests.ConnectionError:
                r = None

        return json.loads(r.content.decode())

    def post_request(self, position: PoseStamped, status: str) -> None:
        """Post the current position to the server."""
        url = f"http://{self.hostName}:{self.serverPort}"

        data = {
            "agent_id": self.turtlebot4_id,
            "position": [
                position.pose.position.x,
                position.pose.position.y,
                position.pose.orientation.z,
            ],
            "status": status,
            "timestep": self.timestep,
        }

        json_data = json.dumps(data)

        while True:
            try:
                requests.post(url, data=json_data)
            except requests.ConnectionError:
                continue
            break

    def getPoseStamped(self, position: Position) -> PoseStamped:
        """
        Fill and return a PoseStamped message.

        :param position: A list consisting of the x and y positions for the Pose. e.g [0.5, 1.2]
        :param rotation: Rotation of the pose about the Z axis in degrees.
        :return: PoseStamped message
        """
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        # Convert Z rotation to quaternion
        pose.pose.orientation.z = sin(radians(position[2]) / 2)
        pose.pose.orientation.w = cos(radians(position[2]) / 2)

        return pose

    def send_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            "Navigating to goal: "
            + str(pose.pose.position.x)
            + " "
            + str(pose.pose.position.y)
            + "..."
        )

        self.nav_to_pose_client.wait_for_server()

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle: NavigateToPose_SendGoal_Response = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self.isExecuting = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, msg: NavigateToPose_FeedbackMessage):
        self.get_logger().debug("Received action feedback message")
        feedback = msg.feedback
        self.pose = feedback.current_pose
        position = feedback.current_pose.pose._position
        orientation = feedback.current_pose.pose.orientation
        eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
        eda = feedback.distance_remaining

        self.get_logger().info(
            "Pose: "
            + str([position.x, position.y])
            + " eta: "
            + str(eta)
            + " eda: "
            + str(eda)
        )
        self.post_request(self.pose, "Executing")

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
        self.get_logger().info("Goal Status: " + status_string)

        self.isExecuting = False;

    def _amclPoseCallback(self, msg):
        self.post_request(msg.pose, "Succeeded")
        self.destroy_subscription(self.localization_pose_sub )

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=0)
    args = parser.parse_args()
    rclpy.init(args=None)
    commander = Turtlebot4_Commander(args.id)
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        print("Shutting Down")

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
