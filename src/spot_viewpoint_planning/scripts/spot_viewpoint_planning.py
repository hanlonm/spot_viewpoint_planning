#!/usr/bin/env python3
import rospy

from spot_msgs.srv import HandPose, HandPoseRequest, HandPoseResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped

class SpotViewpointPlanner:
    def __init__(self) -> None:
        self.test_service = rospy.Service("test_spot_movements", Empty,)
        self.gripper_pose_proxy = rospy.ServiceProxy("/spot/gripper_pose", HandPose)

        self.move_spot_pub = rospy.Publisher("/spot/go_to_pose", PoseStamped)

    def test_srvice_cb(self, req: EmptyRequest):
        gripper_request = HandPoseRequest()
        gripper_request.duration = 10.0
        gripper_request.frame = "body"
        gripper_pose = Pose()
        gripper_pose.position.x = 0.9
        gripper_pose.position.y = 0.1
        gripper_pose.position.z = 0.2
        gripper_pose.orientation.x = 0.0
        gripper_pose.orientation.y = 0.0
        gripper_pose.orientation.z = 0.0
        gripper_pose.orientation.w = 1.0
        gripper_request.pose_point = gripper_pose

        hand_response: HandPoseResponse = self.gripper_pose_proxy.call(gripper_request)

        go_to_pose = PoseStamped()
        go_to_pose.header.frame_id = "body"
        body_pose = Pose()
        body_pose.position.x = 0.5
        body_pose.position.y = 0.0
        body_pose.position.z = 0.0
        body_pose.orientation.x = 0.0
        body_pose.orientation.y = 0.0
        body_pose.orientation.z = 0.0
        body_pose.orientation.w = 1.0
        go_to_pose.pose = body_pose

        self.move_spot_pub.publish(go_to_pose)

        return EmptyResponse()



if __name__ == "__main__":
    rospy.init_node("spot_viewpoint_planner")
    node = SpotViewpointPlanner()
    rospy.spin()


