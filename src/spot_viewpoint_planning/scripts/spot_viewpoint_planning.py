#!/usr/bin/env python3
import rospy
import random

from spot_msgs.srv import HandPose, HandPoseRequest, HandPoseResponse, ArmJointMovement, ArmJointMovementRequest, ArmJointMovementResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped

class SpotViewpointPlanner:
    def __init__(self) -> None:
        
        self.test_service = rospy.Service("test_spot_movements", Empty, self.test_srvice_cb)
        self.gripper_pose_proxy = rospy.ServiceProxy("/spot/gripper_pose", HandPose)

        self.move_spot_pub = rospy.Publisher("/spot/go_to_pose", PoseStamped)
        rospy.loginfo("Spot viewpoint planner running")

        self.base_arm_pose_request = HandPoseRequest()
        self.base_arm_pose_request.duration = 0.0
        self.base_arm_pose_request.frame = "body"
        self.base_arm_pose = Pose()
        self.base_arm_pose.position.x = 0.3
        self.base_arm_pose.position.y = 0.0
        self.base_arm_pose.position.z = 0.7
        self.base_arm_pose.orientation.x = 0.0
        self.base_arm_pose.orientation.y = 0.0
        self.base_arm_pose.orientation.z = 0.0
        self.base_arm_pose.orientation.w = 1.0
        self.base_arm_pose_request.pose_point = self.base_arm_pose

        
        self.arm_angle_proxy = rospy.ServiceProxy("/spot/arm_joint_move", ArmJointMovement)
        self.base_arm_angles = [0.0, -2.8629910945892334, 1.8224114179611206, 0.0, 1.1237012147903442, 0.0]
        self.base_arm_angles_request = ArmJointMovementRequest()
        self.base_arm_angles_request.joint_target = self.base_arm_angles


        self.map_poses = [[1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          [-1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          ]
        self.pose_it = 0

        

    def test_srvice_cb(self, req: EmptyRequest):
        # gripper_request = HandPoseRequest()
        # gripper_request.duration = 10.0
        # gripper_request.frame = "body"
        # gripper_pose = Pose()
        # gripper_pose.position.x = 0.9
        # gripper_pose.position.y = 0.1
        # gripper_pose.position.z = 0.2
        # gripper_pose.orientation.x = 0.0
        # gripper_pose.orientation.y = 0.0
        # gripper_pose.orientation.z = 0.0
        # gripper_pose.orientation.w = 1.0
        # gripper_request.pose_point = gripper_pose

        # hand_response: HandPoseResponse = self.gripper_pose_proxy.call(gripper_request)

        # go_to_pose = PoseStamped()
        # go_to_pose.header.frame_id = "body"
        # body_pose = Pose()
        # body_pose.position.x = 0.5
        # body_pose.position.y = 0.0
        # body_pose.position.z = 0.0
        # body_pose.orientation.x = 0.0
        # body_pose.orientation.y = 0.0
        # body_pose.orientation.z = 0.0
        # body_pose.orientation.w = 1.0
        # go_to_pose.pose = body_pose

        # self.move_spot_pub.publish(go_to_pose)
        self.go_to_map_pose()

        return EmptyResponse()
    
    def go_to_map_pose(self):
        if self.pose_it == len(self.map_poses)-1:
            self.pose_it = 0
        pose_msg = self.pq_to_pose_msg(self.map_poses[self.pose_it])
        go_to_pose = PoseStamped()
        go_to_pose.header.frame_id = "body"
        go_to_pose.pose = pose_msg

        arm_angles = self.base_arm_angles
        arm_angles[0] = arm_angles[0] + random.uniform(-0.8, 0.8)
        arm_angles[4] = arm_angles[4] + random.uniform(-0.2, 0.2)
        arm_angles_req = ArmJointMovementRequest()
        arm_angles_req.joint_target = arm_angles
        self.arm_angle_proxy.call(arm_angles_req)

        self.pose_it += 1




    def pq_to_pose_msg(self, pq):
        pose = Pose()
        pose.position.x = pq[0]
        pose.position.y = pq[1]
        pose.position.z = pq[2]
        pose.orientation.x = pq[4]
        pose.orientation.y = pq[5]
        pose.orientation.z = pq[6]
        pose.orientation.w = pq[3]

        return pose




if __name__ == "__main__":
    rospy.init_node("spot_viewpoint_planner")
    node = SpotViewpointPlanner()
    rospy.spin()


