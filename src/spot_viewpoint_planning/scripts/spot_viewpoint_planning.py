#!/usr/bin/env python3
import rospy
import numpy as np
import random
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr


from spot_msgs.srv import HandPose, HandPoseRequest, HandPoseResponse, ArmJointMovement, ArmJointMovementRequest, ArmJointMovementResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import tf2_ros


class SpotViewpointPlanner:
    def __init__(self) -> None:
        
        self.test_service = rospy.Service("query_viewpoint", Empty, self.query_viewpoint_cb)
        self.next_wp_service = rospy.Service("next_waypoint", Empty, self.next_waypoint_cb)
        self.home_arm_service = rospy.Service("home_arm", Empty, self.home_arm_cb)
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


        self.body_poses = [[1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          [-1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                          ]
        self.map_poses = [[11.84244155883789, -2.155806064605713, 0.0, 0.7114715562211398, 0.0, 0.0, 0.7027148957352971],
                          [11.389702796936035, -1.5879647731781006, 0.0, 0.005799954052785274, 0.0, 0.0, -0.9999831801250387],
                          [11.224780082702637, -4.262598514556885, 0.0, 0.7292919090780428, 0.0, 0.0, -0.6842026829480455],
                          [12.032266616821289, -4.588181495666504, 0.0, 0.9983694875169599, 0.0, 0.0, 0.05708210223111083],
                          ]
        
        """

        header:
        stamp:
            sec: 1691401559
            nanosec: 741685544
        frame_id: world
        pose:
        position:
            x: 11.84244155883789
            y: -2.155806064605713
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.7027148957352971
            w: 0.7114715562211398
        ---
        header:
        stamp:
            sec: 1691401569
            nanosec: 164142233
        frame_id: world
        pose:
        position:
            x: 11.389702796936035
            y: -1.5879647731781006
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: -0.9999831801250387
            w: 0.005799954052785274
        ---
        header:
        stamp:
            sec: 1691401574
            nanosec: 499579242
        frame_id: world
        pose:
        position:
            x: 11.224780082702637
            y: -4.262598514556885
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: -0.6842026829480455
            w: 0.7292919090780428
        ---
        header:
        stamp:
            sec: 1691401580
            nanosec: 995543056
        frame_id: world
        pose:
        position:
            x: 12.032266616821289
            y: -4.588181495666504
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.05708210223111083
            w: 0.9983694875169599
        ---
        
        
        
        """

        self.pose_it = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.plan_viewpoint_pub = rospy.Publisher("plan_viewpoint", TransformStamped)
        self.plan_viewpoint_result_sub = rospy.Subscriber("plan_viewpoint_result_loc_cam", PoseStamped, self.viewpoint_result_cb)

        self.change_mode_pub = rospy.Publisher("planner_mode", String)

        self.goal_pose_sub = rospy.Subscriber("/goal_pose", PoseStamped, self.goal_pose_cb)

    def viewpoint_result_cb(self, msg: PoseStamped):
        quat = np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        euler_zyx = pr.euler_from_quaternion(quat, 2,1,0, False)
        arm_angles = self.base_arm_angles.copy()
        arm_angles[0] = arm_angles[0] + euler_zyx[0]
        arm_angles[4] = arm_angles[4] + euler_zyx[1]
        arm_angles_req = ArmJointMovementRequest()
        arm_angles_req.joint_target = arm_angles
        print(arm_angles)
        self.arm_angle_proxy.call(arm_angles_req)

    def query_viewpoint_cb(self, req: EmptyRequest):
        self.plan_viewpoint()

        return EmptyResponse()
    
    def home_arm_cb(self, req: EmptyRequest):
        self.arm_angle_proxy.call(self.base_arm_angles_request)

        return EmptyResponse()
    
    def next_waypoint_cb(self, req: EmptyRequest):
        self.go_to_map_pose()

        return EmptyResponse()
    
    def go_to_map_pose(self):
        if self.pose_it == len(self.map_poses):
            self.pose_it = 0
        pose_msg = self.pq_to_pose_msg(self.map_poses[self.pose_it])
        go_to_pose = PoseStamped()
        go_to_pose.header.frame_id = "map"
        go_to_pose.pose = pose_msg
        self.move_spot_pub.publish(go_to_pose)
        self.arm_angle_proxy.call(self.base_arm_angles_request)

        self.pose_it += 1

    def goal_pose_cb(self, msg: PoseStamped):
        msg.header.frame_id = "map"
        self.move_spot_pub.publish(msg)
        self.arm_angle_proxy.call(self.base_arm_angles_request)

    def plan_viewpoint(self):
        try:
            # tf_hand_map: TransformStamped = self.tf_buffer.lookup_transform(target_frame="hand", source_frame="map", time=rospy.Time())
            tf_map_hand: TransformStamped = self.tf_buffer.lookup_transform(target_frame="map", source_frame="hand", time=rospy.Time())
            self.plan_viewpoint_pub.publish(tf_map_hand)
        except Exception as e:
            self.plan_viewpoint_pub.publish(TransformStamped())
            print("could not look up transform")


    def test_go_to_map_pose(self):
        if self.pose_it == len(self.map_poses)-1:
            self.pose_it = 0
        pose_msg = self.pq_to_pose_msg(self.map_poses[self.pose_it])
        go_to_pose = PoseStamped()
        go_to_pose.header.frame_id = "body"
        go_to_pose.pose = pose_msg

        arm_angles = self.base_arm_angles.copy()
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
    
    def tf_to_transform(self, tf: TransformStamped) -> np.ndarray:
        pq = np.array([tf.transform.translation.x,
                       tf.transform.translation.y,
                       tf.transform.translation.z,
                       tf.transform.rotation.w,
                       tf.transform.rotation.x,
                       tf.transform.rotation.y,
                       tf.transform.rotation.z])
        
        return pt.transform_from_pq(pq)
    
    def transform_to_tf(self, T_frameId_childFrameId: np.ndarray, frame_id: str, child_frame_id: str, time=None) -> TransformStamped:
        
        pq = pt.pq_from_transform(T_frameId_childFrameId)

        tf = TransformStamped()
        if time == None:
            tf.header.stamp = self.get_clock().now().to_msg()
        else:
            tf.header.stamp = time
        tf.header.frame_id = frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = pq[0]
        tf.transform.translation.y = pq[1]
        tf.transform.translation.z = pq[2]
    
        tf.transform.rotation.x = pq[4]
        tf.transform.rotation.y = pq[5]
        tf.transform.rotation.z = pq[6]
        tf.transform.rotation.w = pq[3]
        return tf




if __name__ == "__main__":
    rospy.init_node("spot_viewpoint_planner")
    node = SpotViewpointPlanner()
    rospy.spin()


