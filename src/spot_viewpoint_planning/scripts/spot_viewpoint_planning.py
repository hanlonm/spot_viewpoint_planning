#!/usr/bin/env python3
import os
from datetime import datetime
import rospy
import numpy as np
import random
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
import cv2
from cv_bridge import CvBridge


from spot_msgs.srv import HandPose, HandPoseRequest, HandPoseResponse, ArmJointMovement, ArmJointMovementRequest, ArmJointMovementResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import tf2_ros


class SpotViewpointPlanner:
    def __init__(self) -> None:
        
        self.test_service = rospy.Service("query_viewpoint", Empty, self.query_viewpoint_cb)
        self.next_wp_service = rospy.Service("next_waypoint", Empty, self.next_waypoint_cb)
        self.start_experiment_service = rospy.Service("start_experiment", Empty, self.start_experiment_cb)
        self.test_method_service = rospy.Service("test_method", Empty, self.test_method_cb)
        self.home_arm_service = rospy.Service("home_arm", Empty, self.home_arm_cb)
        self.gripper_pose_proxy = rospy.ServiceProxy("/spot/gripper_pose", HandPose)

        self.move_spot_pub = rospy.Publisher("/spot/go_to_pose", PoseStamped)
        rospy.loginfo("Spot viewpoint planner running")

        self.base_arm_pose_request = HandPoseRequest()
        self.base_arm_pose_request.duration = 0.0
        self.base_arm_pose_request.frame = "body"
        self.base_arm_pose = Pose()
        self.base_arm_pose.position.x = 0.29
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
        
        self.experiment_modes = ["mlp_clf", "trf_clf", "angle_crit", "max_crit", "fisher_info", "random"]
        self.exp_mode_it = 0

        self.pose_it = 0

        self.current_image_msg: Image = None
        self.output_dir = "/workspace/src/output"
        self.current_mode = None
        self.current_exp_dir = None


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge() 

        self.plan_viewpoint_pub = rospy.Publisher("plan_viewpoint", TransformStamped)
        self.plan_viewpoint_result_sub = rospy.Subscriber("plan_viewpoint_result_loc_cam", PoseStamped, self.viewpoint_result_cb)

        self.change_mode_pub = rospy.Publisher("planner_mode", String)

        self.goal_pose_sub = rospy.Subscriber("/goal_pose", PoseStamped, self.goal_pose_cb)

        self.image_sub = rospy.Subscriber("/spot/camera/hand_color/image", Image, self.image_cb)
        self.mode_sub = rospy.Subscriber("/current_planner_mode", String, self.current_mode_cb)

        self.new_exp_service = rospy.Service("new_experiment", Empty, self.new_experiment_cb)
        self.gt_loc_service = rospy.Service("gt_loc", Empty, self.gt_loc_cb)
        self.save_current_service = rospy.Service("save_current", Empty, self.save_current_cb)
        self.save_forward_service = rospy.Service("save_forward", Empty, self.save_forward)
        self.save_forward_low_service = rospy.Service("save_low_forward", Empty, self.save_low_forward)



    def image_cb(self, msg: Image):
        self.current_image_msg = msg
    
    def current_mode_cb(self, msg: String):
        self.current_mode = msg.data

    def new_experiment_cb(self, req: EmptyRequest):
        print("new experiment")
        current_datetime = datetime.now()
        folder_name = current_datetime.strftime("%y-%m-%d-%H-%M-%S")
        self.current_exp_dir = self.output_dir + "/" + folder_name
        os.mkdir(self.current_exp_dir)

        return EmptyResponse()
    
    def gt_loc_cb(self, req: EmptyRequest):
        try:
            gt_folder = self.current_exp_dir + "/gt"
            os.mkdir(gt_folder)
            current_image = self.bridge.imgmsg_to_cv2(
                self.current_image_msg, desired_encoding="passthrough")
            current_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB)
            tf_dict = {}
            tf_dict["tf_cam_tag"] = self.tf_buffer.lookup_transform(
                target_frame="hand_color_image_sensor", source_frame="tag_2", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_cam_body"] = self.tf_buffer.lookup_transform(
                target_frame="hand_color_image_sensor", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_map_body"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            pose_file = open(gt_folder + "/" "pose_file.txt", "w")
            pose_file.write("# tf_target_source tx ty tz qw qx qy qz")

            for tf in tf_dict.keys():
                transform: TransformStamped = tf_dict[tf]
                pq = [transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z,
                      transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z]
                pose_file.write("\n" + str(tf) + " " + " ".join(map(str, pq)))

            cv2.imwrite(gt_folder + "/loc_image.jpg", current_image)

        except Exception as e:
            print(e)
        return EmptyResponse()

    def save_current_cb(self, req: EmptyRequest):
        try:
            method_folder = self.current_exp_dir + "/" + self.current_mode
            os.mkdir(method_folder)
            current_image = self.bridge.imgmsg_to_cv2(
                self.current_image_msg, desired_encoding="passthrough")
            current_image = cv2.cvtColor(current_image.copy(), cv2.COLOR_BGR2RGB)
            tf_dict = {}
            tf_dict["tf_map_bodyest"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body_est", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_cam_body"] = self.tf_buffer.lookup_transform(
                target_frame="hand_color_image_sensor", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_map_body"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            pose_file = open(method_folder + "/" "pose_file.txt", "w")
            pose_file.write("# tf_target_source tx ty tz qw qx qy qz")

            for tf in tf_dict.keys():
                transform: TransformStamped = tf_dict[tf]
                pq = [transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z,
                      transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z]
                pose_file.write("\n" + str(tf) + " " + " ".join(map(str, pq)))

            cv2.imwrite(method_folder + "/loc_image.jpg", current_image)

        except Exception as e:
            print(e)
        return EmptyResponse()

    def viewpoint_result_cb(self, msg: PoseStamped):
        quat = np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        euler_zyx = pr.euler_from_quaternion(quat, 2,1,0, False)
        arm_angles = self.base_arm_angles.copy()
        arm_angles[0] = arm_angles[0] + euler_zyx[0]
        arm_angles[4] = arm_angles[4] + euler_zyx[1]
        print(arm_angles)
        arm_angles_req = ArmJointMovementRequest()
        arm_angles_req.joint_target = arm_angles
        print(arm_angles)
        # for _ in range(3):
        self.arm_angle_proxy.call(arm_angles_req)
        #     rospy.sleep(0.5)

    def query_viewpoint_cb(self, req: EmptyRequest):
        self.plan_viewpoint()

        return EmptyResponse()
    
    def home_arm_cb(self, req: EmptyRequest):
        self.arm_angle_proxy.call(self.base_arm_angles_request)

        return EmptyResponse()
    
    def start_experiment_cb(self, req: EmptyRequest):
        self.exp_mode_it = 0
        self.arm_angle_proxy.call(self.base_arm_angles_request)
        return EmptyResponse()

    def test_method_cb(self, req: EmptyRequest):
        self.arm_angle_proxy.call(self.base_arm_angles_request)
        if self.exp_mode_it == len(self.experiment_modes):
            self.exp_mode_it == 0
        mode = self.experiment_modes[self.exp_mode_it]
        mode_msg = String()
        mode_msg.data = mode
        self.change_mode_pub.publish(mode_msg)
        rospy.sleep(3.)
        self.plan_viewpoint()

        self.exp_mode_it += 1

        return EmptyResponse()
    
    def save_forward(self, req: EmptyRequest):
        self.arm_angle_proxy.call(self.base_arm_angles_request)
        rospy.sleep(3.)
        try:
            method_folder = self.current_exp_dir + "/forwards"
            os.mkdir(method_folder)
            current_image = self.bridge.imgmsg_to_cv2(
                self.current_image_msg, desired_encoding="passthrough")
            current_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB)
            tf_dict = {}
            tf_dict["tf_map_bodyest"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body_est", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_cam_body"] = self.tf_buffer.lookup_transform(
                target_frame="hand_color_image_sensor", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_map_body"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            pose_file = open(method_folder + "/" "pose_file.txt", "w")
            pose_file.write("# tf_target_source tx ty tz qw qx qy qz")

            for tf in tf_dict.keys():
                transform: TransformStamped = tf_dict[tf]
                pq = [transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z,
                      transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z]
                pose_file.write("\n" + str(tf) + " " + " ".join(map(str, pq)))

            cv2.imwrite(method_folder + "/loc_image.jpg", current_image)

        except Exception as e:
            print(e)

        return EmptyResponse()
    
    def save_low_forward(self, req: EmptyRequest):
        try:
            method_folder = self.current_exp_dir + "/low_forwards"
            os.mkdir(method_folder)
            current_image = self.bridge.imgmsg_to_cv2(
                self.current_image_msg, desired_encoding="passthrough")
            current_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB)
            tf_dict = {}
            tf_dict["tf_map_bodyest"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body_est", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_cam_body"] = self.tf_buffer.lookup_transform(
                target_frame="hand_color_image_sensor", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_dict["tf_map_body"] = self.tf_buffer.lookup_transform(
                target_frame="map", source_frame="body", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            pose_file = open(method_folder + "/" "pose_file.txt", "w")
            pose_file.write("# tf_target_source tx ty tz qw qx qy qz")

            for tf in tf_dict.keys():
                transform: TransformStamped = tf_dict[tf]
                pq = [transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z,
                      transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z]
                pose_file.write("\n" + str(tf) + " " + " ".join(map(str, pq)))

            cv2.imwrite(method_folder + "/loc_image.jpg", current_image)

        except Exception as e:
            print(e)

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


