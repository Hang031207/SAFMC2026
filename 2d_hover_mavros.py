#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Configuration
OFFSET_BEHIND_M = 1.0     # follower stays 1m behind leader
LEADER_TAKEOFF_Z = 1.0    # leader hover altitude
FOLLOWER_TAKEOFF_Z = 1.0  # follower hover altitude (same as leader)
HOVER_WAIT_BEFORE_FOLLOWER_S = 5.0  # wait time after leader is up

class LeaderFollowerController:
    def __init__(self):
        rospy.init_node("leader_follower_node")

        # Leader pose + yaw
        self.leader_pose = None
        self.leader_yaw = 0.0

        # States
        self.leader_state = State()
        self.follower_state = State()

        # Leader subscribers
        self.leader_pose_sub = rospy.Subscriber(
            "/drone1/mavros/local_position/pose",
            PoseStamped,
            self.leader_pose_callback
        )
        self.leader_state_sub = rospy.Subscriber(
            "/drone1/mavros/state",
            State,
            self.leader_state_callback
        )

        # Follower subscribers
        self.follower_state_sub = rospy.Subscriber(
            "/drone2/mavros/state",
            State,
            self.follower_state_callback
        )

        # Publishers (setpoints)
        self.leader_setpoint_pub = rospy.Publisher(
            "/drone1/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10
        )
        self.follower_setpoint_pub = rospy.Publisher(
            "/drone2/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10
        )

        # Leader services
        rospy.wait_for_service("/drone1/mavros/cmd/arming")
        self.leader_arming_client = rospy.ServiceProxy(
            "/drone1/mavros/cmd/arming",
            CommandBool
        )
        rospy.wait_for_service("/drone1/mavros/set_mode")
        self.leader_set_mode_client = rospy.ServiceProxy(
            "/drone1/mavros/set_mode",
            SetMode
        )

        # Follower services
        rospy.wait_for_service("/drone2/mavros/cmd/arming")
        self.follower_arming_client = rospy.ServiceProxy(
            "/drone2/mavros/cmd/arming",
            CommandBool
        )
        rospy.wait_for_service("/drone2/mavros/set_mode")
        self.follower_set_mode_client = rospy.ServiceProxy(
            "/drone2/mavros/set_mode",
            SetMode
        )

        self.rate = rospy.Rate(20)  # 20 Hz

    # ===== Callbacks =====
    def leader_pose_callback(self, msg):
        self.leader_pose = msg
        q = msg.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.leader_yaw = yaw

    def leader_state_callback(self, msg):
        self.leader_state = msg

    def follower_state_callback(self, msg):
        self.follower_state = msg

    # ===== Helper: simple leader takeoff and hover =====
    def arm_and_offboard_leader(self):
        print("[Leader] Waiting for FCU connection...")
        while not rospy.is_shutdown() and not self.leader_state.connected:
            self.rate.sleep()
        print("[Leader] FCU connected!")

        # Build a simple hover setpoint at current x,y, target z
        print("[Leader] Waiting for initial pose...")
        while not rospy.is_shutdown() and self.leader_pose is None:
            self.rate.sleep()
        print("[Leader] Pose received!")

        leader_sp = PoseStamped()
        leader_sp.header.frame_id = "map"
        leader_sp.pose.position.x = self.leader_pose.pose.position.x
        leader_sp.pose.position.y = self.leader_pose.pose.position.y
        leader_sp.pose.position.z = LEADER_TAKEOFF_Z
        leader_sp.pose.orientation = self.leader_pose.pose.orientation

        # Pre-stream setpoints
        print("[Leader] Sending initial setpoints...")
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            leader_sp.header.stamp = rospy.Time.now()
            self.leader_setpoint_pub.publish(leader_sp)
            self.rate.sleep()

        # OFFBOARD
        offb_req = SetModeRequest()
        offb_req.custom_mode = "OFFBOARD"
        print("[Leader] Setting OFFBOARD mode...")
        if self.leader_set_mode_client.call(offb_req).mode_sent:
            print("[Leader] OFFBOARD enabled")
        else:
            print("[Leader] Failed to set OFFBOARD")
            return False

        rospy.sleep(1.0)

        # Arm
        arm_req = CommandBoolRequest()
        arm_req.value = True
        print("[Leader] Arming...")
        if self.leader_arming_client.call(arm_req).success:
            print("[Leader] Armed!")
        else:
            print("[Leader] Failed to arm")
            return False

        # Hold hover altitude
        print("[Leader] Taking off and hovering...")
        hover_start = rospy.Time.now()
        while not rospy.is_shutdown():
            # maintain same x,y, target z
            leader_sp.header.stamp = rospy.Time.now()
            self.leader_setpoint_pub.publish(leader_sp)

            # after some time, just exit and let higher-level logic proceed
            if (rospy.Time.now() - hover_start).to_sec() > HOVER_WAIT_BEFORE_FOLLOWER_S:
                break

            self.rate.sleep()

        print("[Leader] Hover established, proceeding to follower.")
        return True

    # ===== Follower formation logic =====
    def calculate_follower_position(self):
        if self.leader_pose is None:
            return None

        leader_x = self.leader_pose.pose.position.x
        leader_y = self.leader_pose.pose.position.y
        leader_z = self.leader_pose.pose.position.z

        offset_x = -OFFSET_BEHIND_M * math.cos(self.leader_yaw)
        offset_y = -OFFSET_BEHIND_M * math.sin(self.leader_yaw)

        follower_x = leader_x + offset_x
        follower_y = leader_y + offset_y
        follower_z = FOLLOWER_TAKEOFF_Z  # same altitude as leader hover

        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.x = follower_x
        sp.pose.position.y = follower_y
        sp.pose.position.z = follower_z

        q = quaternion_from_euler(0, 0, self.leader_yaw)
        sp.pose.orientation.x = q[0]
        sp.pose.orientation.y = q[1]
        sp.pose.orientation.z = q[2]
        sp.pose.orientation.w = q[3]

        return sp

    def arm_and_offboard_follower(self):
        print("[Follower] Waiting for FCU connection...")
        while not rospy.is_shutdown() and not self.follower_state.connected:
            self.rate.sleep()
        print("[Follower] FCU connected!")

        print("[Follower] Waiting for leader position...")
        while not rospy.is_shutdown() and self.leader_pose is None:
            self.rate.sleep()
        print("[Follower] Leader position received!")

        print("[Follower] Sending initial setpoints...")
        initial_sp = self.calculate_follower_position()
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            if initial_sp:
                initial_sp.header.stamp = rospy.Time.now()
                self.follower_setpoint_pub.publish(initial_sp)
            self.rate.sleep()

        offb_req = SetModeRequest()
        offb_req.custom_mode = "OFFBOARD"
        print("[Follower] Setting OFFBOARD mode...")
        if self.follower_set_mode_client.call(offb_req).mode_sent:
            print("[Follower] OFFBOARD enabled")
        else:
            print("[Follower] Failed to set OFFBOARD")
            return False

        rospy.sleep(1.0)

        arm_req = CommandBoolRequest()
        arm_req.value = True
        print("[Follower] Arming...")
        if self.follower_arming_client.call(arm_req).success:
            print("[Follower] Armed!")
        else:
            print("[Follower] Failed to arm")
            return False

        return True

    def run(self):
        # 1) Leader up first
        if not self.arm_and_offboard_leader():
            return

        # 2) Then follower
        if not self.arm_and_offboard_follower():
            return

        print("\n" + "="*60)
        print("Leader-Follower formation active!")
        print(f"Follower maintaining {OFFSET_BEHIND_M} m behind leader")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")

        try:
            while not rospy.is_shutdown():
                sp = self.calculate_follower_position()
                if sp:
                    self.follower_setpoint_pub.publish(sp)
                self.rate.sleep()
        except KeyboardInterrupt:
            print("\n[Follower] Landing...")
            land_mode = SetModeRequest()
            land_mode.custom_mode = "AUTO.LAND"
            self.follower_set_mode_client.call(land_mode)

if __name__ == "__main__":
    try:
        controller = LeaderFollowerController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
