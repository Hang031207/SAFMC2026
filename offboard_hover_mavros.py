#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# Global variables
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # Subscribe to drone state
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    # Publisher for local position
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Service clients for arming and setting mode
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Set the publishing rate (20 Hz)
    rate = rospy.Rate(20)

    # Wait for FCU connection
    print("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    print("FCU connected!")

    # Create target position for hover
    pose = PoseStamped()
    pose.pose.position.x = current.position.x
    pose.pose.position.y = 0
    pose.pose.position.z = 2  # Hover at 2 meters

    # Send a few setpoints before starting
    # This is required by PX4 to accept offboard mode
    print("Sending initial setpoints...")
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    # Create mode request for OFFBOARD
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Create arming request
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    print("Attempting to enter OFFBOARD mode and arm...")

    while not rospy.is_shutdown():
        # Try to set OFFBOARD mode and arm every 5 seconds
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                print("OFFBOARD mode enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    print("Vehicle armed")
                last_req = rospy.Time.now()

        # Continuously publish setpoint to maintain hover
        local_pos_pub.publish(pose)

        # Display current status
        if current_state.armed and current_state.mode == "OFFBOARD":
            print(f"Hovering at ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}m) - Press Ctrl+C to land")

        rate.sleep()
