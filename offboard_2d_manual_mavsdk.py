#!/usr/bin/env python3
import asyncio
import math

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


LEADER_SYS_ADDR = "udp://:14540"   # change to your leader connection
FOLLOWER_SYS_ADDR = "udp://:14541" # change to your follower connection
OFFSET_BEHIND_M = 1.0              # 1 m behind leader


async def connect_drone(system_address: str, name: str) -> System:
    drone = System()
    await drone.connect(system_address=system_address)
    print(f"[{name}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{name}] Connected")
            break
    return drone


async def wait_for_local_position(drone: System, name: str):
    print(f"[{name}] Waiting for local position and home...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_home_position_ok:
            print(f"[{name}] Local position OK, home OK")
            break

# The function below is not in used, so commented out
"""
async def get_leader_pose(leader: System):
    # Returns (north, east, down, yaw_deg) of leader in local NED.
    async for odom in leader.telemetry.position_velocity_ned():
        north = odom.position.north_m
        east = odom.position.east_m
        down = odom.position.down_m
        # Yaw is not directly in this message; get from attitude_euler.
        break

    async for att in leader.telemetry.attitude_euler():
        yaw_deg = math.degrees(att.yaw_rad)  # att.yaw_rad is in radians
        return north, east, down, yaw_deg

"""
async def start_offboard_follower(follower: System):
    # Set initial setpoint (0,0,0) just to start offboard.
    print("[Follower] Setting initial offboard setpoint (0,0,0)")
    await follower.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("[Follower] Starting offboard")
    try:
        await follower.offboard.start()
    except OffboardError as error:
        print(f"[Follower] Offboard start failed: {error._result.result}")
        print("[Follower] Disarming")
        await follower.action.disarm()
        raise


async def leader_follower_loop(leader: System, follower: System):
    # Arm follower and start offboard.
    print("[Follower] Arming")
    await follower.action.arm()
    await start_offboard_follower(follower)

    print("[Follower] Entering leader-follower loop (Ctrl+C to stop)")
    try:
        while True:
            # Get leader pose
            async for pos in leader.telemetry.position_velocity_ned():
                north_l = pos.position.north_m
                east_l = pos.position.east_m
                down_l = pos.position.down_m
                break

            async for att in leader.telemetry.attitude_euler():
                yaw_rad = att.yaw_rad
                yaw_deg = math.degrees(yaw_rad)
                break

            # Compute follower target 1 m behind leader (body-x negative)
            offset_n = -OFFSET_BEHIND_M * math.cos(yaw_rad)
            offset_e = -OFFSET_BEHIND_M * math.sin(yaw_rad)

            north_f = north_l + offset_n
            east_f = east_l + offset_e
            down_f = down_l  # same altitude in NED

            # Send setpoint to follower
            await follower.offboard.set_position_ned(
                PositionNedYaw(north_f, east_f, down_f, yaw_deg)
            )

            # Run at ~20 Hz to satisfy PX4 offboard requirements
            await asyncio.sleep(0.05)

    except asyncio.CancelledError:
        print("[Follower] Leader-follower loop cancelled")
    finally:
        print("[Follower] Stopping offboard and landing")
        try:
            await follower.offboard.stop()
        except OffboardError as error:
            print(f"[Follower] Offboard stop failed: {error._result.result}")
        await follower.action.land()


async def main():
    # Connect to leader and follower
    leader = await connect_drone(LEADER_SYS_ADDR, "Leader")
    follower = await connect_drone(FOLLOWER_SYS_ADDR, "Follower")

    # Wait for local position on both
    await wait_for_local_position(leader, "Leader")
    await wait_for_local_position(follower, "Follower")

    # Run follower loop
    await leader_follower_loop(leader, follower)


if __name__ == "__main__":
    asyncio.run(main())
