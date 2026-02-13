#!/usr/bin/env python3

import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import math

# Port Values
DRONE_PORTS = [14581, 14582]

# Global Coordinates
DRONE_1_COOR = [0.0, 3.0]
DRONE_2_COOR = [0.0, 6.0]

# Global Waypoints
WAYPOINT1    = [0.0, 5.0]
WAYPOINT2    = [0.0, 3.0]

# Global Variables
HOVER_ALT          = 1.5 # meter
POSITION_TOLERANCE = 0.2 # meter
HEADING_TOLERANCE  = 5.0 # degrees
UPDATE_RATE        = 1.0 / 10.0
K_REP              = 1.5  # Repulsive gain
D_SAFE             = 3.0  # Safe distance threshold (meters)
D_MIN              = 0.1  # Minimum distance to have attractive force
K_ATT              = 0.5  # Attractive gain
C_DAMP             = 5.0  # Damping coefficient
MAX_VELOCITY       = 1.5  # Max velocity m/s
MAX_VELOCITY_Z     = 0.5  # Max vertical velocity m/s

class DroneController:
    def __init__(self, port, name, i):
        self.drone          = System(port=50051+i)
        self.port           = port
        self.name           = name
        self.position       = np.zeros(3)
        self.velocity       = np.zeros(3)
        self.heading        = 0.0
        self.id             = 0

    async def connect(self):
        await self.drone.connect(system_address=f"udp://127.0.0.1:{self.port}")
        print(f"Waiting for {self.name} to connect...")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"{self.name} connected!")
                break

    async def check(self,i):
        # Wait for drone to be ready
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"{self.name} is ready!")
                break

        # Set takeoff altitude
        if i==0:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT)
        elif i==1:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT)

    async def arm_and_takeoff(self):
        print(f"Arming {self.name}...")
        await self.drone.action.arm()
        print(f"{self.name} armed!")

        print(f"{self.name} taking off...")
        await self.drone.action.takeoff()
        await asyncio.sleep(8)  # Wait for takeoff to complete

    async def start_position_updates(self):
        """Background task to continuously update position and heading"""
        print(f"{self.name}: Starting position update task")
        async def update_position():
            async for pos_vel_ned in self.drone.telemetry.position_velocity_ned():
                self.position = np.array([
                    pos_vel_ned.position.north_m,
                    pos_vel_ned.position.east_m,
                    pos_vel_ned.position.down_m
                ])
                self.velocity = np.array([
                    pos_vel_ned.velocity.north_m_s,
                    pos_vel_ned.velocity.east_m_s,
                    pos_vel_ned.velocity.down_m_s
                ])

        # Run concurrently
        async def update_heading():
            async for heading in self.drone.telemetry.heading():
                self.heading = heading.heading_deg

        await asyncio.gather(update_position(),update_heading())

    async def get_position(self):
        return self.position.copy(), self.heading

    async def get_velocity(self):
        return self.velocity.copy()

    async def start_offboard(self,i):
        print(f"Starting offboard mode for {self.name}...")
        # Send initial setpoint before starting offboard
        if i==0:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT, 0.0)
            )
        elif i==1:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT, 0.0)
            )

        try:
            await self.drone.offboard.start()
            print(f"{self.name} offboard mode started")
        except OffboardError as e:
            print(f"Starting offboard mode failed for {self.name}: {e}")
            raise

    async def set_position(self, n, e, d, yaw):
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(n, e, d, yaw)
        )

    async def land(self):
        print(f"Landing {self.name}...")
        await self.drone.action.land()
        
def smooth_turn(current_heading):
    """
    Smoothen the dynamic turning process, when moving pass 0/360 deg point
    """
    if current_heading < 0:
        current_heading += 360
    elif current_heading >=360:
        current_heading -= 360

    return current_heading

def smooth_heading_diff(heading_diff):
    if heading_diff > 180:
        heading_diff -= 360
    elif heading_diff < -180:
        heading_diff += 360

    return heading_diff

async def straight_maneuver(drone, start_n, start_e, start_d, start_heading, end_n, end_e, end_d, v_default, turn_default):
    """
    start_n/e/d : start local coordinate
    end_n/e/d : end local coordinate
    end_heading is defined to be facing the direction of motion
    v_default : An argument to tune the drone maneuver speed
    turn_default: An argument to tune the drone turning speed
    """
    delta_n = end_n - start_n
    delta_e = end_e - start_e
    delta_d = end_d - start_d

    end_heading = math.degrees( math.atan2((end_e - start_e),(end_n - start_n)) )
    if (end_e - start_e)<0:
        end_heading += 360
    print(f"Target heading: {end_heading:.2f}°\n")

    distance = math.sqrt(delta_n**2 + delta_e**2 + delta_d**2)
    if distance < POSITION_TOLERANCE:
        print("[Error] The starting point is too closed to the ending point, straight maneuver aborted!\n")
        return

    delta_h = end_heading - start_heading
    delta_h = smooth_heading_diff(delta_h)

    dn = delta_n/distance
    de = delta_e/distance
    dd = delta_d/distance

    dt = UPDATE_RATE #seconds
    step_size_default = v_default * dt
    travelled_pos = 0.0
    travelled_h = 0.0
    iteration = 0
    while travelled_pos < distance:
        remaining_pos = distance - travelled_pos
        step_pos = min(step_size_default, 0.3*remaining_pos)
        travelled_pos += step_pos

        cur_n = start_n + travelled_pos * dn
        cur_e = start_e + travelled_pos * de
        cur_d = start_d + travelled_pos * dd

        remaining_h = abs(delta_h) - travelled_h
        step_h = min(turn_default, 0.3*remaining_h) #turn 1 degree every UPDATE_RATE, which is 0.1s. So 1 second 10 degree turning
        travelled_h += step_h
        if delta_h>0:
            cur_h = start_heading + travelled_h
        else:
            cur_h = start_heading - travelled_h

        cur_h = smooth_turn(cur_h)

        iteration += 1
        if iteration % 10 == 0:
            print(f"[{drone.name}] Heartbeart during Mission: N={cur_n:.2f}, E={cur_e:.2f}, D={cur_d:.2f}, Heading={cur_h:.2f}")
        await drone.set_position(cur_n,cur_e,cur_d,cur_h)
        if (remaining_pos < POSITION_TOLERANCE) and (remaining_h < HEADING_TOLERANCE):
            return end_heading
        await asyncio.sleep(dt)

def repulsive_force(my_pos, other_pos, my_velocity, other_velocity, iteration):
    diff = my_pos - other_pos
    euclidean_distance = np.linalg.norm(diff)

    # Only apply repulsive force within safe distance
    if euclidean_distance > D_SAFE or euclidean_distance < 0.01:
        return np.zeros(3)

    # Unit direction vector
    direction = diff / euclidean_distance

    # Define spring magnitude
    spring_magnitude = K_REP * (1.0/euclidean_distance - 1.0/D_SAFE) * (1.0/euclidean_distance**2)
    spring_force = spring_magnitude * direction

    rel_vel = my_velocity - other_velocity
    rel_vel_radial = np.dot(rel_vel, direction)
    damping_force = -C_DAMP * rel_vel_radial * direction
    
    if iteration % 50 == 0:
        print(f"[repulsive_force] drone1_pos: {drone1_pos}")
        print(f"[repulsive_force] drone2_pos: {drone2_pos}")
        print(f"[repulsive_force] drone1_velocity: {drone1_velocity}")
        print(f"[repulsive_force] drone2_velocity: {drone2_velocity}")
        print(f"[repulsive_force] diff: {diff}")
        print(f"[repulsive_force] euclidean_distance: {euclidean_distance}")
        print(f"[repulsive_force] direction: {direction}")
        print(f"[repulsive_force] spring_magnitude: {spring_magnitude}")
        print(f"[repulsive_force] rel_vel_radial: {rel_vel_radial}")
        print(f"[repulsive_force] damping_force: {damping_force}")
    
    return spring_force + damping_force

def attractive_force(my_pos, goal_pos, my_velocity, iteration):
    diff = goal_pos - my_pos
    euclidean_distance = np.linalg.norm(diff)

    # Only apply attractive force outside minimum distance
    if euclidean_distance < D_MIN:
        return np.zeros(3)

    # Unit direction vector
    direction = diff / euclidean_distance

    # Define spring magnitude
    spring_magnitude = K_ATT * euclidean_distance
    spring_force = spring_magnitude * direction
    
    # Damping - resist my own velocity in direction of goal
    rel_vel = drone2_velocity - drone1_velocity
    rel_vel_radial = np.dot(rel_vel,direction)
    damping_force = -C_DAMP * rel_vel_radial * direction
    
    if iteration % 50 == 0:
        print(f"[attractive_force] my_pos: {my_pos}")
        print(f"[attractive_force] goal_pos: {goal_pos}")
        print(f"[attractive_force] my_velocity: {my_velocity}")
        print(f"[attractive_force] diff: {diff}")
        print(f"[attractive_force] euclidean_distance: {euclidean_distance}")
        print(f"[attractive_force] direction: {direction}")
        print(f"[attractive_force] spring_magnitude: {spring_magnitude}")
        print(f"[attractive_force] rel_vel_radial: {rel_vel_radial}")
        print(f"[attractive_force] damping_force: {damping_force}")

    return spring_force + damping_force

def compute_velocity(my_pos, goal_pos, other_pos, my_velocity, other_velocity, iteration):
    f_att = attractive_force(drone1_pos, drone2_pos, drone1_velocity, drone2_velocity, iteration)
    f_rep = repulsive_force(drone1_pos, drone2_pos, drone1_velocity, drone2_velocity, iteration)

    # Total desired velocity is the sum of all the vectors
    desired_vel = f_att + f_rep

    # Limit horizontal velocity
    my_vel_radial = np.dot(my_velocity, direction)
    

    return desired_vel

def coordinate_transformation(id, p2_n, p2_e, i):
    """
    Point 1: Origin global coordinate (0,0)
    Point 2: Waypoint coordinate
    Point 3: Drone (match id) global coordinate

    i argument:
        0 : Convert from global to local coordinate (point 2)
        1 : Convert from local to global coordinate (point 2)
    """

    if id == 1:
        p3_n = DRONE_1_COOR[1]
        p3_e = DRONE_1_COOR[0]
    elif id == 2:
        p3_n = DRONE_2_COOR[1]
        p3_e = DRONE_2_COOR[0]
    else:
        print("Instructions unclear: invalid id!")
        return

    if i == 0:
        vec_3to2 = [p2_n-p3_n, p2_e-p3_e]
        return vec_3to2[0], vec_3to2[1]
    elif i == 1:
        vec_1to2 = [p2_n+p3_n, p2_e+p3_e]
        return vec_1to2[0], vec_1to2[1]
    else:
        print("Instructions unclear: invalid i argument!")
        return

async def drone1_behavior(drone1):
    print(f"[{drone1.name}] Drone hovering ...")
    await asyncio.sleep(2)
    
    hover_pos,current_heading = await drone1.get_position()
    print(f"[{drone1.name}] starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone1.name}] Starting heading: {current_heading:.1f}°")

    print(f"[{drone1.name}] +++ Start 1st Mission +++")
    wp1_local_n, wp1_local_e = coordinate_transformation(drone1.id,WAYPOINT1[1],WAYPOINT1[0],0)
    print(f"[{drone1.name}] Moving to N={wp1_local_n:.2f}, E={wp1_local_e:.2f}, D={hover_pos[2]:.2f}")
    end_heading1 = await straight_maneuver(drone1, hover_pos[0], hover_pos[1], hover_pos[2], current_heading, wp1_local_n, wp1_local_e, hover_pos[2], 1.0, 0.8)
    print(f"[{drone1.name}] +++ End 1st Mission +++")
    print(f"[{drone1.name}] Wait for follower's reaction")
    await asyncio.sleep(10)

    print(f"[{drone1.name}] +++ Start 2nd Mission +++")
    wp2_local_n, wp2_local_e = coordinate_transformation(drone1.id,WAYPOINT2[1],WAYPOINT2[0],0)
    print(f"[{drone1.name}] Moving to N={wp2_local_n:.2f}, E={wp2_local_e:.2f}, D={hover_pos[2]:.2f}")
    end_heading2 = await straight_maneuver(drone1, wp1_local_n, wp1_local_e, hover_pos[2], end_heading1, wp2_local_n, wp2_local_e, hover_pos[2], 1.0, 0.8)
    print(f"[{drone1.name}] +++ End 2nd Mission +++")
    print(f"[{drone1.name}] Wait for follower's reaction")
    await asyncio.sleep(10)

    # Hold leader position after finish maneuvering
    while True:
        await drone1.set_position(
            wp2_local_n, wp2_local_e, hover_pos[2], end_heading2
        )
        await asyncio.sleep(UPDATE_RATE)

async def drone2_behavior(drone1,drone2):
    dt = 1.0 / UPDATE_RATE

    print(f"[{drone2.name}] Initializing... ")
    await asyncio.sleep(2)

    hover_pos,current_heading = await drone2.get_position()
    print(f"[{drone2.name}] starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone2.name}] Starting heading: {current_heading:.1f}°")

    """
    --- APF Implementation ---
    """

    iteration = 0
    while True:
        drone1_pos, drone1_heading = await drone1.get_position()
        drone2_pos, drone2_heading = await drone2.get_position()
        drone1_velocity = await drone1.get_velocity()
        drone2_velocity = await drone2.get_velocity()
        
        # Convert drone1 position to w.r.t. drone2's local frame
        drone1_pos[1],drone1_pos[0] = coordinate_transformation(drone1.id, drone1_pos[0], drone1_pos[1], 1)
        drone1_pos[1],drone1_pos[0] = coordinate_transformation(drone2.id, drone1_pos[0], drone1_pos[1], 0)
        
        # Calculate desired velocity
        desired_vel = compute_velocity(drone1_pos, drone2_pos, drone1_velocity, drone2_velocity, iteration)

        # Integrate velocity to get desired position
        next_pos = drone2_pos + desired_vel * dt
        await drone2.set_position(next_pos[0], next_pos[1], next_pos[2], 0.0)

        # Print status every 2 seconds
        if iteration % 20 == 0:
            current_distance = np.linalg.norm(drone2_pos - drone1_pos)
            print(f"[{drone2.name}] Distance: {current_distance:.2f}m | "
                  f"[{drone1.name}] Current position: N={drone1_pos[0]:.2f}, E={drone1_pos[1]:.2f}, D={drone1_pos[2]:.2f} |"
                  f"[{drone2.name}] Current position: N={drone2_pos[0]:.2f}, E={drone2_pos[1]:.2f}, D={drone2_pos[2]:.2f}")

        iteration += 1
        await asyncio.sleep(UPDATE_RATE)

async def main():
    print("=== APF Testing Starts ===")
    # Initialize drones
    drone1 = DroneController(DRONE_PORTS[0], "Super",0)
    drone2 = DroneController(DRONE_PORTS[1], "QAV250",1)

    # Assign drone id
    drone1.id = 1
    drone2.id = 2

    # Connect both drones
    await asyncio.gather(
        drone1.connect(),
        drone2.connect()
    )

    # Do all the pre-flight checks for both drones
    await asyncio.gather(
        drone1.check(0),
        drone2.check(1)
    )

    # Arm and takeoff
    await asyncio.gather(
        drone1.arm_and_takeoff(),
        drone2.arm_and_takeoff()
    )

    print("Both drones in air now!")

    # Start position update tasks
    position_task_drone1 = asyncio.create_task(drone1.start_position_updates())
    position_task_drone2 = asyncio.create_task(drone2.start_position_updates())

    # Start offboard mode for both
    await asyncio.gather(
        drone1.start_offboard(0),
        drone2.start_offboard(1),
    )

    # Run formation control
    try:
        await asyncio.gather(
            drone1_behavior(drone1),
            drone2_behavior(drone1, drone2)
            )
    except KeyboardInterrupt:
        print("\nStopping formation control...")
    finally:
        position_task_drone1.cancel()
        position_task_drone2.cancel()

    # Land both drones
    await asyncio.gather(
        drone1.land(),
        drone2.land()
    )

    print("===APF Testing Complete ===")


if __name__ == "__main__":
    asyncio.run(main())
