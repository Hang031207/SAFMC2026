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
WAYPOINT1    = [0.0, 8.0]
WAYPOINT2    = [4.0, 4.0]

# Global Variables
HOVER_ALT          = 1.5 # meter
POSITION_TOLERANCE = 0.2 # meter
HEADING_TOLERANCE  = 5.0 # degrees
UPDATE_RATE        = 1.0 / 10.0

# APF Parameters
D_FORMATION        = 3.0 # Desired separation distance (meters) 
D_TOLERANCE        = 0.3 # Acceptable distance error (meters)
K_SPRING           = 8.0 # Spring constant
C_DAMP             = 2.0 # Damping coefficient
MAX_VELOCITY       = 1.5 # Max velocity m/s
MAX_VELOCITY_Z     = 0.5 # Max vertical velocity m/s
P                  = 8  # A constant to scale the effect of desired velocity on the real set_position function

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
                PositionNedYaw(0.0, 0.0, -HOVER_ALT, 0)
            )
        elif i==1:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT, 0)
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

def formation_spring_damper_force(my_pos, other_pos, my_velocity, other_velocity, formation_distance, iteration):
    """
    Spring-damper model for formation flying.
    - If distance > desired: attract (pull closer)
    - If distance < desired: repel (push away)
    - Damping reduces oscillation
    
    This is like a virtual spring connecting the two drones at formation_distance
    """
    # Vector from other to me
    diff = my_pos - other_pos
    current_distance = np.linalg.norm(diff)
    
    if current_distance < 0.01:  # Avoid division by zero
        return np.zeros(3)
    
    # Direction from other to me
    direction = diff / current_distance
    
    # Spring force: proportional to distance error
    distance_error = current_distance - formation_distance
    spring_force_magnitude = -K_SPRING * distance_error
    spring_force = spring_force_magnitude * direction
    
    # Damping force: proportional to relative velocity
    relative_velocity = my_velocity - other_velocity
    relative_vel_radial = np.dot(relative_velocity, direction)
    damping_force = -C_DAMP * relative_vel_radial * direction
    
    total_force = spring_force + damping_force
    
    if iteration % 50 == 0:
        print(f"[FORMATION] Current distance: {current_distance:.2f}m, Desired: {formation_distance:.2f}m")
        print(f"[FORMATION] Distance error: {distance_error:.2f}m")
        print(f"[FORMATION] Spring force magnitude: {spring_force_magnitude:.3f}")
        print(f"[FORMATION] Spring force: {spring_force}")
        print(f"[FORMATION] Damping force: {damping_force}")
        print(f"[FORMATION] Total force: {total_force}\n")
    
    return total_force

def compute_formation_velocity(my_pos, other_pos, my_velocity, other_velocity, formation_distance, iteration):
    force = formation_spring_damper_force(my_pos, other_pos, my_velocity, other_velocity, formation_distance, iteration)
    
    # Force directly becomes desired velocity (simple model)
    desired_vel = force

    # Limit horizontal velocity
    vel_horizontal = np.array([desired_vel[0], desired_vel[1], 0.0])
    speed_horizontal = np.linalg.norm(vel_horizontal)
    if speed_horizontal > MAX_VELOCITY:
        scale = MAX_VELOCITY / speed_horizontal
        desired_vel[0] *= scale
        desired_vel[1] *= scale
    
    # Limit vertical velocity
    if abs(desired_vel[2]) > MAX_VELOCITY_Z:
        desired_vel[2] = np.sign(desired_vel[2]) * MAX_VELOCITY_Z
    
    if iteration % 50 == 0:
        speed = np.linalg.norm(desired_vel[:2])
        print(f"[VELOCITY] Desired velocity: {desired_vel}m/s")
        print(f"[SPEED] Speed: {speed:.2f}m/s\n")

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
    await asyncio.sleep(5)
    
    hover_pos,current_heading = await drone1.get_position()
    print(f"[{drone1.name}] starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone1.name}] Starting heading: {current_heading:.1f}°")

    print(f"[{drone1.name}] +++ Start 1st Mission: Move to (0.0, 5.0) +++")
    wp1_local_n, wp1_local_e = coordinate_transformation(drone1.id,WAYPOINT1[1],WAYPOINT1[0],0)
    print(f"[{drone1.name}] Moving to N={wp1_local_n:.2f}, E={wp1_local_e:.2f}, D={hover_pos[2]:.2f}")
    end_heading1 = await straight_maneuver(drone1, hover_pos[0], hover_pos[1], hover_pos[2], current_heading, wp1_local_n, wp1_local_e, hover_pos[2], 1.0, 0.8)
    print(f"[{drone1.name}] +++ End 1st Mission +++")
    print(f"[{drone1.name}] Holding at (0.0, 5.0), wait for follower's reaction")
    await asyncio.sleep(15)

    print(f"[{drone1.name}] +++ Start 2nd Mission: Return to (0.0, 3.0) +++")
    wp2_local_n, wp2_local_e = coordinate_transformation(drone1.id,WAYPOINT2[1],WAYPOINT2[0],0)
    print(f"[{drone1.name}] Moving to N={wp2_local_n:.2f}, E={wp2_local_e:.2f}, D={hover_pos[2]:.2f}")
    end_heading2 = await straight_maneuver(drone1, wp1_local_n, wp1_local_e, hover_pos[2], end_heading1, wp2_local_n, wp2_local_e, hover_pos[2], 1.0, 0.8)
    print(f"[{drone1.name}] +++ End 2nd Mission +++")
    print(f"[{drone1.name}] Holding at (0.0, 3.0), wait for follower's reaction")
    await asyncio.sleep(15)

    # Hold leader position after finish maneuvering
    while True:
        await drone1.set_position(
            wp2_local_n, wp2_local_e, hover_pos[2], end_heading2
        )
        await asyncio.sleep(UPDATE_RATE)

async def drone2_behavior(drone1, drone2):

    print(f"[{drone2.name}] Initializing... ")
    await asyncio.sleep(5)

    hover_pos,current_heading = await drone2.get_position()
    print(f"[{drone2.name}] starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone2.name}] Starting heading: {current_heading:.1f}°")
    print(f"[{drone2.name}] Formation mode: Maintain {D_FORMATION}m distance from {drone1.name}\n")

    """
    --- APF Implementation ---
    """

    iteration = 0
    while True:
        drone1_pos, _ = await drone1.get_position()
        drone2_pos, _ = await drone2.get_position()
        drone1_velocity = await drone1.get_velocity()
        drone2_velocity = await drone2.get_velocity()
        
        # Convert drone1 position to w.r.t. drone2's local frame
        drone1_local_n, drone1_local_e = coordinate_transformation(drone1.id, drone1_pos[0], drone1_pos[1], 1)
        drone1_local_n, drone1_local_e = coordinate_transformation(drone2.id, drone1_local_n, drone1_local_e, 0)
        drone1_pos_local = np.array([drone1_local_n, drone1_local_e, drone1_pos[2]])
        
        # Calculate desired velocity
        desired_vel = compute_formation_velocity(drone2_pos, # My position
                                                 drone1_pos_local, # Other drone position (in my frame)
                                                 drone2_velocity, # My velocity
                                                 drone1_velocity, # Other drone velocity
                                                 D_FORMATION, # Desired separation
                                                 iteration
                                                )

        # Integrate velocity to get desired position
        next_pos = drone2_pos + desired_vel * UPDATE_RATE * P
        await drone2.set_position(next_pos[0], next_pos[1], next_pos[2], 0.0)

        # Print status every 2 seconds
        if iteration % 20 == 0:
            current_distance = np.linalg.norm(drone2_pos[:2] - drone1_pos_local[:2])
            distance_error = current_distance - D_FORMATION
            print(f"{'='*80}")
            print(f"[STATUS] Time: {iteration * UPDATE_RATE:.1f}s | "
                  f"[{drone2.name}] Distance: {current_distance:.2f}m | Error: {distance_error:+.2f}m | "
                  f"[{drone1.name}] Current position: N={drone1_pos_local[0]:.2f}, E={drone1_pos_local[1]:.2f}, D={drone1_pos_local[2]:.2f} | "
                  f"[{drone2.name}] Current position: N={drone2_pos[0]:.2f}, E={drone2_pos[1]:.2f}, D={drone2_pos[2]:.2f} | "
                  f"[{drone2.name}] Velocity: N={drone2_velocity[0]:.2f}, E={drone2_velocity[1]:.2f} m/s")
                  
            # Check if formation is stable
            if abs(distance_error) < D_TOLERANCE:
                print(f"[{drone2.name}] ✓ Formation STABLE (within {D_TOLERANCE}m tolerance)")
            else:
                print(f"[{drone2.name}] ⟳ Adjusting formation...")
            print(f"{'='*80}\n")

        iteration += 1
        await asyncio.sleep(UPDATE_RATE)

async def main():
    print("=== APF Testing Starts ===")
    print(f"Formation Parameters:")
    print(f"  Desired Distance: {D_FORMATION}m")
    print(f"  Spring Constant (K): {K_SPRING}")
    print(f"  Damping Coefficient (C): {C_DAMP}")
    print(f"  Max Velocity: {MAX_VELOCITY} m/s")
    print(f"\nExpected Behavior:")
    print(f"  - Drone1 starts at (0,3), moves to (0,5), then back to (0,3)")
    print(f"  - Drone2 starts at (0,6), should move to (0,8), then back to (0,6)")
    print(f"  - Distance maintained: {D_FORMATION}m ± {D_TOLERANCE}m\n")
    
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
    await asyncio.sleep(3)

    # Start position update tasks
    position_task_drone1 = asyncio.create_task(drone1.start_position_updates())
    position_task_drone2 = asyncio.create_task(drone2.start_position_updates())
    await asyncio.sleep(2)

    # Start offboard mode for both
    await asyncio.gather(
        drone1.start_offboard(0),
        drone2.start_offboard(1),
    )
    await asyncio.sleep(2)

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
