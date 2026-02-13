#!/usr/bin/env python3

"""
Build on top of v8 code, but now try to reduce the influence of gate APF field
Since the drone will be mostly following the path, then the influence of gate APF field no need to be so great, just need to be able to correct small deviation from the path
"""

import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import math

# Port Values
DRONE_PORTS = [14581, 14582]

# Global Coordinates (in competition arena)
START_GATE_COOR    = [2.5, 5.0]
DRONE_1_COOR       = [0.0, 3.0]
DRONE_2_COOR       = [0.0, 6.0]
DRONE1_START_COOR  = [1.5, 5.0]
DRONE2_START_COOR  = [0.0, 5.0]

# Gate Global Coordinates
GATE_1_COOR        = [14.5, 4.0]
GATE_WIDTH         = 2.0  # Gate width

# Waypoints
WAYPOINT1_1 = [ 7.0, 5.0]
WAYPOINT2_1 = [ 8.5, 5.0]
WAYPOINT1_2 = [11.0, 4.0]
WAYPOINT2_2 = [12.5, 4.0]
WAYPOINT1_3 = [19.0, 4.0]
WAYPOINT2_3 = [20.5, 4.0]


# Global Variables
HOVER_ALT          = 1.5  # meter
POSITION_TOLERANCE = 0.2  # meter
HEADING_TOLERANCE  = 5.0  # degrees
UPDATE_RATE        = 0.1  # 10 Hz

# APF Parameters - TUNED FOR 1.5M FORMATION
D_FORMATION        = 1.5  # CHANGED: 1.5m formation distance
D_TOLERANCE        = 0.3  # Acceptable distance error (meters)
K_SPRING           = 6.0  # Spring constant
C_DAMP             = 1.0  # Damping coefficient
MAX_VEL_FORMATION  = 2.5  # Max velocity m/s
MAX_VEL_GATE       = 3.0
K_VEL_MATCH        = 2.0  # Velocity matching gain
P                  = 10.0

# Gate avoidance parameters
K_GATE_REPEL       = 3.0  # Gate repulsion strength
D_GATE_SAFE        = 1.0  # Safe distance from gate edges
D_GATE_INFLUENCE   = 3.0  # Gate influence radius

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

    async def check(self, i):
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"{self.name} is ready!")
                break
        await self.drone.action.set_takeoff_altitude(HOVER_ALT)

    async def arm_and_takeoff(self):
        print(f"Arming {self.name}...")
        await self.drone.action.arm()
        print(f"{self.name} armed!")

        print(f"{self.name} taking off...")
        await self.drone.action.takeoff()
        await asyncio.sleep(8)

    async def start_position_updates(self):
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

        async def update_heading():
            async for heading in self.drone.telemetry.heading():
                self.heading = heading.heading_deg

        await asyncio.gather(update_position(), update_heading())

    async def get_position(self):
        return self.position.copy(), self.heading

    async def get_velocity(self):
        return self.velocity.copy()

    async def start_offboard(self, i):
        print(f"Starting offboard mode for {self.name}...")
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
    if current_heading < 0:
        current_heading += 360
    elif current_heading >= 360:
        current_heading -= 360
    return current_heading

def smooth_heading_diff(heading_diff):
    if heading_diff > 180:
        heading_diff -= 360
    elif heading_diff < -180:
        heading_diff += 360
    return heading_diff

def formation_spring_damper_force(my_pos, other_pos, my_velocity, other_velocity, formation_distance):
    """Formation control force with velocity matching"""
    diff = my_pos - other_pos
    current_distance = np.linalg.norm(diff)
    
    if current_distance < 0.01 or current_distance > 2.0:
        return np.zeros(3)
    
    direction = diff / current_distance
    distance_error = current_distance - formation_distance
    
    # Adaptive gains based on error magnitude
    if abs(distance_error) > 2.0:
        k_spring = K_SPRING + 3
        c_damp = C_DAMP + 0.5
    elif abs(distance_error) > 1.0:
        k_spring = K_SPRING + 2
        c_damp = C_DAMP + 0.5
    else:
        k_spring = K_SPRING
        c_damp = C_DAMP + 1.0
    
    # Spring force
    spring_force_magnitude = -k_spring * distance_error
    position_force = spring_force_magnitude * direction
    
    # Velocity matching
    velocity_error = other_velocity - my_velocity
    velocity_matching_force = K_VEL_MATCH * velocity_error
    
    # Damping
    relative_velocity = my_velocity - other_velocity
    relative_vel_radial = np.dot(relative_velocity, direction)
    damping_force = -c_damp * relative_vel_radial * direction
    
    return position_force + velocity_matching_force + damping_force

def gate_alignment_force(my_pos, gate_center, gate_width):
    """
    Repulsive force from gate pillars
    Assumes gate has two pillars at gate_center ± (0, gate_width/2, 0)
    """
    # Two pillars: left and right (in East direction)
    pillar_up = np.array([gate_center[0] + gate_width/2, gate_center[1], my_pos[2]])
    pillar_down = np.array([gate_center[0] - gate_width/2, gate_center[1], my_pos[2]])
    
    total_repulsive = np.zeros(3)
    # To decide whether influenced by gate upper pillar or lower pillar
    if my_pos[0] > gate_center[0]:
        k=1
        pillar = pillar_up
    else: 
        k=-1
        pillar = pillar_down
    # k=1 means CW, k=-1 means CCW later
    
    diff = my_pos - pillar
    distance = np.linalg.norm(diff[:2])  # Only horizontal distance
    
    # Avoid division by zero, and also avoid applying this if too far from gate
    if distance > D_GATE_INFLUENCE or distance < 0.01:
        return np.zeros(3), np.zeros(3)
            
    # Obtain radial unit vector, then convert to tangential unit vector
    e_r = diff / distance
    e_r = e_r[:2]
    
    # Define rotation matrix: for vector transformation
    if k == 1:
        e_t = vector_rot(e_r, -np.pi/2) # CW rotation
    elif k == -1:
        e_t = vector_rot(e_r, np.pi/2) # CCW rotation
    
    # 1. Radial repulsive force
    radial_magnitude = K_GATE_REPEL * 0.3 * (1.0/distance - 1.0/D_GATE_INFLUENCE) * (1.0/distance**2)
    radial_force = np.array([
            radial_magnitude * e_r[0],
            radial_magnitude * e_r[1],
            0.0
        ])
    
    # 2. Tangential force
    tangential_magnitude = K_GATE_REPEL * 0.3 * (1.0 - distance/D_GATE_INFLUENCE)
    tangential_force = np.array([
            tangential_magnitude * e_t[0],
            tangential_magnitude * e_t[1],
            0.0
        ])
    
    # Return radial force and tangential force individually
    return radial_force, tangential_force

async def straight_maneuver_with_apf(
    my_drone, other_drone,
    start_n, start_e, start_d, start_heading,
    end_n, end_e, end_d,
    v_default, turn_default,
    formation_distance, use_apf=True, gate_centers=None
):
    """
    Enhanced straight maneuver with APF for formation + gate avoidance
    
    Args:
        my_drone: This drone controller
        other_drone: Other drone controller (for formation)
        gate_centers: List of gate center positions [(n1, e1), (n2, e2), ...]
        use_apf: Enable APF corrections (False for mission 0, True for mission 1+)
    """
    delta_n = end_n - start_n
    delta_e = end_e - start_e
    delta_d = end_d - start_d

    end_heading = math.degrees(math.atan2(delta_e, delta_n))
    if delta_e < 0:
        end_heading += 360
    print(f"[{my_drone.name}] Target heading: {end_heading:.2f}°")

    distance = math.sqrt(delta_n**2 + delta_e**2 + delta_d**2)
    if distance < POSITION_TOLERANCE:
        print(f"[{my_drone.name}] Starting point too close to ending point, aborting!")
        return start_heading

    delta_h = end_heading - start_heading
    delta_h = smooth_heading_diff(delta_h)

    # Direction unit vectors
    dn = delta_n / distance
    de = delta_e / distance
    dd = delta_d / distance

    dt = UPDATE_RATE
    step_size_default = v_default * dt
    travelled_pos = 0.0
    travelled_h = 0.0
    iteration = 0

    while travelled_pos < distance:
        # Nominal path position
        remaining_pos = distance - travelled_pos
        step_pos = min(step_size_default, 0.3 * remaining_pos)
        travelled_pos += step_pos
        
        next_pos= np.array([start_n + travelled_pos * dn,
                            start_e + travelled_pos * de,
                            start_d + travelled_pos * dd
                          ])
        # Heading update
        remaining_h = abs(delta_h) - travelled_h
        step_h = min(turn_default, 0.3 * remaining_h)
        travelled_h += step_h
        if delta_h > 0:
            nominal_h = start_heading + travelled_h
        else:
            nominal_h = start_heading - travelled_h
        nominal_h = smooth_turn(nominal_h)

        # Get current actual position and velocity
        my_pos, _ = await my_drone.get_position()
        my_vel = await my_drone.get_velocity()
        other_pos, _ = await other_drone.get_position()
        other_vel = await other_drone.get_velocity()
        
        apf_vel          = np.zeros(3)
        radial_force     = np.zeros(3)
        tangential_force = np.zeros(3)
        formation_vel    = np.zeros(3)
        gate_vel         = np.zeros(3)
        
        if use_apf:
            # Convert other drone to my local frame
            other_local_n, other_local_e = coordinate_transformation(
                other_drone.id, other_pos[0], other_pos[1], 1
            ) # To global frame first
            other_local_n, other_local_e = coordinate_transformation(
                my_drone.id, other_local_n, other_local_e, 0
            ) # Then to my drone local frame
            other_pos_local = np.array([other_local_n, other_local_e, other_pos[2]])

            # Calculate APF correction velocities
            # 1. Formation maintenance
            formation_vel = formation_spring_damper_force(
                my_pos, other_pos_local, my_vel, other_vel, formation_distance
            )

            # 2. Gate avoidance
            if gate_centers:
                for gate_center_global in gate_centers:
                    # Convert gate to local frame
                    gate_local_n, gate_local_e = coordinate_transformation(
                        my_drone.id, gate_center_global[0], gate_center_global[1], 0
                    )
                    gate_local = np.array([gate_local_n, gate_local_e])
                
                    radial_force, tangential_force = gate_alignment_force(my_pos, gate_local, GATE_WIDTH)
                    gate_vel += radial_force + tangential_force
        
        # Velocity limiting (for formation velocity only!)
        formation_vel_limited = formation_vel.copy()
        formation_speed = np.linalg.norm(formation_vel_limited[:2])
        if formation_speed > MAX_VEL_FORMATION:
            scale = MAX_VEL_FORMATION / formation_speed
            formation_vel_limited[0] *= scale
            formation_vel_limited[1] *= scale
        
        # Add the apf_vel consideration to the calculation of next position 
        apf_vel = formation_vel_limited + gate_vel
        next_pos += apf_vel * dt * P
        
        # Send position command
        await my_drone.set_position(next_pos[0], next_pos[1], next_pos[2], nominal_h)

        # Status printout
        if iteration % 20 == 0:
            current_distance = np.linalg.norm(my_pos[:2] - other_pos[:2]) if use_apf else 0
            print(f"[{my_drone.name}] Formation velocity: N={formation_vel_limited[0]:.2f}m/s, E={formation_vel_limited[1]:.2f}m/s, D={formation_vel_limited[2]:.2f}m/s | "
                  f"[{my_drone.name}] Radial velocity: N={radial_force[0]:.2f}m/s, E={radial_force[1]:.2f}m/s, D={radial_force[2]:.2f}m/s | "
                  f"Tangential velocity: N={tangential_force[0]:.2f}m/s, E={tangential_force[1]:.2f}m/s, D={tangential_force[2]:.2f}m/s | "
                  f"APF velocity: N={apf_vel[0]:.2f}m/s, E={apf_vel[1]:.2f}m/s, D={apf_vel[2]:.2f}m/s")
            print(f"[{my_drone.name}]t={iteration*dt:.1f}s | "
                  f"Pos: N={my_pos[0]:.2f}, E={my_pos[1]:.2f}, D={my_pos[2]:.2f} | "
                  f"Form dist: {distance:.2f}m | "
                  f"APF Speed: {np.linalg.norm(apf_vel[:2]):.2f} m/s")

        # Check completion
        if remaining_pos < POSITION_TOLERANCE and remaining_h < HEADING_TOLERANCE:
            print(f"[{my_drone.name}] Straight maneuver almost completed!")
            return end_heading

        iteration += 1
        await asyncio.sleep(dt)

    return end_heading

def coordinate_transformation(id, p2_n, p2_e, i):
    """Coordinate transformation between global and local frames"""
    if id == 1:
        p3_n, p3_e = DRONE_1_COOR[1], DRONE_1_COOR[0]
    elif id == 2:
        p3_n, p3_e = DRONE_2_COOR[1], DRONE_2_COOR[0]
    else:
        print("Invalid drone id!")
        return None, None

    if i == 0:  # Global to local
        return p2_n - p3_n, p2_e - p3_e
    elif i == 1:  # Local to global
        return p2_n + p3_n, p2_e + p3_e
    else:
        print("Invalid transformation mode!")
        return None, None
        
def vector_rot(e, angle_rad):
    R = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                  [np.sin(angle_rad),  np.cos(angle_rad)]])
    return R @ e

async def drone1_behavior(drone1, drone2):
    """Leader drone behavior"""
    print(f"[{drone1.name}] Waiting for formation setup...")
    await asyncio.sleep(3)
    
    hover_pos, current_heading = await drone1.get_position()
    print(f"[{drone1.name}] Starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone1.name}] Starting heading: {current_heading:.1f}°")
    
    # Mission 0: Move to gate front (lineup position)
    print(f"\n[{drone1.name}] +++ Mission 0: Move to gate front (1.5, 5.0) +++")
    start_local_n, start_local_e = coordinate_transformation(
        drone1.id, DRONE1_START_COOR[1], DRONE1_START_COOR[0], 0
    )
    print(f"[{drone1.name}] Target: N={start_local_n:.2f}, E={start_local_e:.2f}")
    
    end_heading1 = await straight_maneuver_with_apf(
        drone1, drone2,
        hover_pos[0], hover_pos[1], hover_pos[2], current_heading,
        start_local_n, start_local_e, hover_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=False
    )
    
    print(f"[{drone1.name}] +++ Reached start formation, holding for 5s... +++")
    current_pos, _ = await drone1.get_position()
    for _ in range(100):  # 10 seconds at 10Hz
        await drone1.set_position(current_pos[0], current_pos[1], current_pos[2], end_heading1)
        await asyncio.sleep(UPDATE_RATE)
    
    # Mission 1: Move to waypoint 1
    print(f"\n[{drone1.name}] +++ Mission 1: Move to waypoint (7.0, 5.0) +++")
    wp1_local_n, wp1_local_e = coordinate_transformation(
        drone1.id, WAYPOINT1_1[1], WAYPOINT1_1[0], 0
    )
    print(f"[{drone1.name}] Target: N={wp1_local_n:.2f}, E={wp1_local_e:.2f}")
    
    # Gate obstacle during this path
    gates_on_path = [(GATE_1_COOR[1], GATE_1_COOR[0])]  # Global coordinates
    
    end_heading2 = await straight_maneuver_with_apf(
        drone1, drone2,
        start_local_n, start_local_e, current_pos[2], end_heading1,
        wp1_local_n, wp1_local_e, current_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=True, gate_centers=gates_on_path
    )
    
    print(f"[{drone1.name}] +++ Reach waypoint 1 +++")
    await asyncio.sleep(2)
    
    # Mission 2: Move to waypoint 2
    print(f"\n[{drone1.name}] +++ Mission 2: Move to waypoint (11.0, 4.0) +++")
    wp2_local_n, wp2_local_e = coordinate_transformation(
        drone1.id, WAYPOINT1_2[1], WAYPOINT1_2[0], 0
    )
    print(f"[{drone1.name}] Target: N={wp2_local_n:.2f}, E={wp2_local_e:.2f}")
    
    # Gate obstacle during this path
    gates_on_path = [(GATE_1_COOR[1], GATE_1_COOR[0])]  # Global coordinates
    
    end_heading3 = await straight_maneuver_with_apf(
        drone1, drone2,
        wp1_local_n, wp1_local_e, current_pos[2], end_heading2,
        wp2_local_n, wp2_local_e, current_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=True, gate_centers=gates_on_path
    )
    
    print(f"[{drone1.name}] +++ Reach waypoint 2 +++")
    await asyncio.sleep(2)
    
    # Mission 3: Move to waypoint 3
    print(f"\n[{drone1.name}] +++ Mission 2: Move to waypoint (19.0, 4.0) +++")
    wp3_local_n, wp3_local_e = coordinate_transformation(
        drone1.id, WAYPOINT1_3[1], WAYPOINT1_3[0], 0
    )
    print(f"[{drone1.name}] Target: N={wp3_local_n:.2f}, E={wp3_local_e:.2f}")
    
    # Gate obstacle during this path
    gates_on_path = [(GATE_1_COOR[1], GATE_1_COOR[0])]  # Global coordinates
    
    end_heading4 = await straight_maneuver_with_apf(
        drone1, drone2,
        wp2_local_n, wp2_local_e, current_pos[2], end_heading3,
        wp3_local_n, wp3_local_e, current_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=True, gate_centers=gates_on_path
    )
    
    print(f"[{drone1.name}] +++ Reach waypoint 3 +++")
    await asyncio.sleep(2)
    
    # Hold position
    final_pos, _ = await drone1.get_position()
    while True:
        await drone1.set_position(final_pos[0], final_pos[1], final_pos[2], end_heading2)
        await asyncio.sleep(UPDATE_RATE)

async def drone2_behavior(drone1, drone2):
    """Follower drone behavior - starts with delay"""
    print(f"[{drone2.name}] Waiting for leader to start...")
    await asyncio.sleep(3)  # Sync with drone1
    
    hover_pos, current_heading = await drone2.get_position()
    print(f"[{drone2.name}] Starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone2.name}] Formation mode: {D_FORMATION}m from leader")
    
    # SEQUENTIAL START: Wait extra seconds so drone1 starts first
    print(f"[{drone2.name}] Delaying start by 3s for sequential launch...")
    current_pos, _ = await drone2.get_position()
    for _ in range(10):  # 1 seconds at 10Hz
        await drone2.set_position(current_pos[0], current_pos[1], current_pos[2], current_heading)
        await asyncio.sleep(UPDATE_RATE)
    
    # Mission 0: Move to starting position (0.0, 5.0) - 1.5m from gate front
    print(f"\n[{drone2.name}] +++ Mission 0: Move to starting line (0.0, 5.0) +++")
    start_local_n, start_local_e = coordinate_transformation(
        drone2.id, DRONE2_START_COOR[1], DRONE2_START_COOR[0], 0  # (0.0, 5.0) in global
    )
    print(f"[{drone1.name}] Target: N={start_local_n:.2f}, E={start_local_e:.2f}")
    end_heading1 = await straight_maneuver_with_apf(
        drone2, drone1,
        hover_pos[0], hover_pos[1], hover_pos[2], current_heading,
        start_local_n, start_local_e, hover_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=False
    )
    
    print(f"[{drone2.name}] +++ Reached starting formation, holding ... +++")
    current_pos, _ = await drone2.get_position()
    for _ in range(20):
        await drone2.set_position(current_pos[0], current_pos[1], current_pos[2], end_heading1)
        await asyncio.sleep(UPDATE_RATE)
    
    # Mission 1: Move to waypoint 1
    print(f"\n[{drone2.name}] +++ Mission 1: Move to waypoint (8.5, 5.0) +++")
    wp1_local_n, wp1_local_e = coordinate_transformation(
        drone2.id, WAYPOINT2_1[1], WAYPOINT2_1[0], 0
    )
    print(f"[{drone2.name}] Target: N={wp1_local_n:.2f}, E={wp1_local_e:.2f}")
    
    gates_on_path = [(GATE_1_COOR[1], GATE_1_COOR[0])]
    
    end_heading2 = await straight_maneuver_with_apf(
        drone2, drone1,
        start_local_n, start_local_e, current_pos[2], end_heading1,
        wp1_local_n, wp1_local_e, current_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=True, gate_centers=gates_on_path
    )
    
    print(f"[{drone2.name}] +++ Reach waypoint 1, holding ... +++")
    await asyncio.sleep(2)
    
    # Mission 2: Move to waypoint 2
    print(f"\n[{drone2.name}] +++ Mission 2: Move to waypoint (12.5, 4.0) +++")
    wp2_local_n, wp2_local_e = coordinate_transformation(
        drone2.id, WAYPOINT2_2[1], WAYPOINT2_2[0], 0
    )
    print(f"[{drone2.name}] Target: N={wp2_local_n:.2f}, E={wp2_local_e:.2f}")
    
    # Gate obstacle during this path
    gates_on_path = [(GATE_1_COOR[1], GATE_1_COOR[0])]  # Global coordinates
    
    end_heading3 = await straight_maneuver_with_apf(
        drone2, drone1,
        wp1_local_n, wp1_local_e, current_pos[2], end_heading2,
        wp2_local_n, wp2_local_e, current_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=True, gate_centers=gates_on_path
    )
    
    print(f"[{drone2.name}] +++ Reach waypoint 2 +++")
    await asyncio.sleep(2)
    
    # Mission 3: Move to waypoint 3
    print(f"\n[{drone2.name}] +++ Mission 3: Move to waypoint (20.5, 4.0) +++")
    wp3_local_n, wp3_local_e = coordinate_transformation(
        drone2.id, WAYPOINT2_3[1], WAYPOINT2_3[0], 0
    )
    print(f"[{drone2.name}] Target: N={wp3_local_n:.2f}, E={wp3_local_e:.2f}")
    
    # Gate obstacle during this path
    gates_on_path = [(GATE_1_COOR[1], GATE_1_COOR[0])]  # Global coordinates
    
    end_heading4 = await straight_maneuver_with_apf(
        drone2, drone1,
        wp2_local_n, wp2_local_e, current_pos[2], end_heading3,
        wp3_local_n, wp3_local_e, current_pos[2],
        v_default=1.0, turn_default=0.8,
        formation_distance=D_FORMATION, use_apf=True, gate_centers=gates_on_path
    )
    
    print(f"[{drone2.name}] +++ Reach waypoint 3 +++")
    await asyncio.sleep(2)
    
    # Hold position
    final_pos, _ = await drone2.get_position()
    while True:
        await drone2.set_position(final_pos[0], final_pos[1], final_pos[2], end_heading2)
        await asyncio.sleep(UPDATE_RATE)

async def main():
    print("=== Competition APF Test: Formation + Gate Avoidance ===\n")
    print(f"Parameters:")
    print(f"  Formation Distance: {D_FORMATION}m")
    print(f"  Spring Constant: {K_SPRING}")
    print(f"  Gate Repulsion: {K_GATE_REPEL}")
    print(f"  Gate Safe Distance: {D_GATE_SAFE}m")
    print(f"\nMission Plan:")
    print(f"  1. Drone1 -> (1.5, 5.0), Drone2 -> (0.0, 5.0) [Lineup]")
    print(f"  2. Both -> (24.0, 5.0) [Avoid gate at (14.5, 5.0)]")
    print(f"  3. Maintain {D_FORMATION}m formation throughout\n")
    
    # Initialize drones
    drone1 = DroneController(DRONE_PORTS[0], "Super", 0)
    drone2 = DroneController(DRONE_PORTS[1], "QAV250", 1)
    drone1.id = 1
    drone2.id = 2

    # Connect
    await asyncio.gather(drone1.connect(), drone2.connect())
    
    # Pre-flight checks
    await asyncio.gather(drone1.check(0), drone2.check(1))
    
    # Takeoff
    await asyncio.gather(drone1.arm_and_takeoff(), drone2.arm_and_takeoff())
    print("Both drones airborne!")
    await asyncio.sleep(3)

    # Start telemetry
    position_task_drone1 = asyncio.create_task(drone1.start_position_updates())
    position_task_drone2 = asyncio.create_task(drone2.start_position_updates())
    await asyncio.sleep(2)

    # Start offboard
    await asyncio.gather(drone1.start_offboard(0), drone2.start_offboard(1))
    print("Offboard mode active\n")
    await asyncio.sleep(2)

    # Run missions
    try:
        await asyncio.gather(
            drone1_behavior(drone1, drone2),
            drone2_behavior(drone1, drone2)
        )
    except KeyboardInterrupt:
        print("\nMission interrupted!")
    finally:
        position_task_drone1.cancel()
        position_task_drone2.cancel()

    # Land
    await asyncio.gather(drone1.land(), drone2.land())
    print("\n=== Mission Complete ===")

if __name__ == "__main__":
    asyncio.run(main())
