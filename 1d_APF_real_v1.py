#!/usr/bin/env python3

"""
Single drone gate repulsion test for REAL HARDWARE
- Uses UWB for positioning (global coordinates)
- No coordinate transformation needed
- Tests APF gate avoidance mechanism
- Drone approaches obstacle and should stop at safe distance

Things to change if it is physical test:
1. DRONE_1_COOR, DRONE_2_COOR, might need to know the global coordinate before flight
2. TARGET_POSITION, simplest way is just vector from drone position to gate position x2
3. GATE_PILLAR, defined based on drone starting position
3. 
"""

import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import math

# ============================================================================
# CONFIGURATION - ADJUST THESE FOR YOUR SETUP
# ============================================================================

# Connection
DRONE_PORT = 14571  # For QAV250, change accordingly
DRONE_PORT_GZ = 14581 # For gz simulation
USE_GZ = True # Check if using gz

# Drone Coordinate (Need to change if real test)
DRONE_1_COOR       = [0.0, 3.0]
DRONE_2_COOR       = [0.0, 6.0]

# Test parameters
HOVER_ALT = -1.5  # Hover altitude (negative in NED, meters)
UPDATE_RATE = 0.1  # 10 Hz control rate

# Mission waypoints (if real, GLOBAL coordinates from UWB)
TARGET_POSITION = [DRONE_1_COOR[0]+3, DRONE_1_COOR[1]+3]  # [East, North] - beyond the obstacle

# Obstacle/Gate pillar location (GLOBAL coordinates)
GATE_PILLAR = [DRONE_1_COOR[0]+1.5, DRONE_1_COOR[1]+2]  # [East, North] - pillar in the path
GATE_WIDTH = 2.0  # Gate width (distance between pillars if 2 pillars)

# APF Parameters for gate avoidance
K_GATE_REPEL = 15.0  # Repulsive force strength
D_GATE_INFLUENCE = 2.0  # Start repelling at this distance (meters)
D_GATE_SAFE = 1.0  # Target safe distance from pillar
MAX_VELOCITY = 3.0  # Max velocity for safety (m/s)
MAX_VELOCITY_Z = 0.3  # Max vertical velocity
P = 10.0

# Approach parameters
APPROACH_SPEED = 0.5  # Normal approach speed (m/s)
POSITION_TOLERANCE = 0.3  # Position reached tolerance (meters)
HEADING_TOLERANCE = 5.0

# ============================================================================
# DRONE CONTROLLER
# ============================================================================

class DroneController:
    def __init__(self, port=DRONE_PORT):
        self.drone = System(port=50051)
        self.position = np.zeros(3)  # [N, E, D]
        self.velocity = np.zeros(3)
        self.heading = 0.0
        self.name = "QAV250"
        self.id = 1

    async def connect(self):
        """Connect to drone"""
        if USE_GZ:
            connection_string = f"udp://127.0.0.1:{DRONE_PORT_GZ}"
        else:
            connection_string = f"udp://:{DRONE_PORT}"
        
        print(f"Connecting to drone at {connection_string}...")
        await self.drone.connect(system_address=connection_string)
        
        print("Waiting for drone connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✓ Drone connected!")
                break

    async def check_health(self):
        """Wait for drone to be ready"""
        print("Checking drone health...")
        async for health in self.drone.telemetry.health():
            if health.is_local_position_ok and health.is_home_position_ok:
                print("✓ Drone is ready!")
                print(f"  - Local position: OK")
                print(f"  - Home position: OK")
                break
        
        await self.drone.action.set_takeoff_altitude(abs(HOVER_ALT))

    async def arm_and_takeoff(self):
        """Arm and takeoff"""
        print("\n=== ARMING ===")
        await self.drone.action.arm()
        print("✓ Armed")
        
        print("\n=== TAKING OFF ===")
        await self.drone.action.takeoff()
        await asyncio.sleep(8)  # Wait for takeoff
        print("✓ Airborne")

    async def start_position_updates(self):
        """Background task to update position and velocity"""
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
        
    async def start_offboard(self):
        """Start offboard mode"""
        print("\n=== STARTING OFFBOARD MODE ===")
        # Set initial setpoint
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, HOVER_ALT, 0.0)
        )
        
        try:
            await self.drone.offboard.start()
            print("✓ Offboard mode started")
        except OffboardError as e:
            print(f"✗ Offboard mode failed: {e}")
            raise

    async def set_position(self, n, e, d, yaw):
        """Send position setpoint"""
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(n, e, d, yaw)
        )

    async def land(self):
        """Land the drone"""
        print("\n=== LANDING ===")
        await self.drone.offboard.stop()
        await self.drone.action.land()

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

# ============================================================================
# APF GATE AVOIDANCE FUNCTIONS
# ============================================================================

def vector_rot(e, angle_rad):
    """Rotate 2D vector by angle"""
    R = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad), np.cos(angle_rad)]
    ])
    return R @ e

def gate_repulsive_force(drone_pos, pillar_pos):
    """
    Calculate repulsive force from gate pillar
    Combines radial repulsion + tangential guidance
    
    Args:
        drone_pos: [N, E, D] drone position (global)
        pillar_pos: [N, E] pillar position (global)
    
    Returns:
        force: [N, E, D] repulsive force vector
    """
    # Only consider horizontal position
    drone_2d = drone_pos[:2]
    
    # Distance to pillar
    diff = drone_2d - pillar_pos
    distance = np.linalg.norm(diff)
    
    # No force if too far or too close (avoid singularity)
    if distance > D_GATE_INFLUENCE or distance < 0.01:
        return np.zeros(3),np.zeros(3)
    
    # Unit vector from pillar to drone (radial direction)
    e_r = diff / distance
    
    # Tangential direction (perpendicular, guides around pillar)
    # Choose direction based on approach angle
    e_t = vector_rot(e_r, -np.pi/2)  # CW 90 degree rotation
    
    # 1. RADIAL REPULSION (pushes away from pillar)
    # Inverse square law - very strong when close
    radial_magnitude = K_GATE_REPEL * (1.0/distance - 1.0/D_GATE_INFLUENCE) * (1.0/distance**2)
    radial_force = radial_magnitude * e_r
    
    # 2. TANGENTIAL GUIDANCE (guides to side)
    # Linear decay - helps route around obstacle
    tangential_magnitude = K_GATE_REPEL * 0.5 * (1.0 - distance/D_GATE_INFLUENCE)
    tangential_force = tangential_magnitude * e_t
    
    return radial_force, tangential_force

def compute_apf_velocity(drone_pos, target_pos, pillar_pos):
    """
    Compute velocity combining nominal path + gate avoidance
    
    Args:
        drone_pos: [N, E, D] current position
        target_pos: [N, E] target position
        pillar_pos: [N, E] pillar position
    
    Returns:
        velocity: [N, E, D] desired velocity vector
    """
    # 1. NOMINAL VELOCITY (toward target)
    target_3d = np.array([target_pos[0], target_pos[1], drone_pos[2]])
    diff = target_3d - drone_pos
    distance = np.linalg.norm(diff)
    
    if distance < 0.01:
        return np.zeros(3)
    
    direction = diff / distance
    
    # 2. GATE REPULSIVE FORCE
    radial_force, tangential_force = gate_repulsive_force(drone_pos, pillar_pos)
    
    # 3. COMBINE
    apf_vel = np.array([radial_force[0]+tangential_force[0],
                        radial_force[1]+tangential_force[1], 
                        0.0])
    
    # 4. VELOCITY LIMITING
    vel_horizontal = apf_vel[:2]
    speed_horizontal = np.linalg.norm(vel_horizontal)
    if speed_horizontal > MAX_VELOCITY:
        scale = MAX_VELOCITY / speed_horizontal
        apf_vel[0] *= scale
        apf_vel[1] *= scale
    
    if abs(apf_vel[2]) > MAX_VELOCITY_Z:
        apf_vel[2] = np.sign(apf_vel[2]) * MAX_VELOCITY_Z
    
    return apf_vel

# ============================================================================
# MISSION EXECUTION
# ============================================================================

async def straight_maneuver_with_apf(
    my_drone,
    start_n, start_e, start_d, start_heading,
    end_n, end_e, end_d,
    v_default, turn_default,
    use_apf=True, gate_centers=None
):
    """
    Enhanced straight maneuver with APF for formation + gate avoidance
    
    Args:
        my_drone: This drone controller
        gate_centers: List of gate center positions [(n1, e1), (n2, e2), ...]
        use_apf: Enable APF corrections
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
        
        apf_vel = np.zeros(3)        
        if use_apf:
            # Gate avoidance algo only for this version
            if USE_GZ:
                # Convert gate to local frame
                gate_n, gate_e = coordinate_transformation(
                    my_drone.id, GATE_PILLAR[1], GATE_PILLAR[0], 0
                )
            else: 
                gate_n = GATE_PILLAR[1]
                gate_e = GATE_PILLAR[0]
            
            gate_local = np.array([gate_n, gate_e])
            target_local = np.array([end_n, end_e])
            apf_vel = compute_apf_velocity(my_pos, target_local, gate_local)

        distance_with_gate = math.sqrt((my_pos[0]-gate_n)**2 + (my_pos[1]-gate_e)**2 )
        # Add the effect due to gate APF
        next_pos += apf_vel * dt * P
        
        # Send position command
        await my_drone.set_position(next_pos[0], next_pos[1], next_pos[2], nominal_h)

        # Status printout
        if iteration % 10 == 0:
            print(f"[{my_drone.name}] APF velocity: N={apf_vel[0]:.2f}m/s, E={apf_vel[1]:.2f}m/s, D={apf_vel[2]:.2f}m/s | "
                  f"APF Speed: {np.linalg.norm(apf_vel[:2]):.2f} m/s")
            print(f"t={iteration*dt:.1f}s | "
                  f"Pos: N={my_pos[0]:.2f}, E={my_pos[1]:.2f}, D={my_pos[2]:.2f} | "
                  f"Drone thinks the gate is at: N={gate_n:.2f}, E={gate_e:.2f} | "
                  f"Dist with gate: {distance_with_gate:.2f}m")

        # Check completion
        if remaining_pos < POSITION_TOLERANCE and remaining_h < HEADING_TOLERANCE:
            print(f"[{my_drone.name}] Straight maneuver almost completed!")
            return end_heading

        iteration += 1
        await asyncio.sleep(dt)

    return end_heading

async def drone_behavior(drone):
    """Leader drone behavior"""
    print(f"[{drone.name}] Hovering for a few seconds...")
    await asyncio.sleep(3)
    
    hover_pos, current_heading = await drone.get_position()
    print(f"[{drone.name}] Starting position: N={hover_pos[0]:.2f}, E={hover_pos[1]:.2f}, D={hover_pos[2]:.2f}")
    print(f"[{drone.name}] Starting heading: {current_heading:.1f}°")
    
    # Mission 0: Move towards virtual pillar
    print(f"\n[{drone.name}] +++ Mission 0: Move towards target position +++")
    
    if USE_GZ:
        target_local_n, target_local_e = coordinate_transformation(
            drone.id, TARGET_POSITION[1], TARGET_POSITION[0], 0
        )
    else:
        target_local_n = TARGET_POSITION[1]
        target_local_e = TARGET_POSITION[0]
    print(f"[{drone.name}] Target: N={target_local_n:.2f}, E={target_local_e:.2f}")
    
    end_heading1 = await straight_maneuver_with_apf(
        drone,
        hover_pos[0], hover_pos[1], HOVER_ALT, current_heading,
        target_local_n, target_local_e, HOVER_ALT,
        v_default=1.0, turn_default=0.8,
        use_apf=True
    )
    
    print(f"[{drone.name}] +++ Holding ... +++")
    current_pos, _ = await drone.get_position()
    for _ in range(50):  # 10 seconds at 10Hz
        await drone.set_position(current_pos[0], current_pos[1], current_pos[2], end_heading1)
        await asyncio.sleep(UPDATE_RATE)

# ============================================================================
# MAIN MISSION
# ============================================================================

async def run_mission():
    """Main mission execution"""
    print("\n" + "="*60)
    print("SINGLE DRONE GATE REPULSION TEST - REAL HARDWARE")
    print("="*60)
    
    # Initialize drone
    drone = DroneController()
    
    # Connect
    await drone.connect()
    await drone.check_health()
    
    # Arm and takeoff
    await drone.arm_and_takeoff()
    
    # Start telemetry
    print("\nStarting telemetry updates...")
    telemetry_task = asyncio.create_task(drone.start_position_updates())
    await asyncio.sleep(2)
    
    # Get initial position
    initial_pos = drone.position.copy()
    print(f"\nInitial position: N={initial_pos[0]:.2f}, E={initial_pos[1]:.2f}, D={initial_pos[2]:.2f}")
    
    # Start offboard mode
    await drone.start_offboard()
    await asyncio.sleep(2)
    
    # Hold at takeoff position
    await asyncio.sleep(5)
    
    try:
        # Mission: Approach target with pillar in the way
        await drone_behavior(drone)
        
        # Hold final position
        await asyncio.sleep(3)
        
    except KeyboardInterrupt:
        print("\n\n⚠ Mission interrupted by user")
    except Exception as e:
        print(f"\n\n✗ Error during mission: {e}")
    finally:
        # Cleanup
        telemetry_task.cancel()
        
        # Land
        await drone.land()
        
        print("\n" + "="*60)
        print("MISSION COMPLETE")
        print("="*60)

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    asyncio.run(run_mission())
