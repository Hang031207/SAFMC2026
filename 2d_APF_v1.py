#!/usr/bin/env python3

import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# Configuration
USE_GZ = True
if USE_GZ:
    DRONE_PORTS = [14581, 14582]
else:
    DRONE_PORTS = [14561, 14562]

HOVER_ALT = -1.0
OFFBOARD_RATE_HZ = 20.0

# APF Parameters
class APFParams:
    # Repulsive force (collision avoidance)
    K_REP = 15.0           # Repulsive gain
    D_SAFE = 3.0           # Safe distance threshold (meters)
    D_MAX = 5.0            # Maximum influence distance
    
    # Attractive force (to goal)
    K_ATT = 1.5            # Attractive gain
    
    # Damping (for spring-damper model)
    USE_DAMPING = True
    C_DAMP = 5.0           # Damping coefficient
    
    # Velocity limits
    MAX_VELOCITY = 0.8     # Max velocity m/s
    MAX_VELOCITY_Z = 0.5   # Max vertical velocity m/s

class SimpleAPF:
    def __init__(self, params: APFParams):
        self.params = params
    
    def repulsive_force(self, my_pos, my_vel, other_pos, other_vel):
        """Calculate repulsive force between two drones (spring-damper model)"""
        # Position difference vector (from other to me)
        diff = my_pos - other_pos
        distance = np.linalg.norm(diff)
        
        # Only apply repulsive force within max influence distance
        if distance > self.params.D_MAX or distance < 0.01:
            return np.zeros(3)
        
        # Unit direction vector
        direction = diff / distance
        
        # Spring force: repel when too close
        # When distance < D_SAFE: strong repulsion
        if distance < self.params.D_SAFE:
            spring_magnitude = self.params.K_REP * (1.0/distance - 1.0/self.params.D_SAFE) * (1.0/distance**2)
        else:
            # Gradual decrease between D_SAFE and D_MAX
            spring_magnitude = self.params.K_REP * 0.1 * (self.params.D_MAX - distance) / (self.params.D_MAX - self.params.D_SAFE)
        
        spring_force = spring_magnitude * direction
        
        # Damping force: reduce relative velocity
        if self.params.USE_DAMPING:
            relative_vel = my_vel - other_vel
            # Project onto direction
            relative_vel_radial = np.dot(relative_vel, direction)
            damping_force = -self.params.C_DAMP * relative_vel_radial * direction
        else:
            damping_force = np.zeros(3)
        
        return spring_force + damping_force
    
    def attractive_force(self, my_pos, goal_pos):
        """Calculate attractive force toward goal"""
        diff = goal_pos - my_pos
        distance = np.linalg.norm(diff)
        
        if distance < 0.1:  # Close enough to goal
            return np.zeros(3)
        
        direction = diff / distance
        # Linear attractive force
        magnitude = self.params.K_ATT * distance
        
        return magnitude * direction
    
    def compute_velocity(self, my_pos, my_vel, goal_pos, other_positions, other_velocities):
        """Compute desired velocity considering all forces"""
        # Attractive force to goal
        f_att = self.attractive_force(my_pos, goal_pos)
        
        # Repulsive forces from other drones
        f_rep = np.zeros(3)
        for other_pos, other_vel in zip(other_positions, other_velocities):
            f_rep += self.repulsive_force(my_pos, my_vel, other_pos, other_vel)
        
        # Total desired velocity
        desired_vel = f_att + f_rep
        
        # Limit horizontal velocity
        vel_horizontal = np.array([desired_vel[0], desired_vel[1], 0.0])
        speed_horizontal = np.linalg.norm(vel_horizontal)
        if speed_horizontal > self.params.MAX_VELOCITY:
            scale = self.params.MAX_VELOCITY / speed_horizontal
            desired_vel[0] *= scale
            desired_vel[1] *= scale
        
        # Limit vertical velocity
        if abs(desired_vel[2]) > self.params.MAX_VELOCITY_Z:
            desired_vel[2] = np.sign(desired_vel[2]) * self.params.MAX_VELOCITY_Z
        
        return desired_vel

class DroneState:
    """Store drone state"""
    def __init__(self):
        self.position = np.zeros(3)  # NED
        self.velocity = np.zeros(3)  # NED
        self.goal = np.zeros(3)      # NED

async def connect_drone(port, i):
    """Connect to single drone."""
    drone = System(port=50051+i)
    await drone.connect(system_address=f"udp://:{port}")
    
    print(f"[Drone {i+1}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {i+1}] Connected!")
            break
    
    # Health check
    async for health in drone.telemetry.health():
        print(f"[Drone {i+1}] Health: local_pos_ok={health.is_local_position_ok}")
        if health.is_local_position_ok:
            break
    
    return drone

async def setup_offboard(drone: System, drone_id: int):
    """Arm and start offboard mode."""
    print(f"[Drone {drone_id}] Arming...")
    await drone.action.arm()
    
    # Get current position
    print(f"[Drone {drone_id}] Getting current position...")
    async for pos_vel_ned in drone.telemetry.position_velocity_ned():
        current_pos = np.array([
            pos_vel_ned.position.north_m,
            pos_vel_ned.position.east_m,
            pos_vel_ned.position.down_m
        ])
        print(f"[Drone {drone_id}] Current: N={current_pos[0]:.2f}, E={current_pos[1]:.2f}, D={current_pos[2]:.2f}")
        
        # Set hover target (above current position)
        hover_target = current_pos.copy()
        hover_target[2] += HOVER_ALT
        
        # Set initial setpoint
        initial_wp = PositionNedYaw(hover_target[0], hover_target[1], hover_target[2], 0.0)
        await drone.offboard.set_position_ned(initial_wp)
        break
    
    print(f"[Drone {drone_id}] Starting offboard mode...")
    await drone.offboard.start()
    
    return hover_target

async def update_drone_state(drone: System, state: DroneState):
    """Update drone state from telemetry (single read)."""
    async for pos_vel in drone.telemetry.position_velocity_ned():
        state.position = np.array([
            pos_vel.position.north_m,
            pos_vel.position.east_m,
            pos_vel.position.down_m
        ])
        state.velocity = np.array([
            pos_vel.velocity.north_m_s,
            pos_vel.velocity.east_m_s,
            pos_vel.velocity.down_m_s
        ])
        break

async def apf_control_loop(drones, states, apf, phase):
    """Main APF control loop."""
    dt = 1.0 / OFFBOARD_RATE_HZ
    
    while True:
        # Update all drone states
        await asyncio.gather(
            update_drone_state(drones[0], states[0]),
            update_drone_state(drones[1], states[1])
        )
        
        # Compute desired positions for each drone
        for i, (drone, state) in enumerate(zip(drones, states)):
            # Collect other drones' positions and velocities
            other_positions = [states[j].position for j in range(len(states)) if j != i]
            other_velocities = [states[j].velocity for j in range(len(states)) if j != i]
            
            # Compute velocity command using APF
            desired_vel = apf.compute_velocity(
                state.position,
                state.velocity,
                state.goal,
                other_positions,
                other_velocities
            )
            
            # Integrate velocity to get position (simple euler integration)
            next_pos = state.position + desired_vel * dt
            
            # Send position command
            await drone.offboard.set_position_ned(
                PositionNedYaw(next_pos[0], next_pos[1], next_pos[2], 0.0)
            )
        
        # Print status every 1 second
        if int(asyncio.get_event_loop().time() * OFFBOARD_RATE_HZ) % int(OFFBOARD_RATE_HZ) == 0:
            distance = np.linalg.norm(states[0].position - states[1].position)
            print(f"[{phase}] Distance: {distance:.2f}m | "
                  f"D1: ({states[0].position[0]:.2f}, {states[0].position[1]:.2f}) | "
                  f"D2: ({states[1].position[0]:.2f}, {states[1].position[1]:.2f})")
        
        await asyncio.sleep(dt)

async def run():
    """Main execution."""
    print("=== Multi-Drone APF Collision Avoidance ===")
    print(f"APF Parameters: K_REP={APFParams.K_REP}, D_SAFE={APFParams.D_SAFE}m, "
          f"D_MAX={APFParams.D_MAX}m, USE_DAMPING={APFParams.USE_DAMPING}")
    
    # Connect drones
    print("\n1. Connecting to drones...")
    drones = await asyncio.gather(
        connect_drone(DRONE_PORTS[0], 0),
        connect_drone(DRONE_PORTS[1], 1)
    )
    
    # Setup offboard and get initial hover positions
    print("\n2. Setting up offboard mode...")
    hover_positions = await asyncio.gather(
        setup_offboard(drones[0], 1),
        setup_offboard(drones[1], 2)
    )
    
    # Initialize drone states
    states = [DroneState(), DroneState()]
    for i, hover_pos in enumerate(hover_positions):
        states[i].goal = hover_pos.copy()
        states[i].position = hover_pos.copy()
    
    # Create APF controller
    apf = SimpleAPF(APFParams())
    
    # Phase 1: Hover in place for 5 seconds
    print("\n3. PHASE 1: Hovering in place (5s)...")
    control_task = asyncio.create_task(apf_control_loop(drones, states, apf, "HOVER"))
    await asyncio.sleep(5.0)
    
    # Phase 2: Drone 1 moves towards Drone 2
    print("\n4. PHASE 2: Drone 1 moving towards Drone 2 (15s)...")
    # Move Drone 1's goal 4 meters East (towards Drone 2 if they start apart)
    states[0].goal[1] += 4.0  # East direction
    print(f"   Drone 1 new goal: ({states[0].goal[0]:.2f}, {states[0].goal[1]:.2f}, {states[0].goal[2]:.2f})")
    print(f"   Drone 2 stays at: ({states[1].goal[0]:.2f}, {states[1].goal[1]:.2f}, {states[1].goal[2]:.2f})")
    print("   Watch Drone 2 react to avoid collision!")
    await asyncio.sleep(15.0)
    
    # Phase 3: Drone 1 moves away from Drone 2
    print("\n4. PHASE 3: Drone 1 moving away from Drone 2 (15s)...")
    # Move Drone 1's goal 4 meters West
    states[0].goal[1] -= 4.0  # East direction
    print(f"   Drone 1 new goal: ({states[0].goal[0]:.2f}, {states[0].goal[1]:.2f}, {states[0].goal[2]:.2f})")
    print(f"   Drone 2 stays at: ({states[1].goal[0]:.2f}, {states[1].goal[1]:.2f}, {states[1].goal[2]:.2f})")
    print("   Watch Drone 2 react to avoid collision!")
    await asyncio.sleep(15.0)
    
    # Stop control loop
    control_task.cancel()
    try:
        await control_task
    except asyncio.CancelledError:
        pass
    
    # Land both drones
    print("\n6. Landing both drones...")
    await asyncio.gather(
        drones[0].offboard.stop(),
        drones[1].offboard.stop(),
        return_exceptions=True
    )
    await asyncio.gather(
        drones[0].action.land(),
        drones[1].action.land()
    )
    
    print("\n=== Mission Complete! ===")

if __name__ == "__main__":
    asyncio.run(run())
