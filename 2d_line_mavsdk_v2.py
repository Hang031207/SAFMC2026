import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import math

# Configuration
USE_GZ = 1
if USE_GZ:
    DRONE_PORTS = [14581, 14582]
else:
    DRONE_PORTS = [14561, 14562]

HOVER_ALT = 2.5  # NED: negative = up
FORMATION_DISTANCE = 5.0  # meters behind leader
FORMATION_HEIGHT_OFFSET = 0.0  # same altitude in formation
HEIGHT_OFFSET = 1.0  # follower flies 1.5m lower during positioning
UPDATE_RATE = 1.0 / 10.0  # 10Hz update rate
POSITION_TOLERANCE = 0.3  # meters
HEADING_TOLERANCE = 5.0  # degrees
TARGET_HEADING = 45 #degrees, edit based on gate location

class DroneController:
    def __init__(self, port, name, i):
        self.drone = System(port=50051 + i)
        self.port = port
        self.name = name
        self.position = {'n': 0.0, 'e': 0.0, 'd': 0.0}
        self.heading = 0.0

    async def connect(self):
        await self.drone.connect(system_address=f"udp://127.0.0.1:{self.port}")
        print(f"Waiting for {self.name} to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"{self.name} connected!")
                break

    async def setup(self,i):
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"{self.name} is ready!")
                break
        if i==0:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT)
        elif i==1:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT + HEIGHT_OFFSET)

    async def arm_and_takeoff(self):
        print(f"Arming {self.name}...")
        await self.drone.action.arm()
        print(f"{self.name} taking off...")
        await self.drone.action.takeoff()
        await asyncio.sleep(8)

    async def start_position_updates(self):
        """Background task to continuously update position and heading"""
        print(f"{self.name}: Starting position update task")
        
        async def update_position():
            async for pos_vel_ned in self.drone.telemetry.position_velocity_ned():
                self.position['n'] = pos_vel_ned.position.north_m
                self.position['e'] = pos_vel_ned.position.east_m
                self.position['d'] = pos_vel_ned.position.down_m
        
        async def update_heading():
            async for heading_data in self.drone.telemetry.heading():
                self.heading = heading_data.heading_deg
        
        await asyncio.gather(update_position(), update_heading())

    async def global_pos_test(self):
        async for i in self.drone.telemetry.position():
            print(f"Latitude:" {i.latitude_deg} deg")
            print(f"Longitude:" {i.longitude_deg} deg")
            print(f"Absolute Altitude (AMSL):" {i.absolute_altitude_m} m")
            print(f"Relative Altitude (to takeoff):" {i.relative_altitude_m} m")

    async def get_position(self):
        return self.position.copy(), self.heading

    async def start_offboard(self):
        print(f"Starting offboard mode for {self.name}...")
        pos = self.position.copy()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(pos['n'], pos['e'], pos['d'], self.heading)
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
        print(f"Stopping offboard mode for {self.name}...")
        try:
            await self.drone.offboard.stop()
        except:
            pass
        print(f"Landing {self.name}...")
        await self.drone.action.land()


def calculate_follower_position(leader_pos, leader_heading, distance, height_offset):
    """Calculate follower position behind the leader"""
    heading_rad = math.radians(leader_heading)
    follower_n = leader_pos['n'] - distance * math.cos(heading_rad)
    follower_e = leader_pos['e'] - distance * math.sin(heading_rad)
    follower_d = leader_pos['d'] + height_offset
    return follower_n, follower_e, follower_d


async def leader_behavior(leader, target_heading):
    """
    IMPROVED Leader behavior:
    1. Hover at current position (let heading drift naturally)
    2. Turn to target heading
    3. Hold position and heading
    """
    await asyncio.sleep(2)
    
    hover_pos, current_heading = await leader.get_position()
    print(f"\n{'='*60}")
    print(f"LEADER: Initial position N={hover_pos['n']:.2f}, E={hover_pos['e']:.2f}, D={hover_pos['d']:.2f}")
    print(f"LEADER: Current heading {current_heading:.1f}°")
    print(f"{'='*60}\n")
    
    # Phase 1: Hover and wait for follower to stabilize (5 seconds)
    print("[LEADER PHASE 1] Hovering, waiting for follower...")
    for i in range(50):
        _, current_heading = await leader.get_position()
        await leader.set_position(
            hover_pos['n'], hover_pos['e'], hover_pos['d'],
            current_heading  # Just follow current heading
        )
        await asyncio.sleep(UPDATE_RATE)
    
    # Phase 2: Turn to target heading
    print(f"\n[LEADER PHASE 2] Turning to {target_heading}°...")
    _, start_heading = await leader.get_position()
    print(f"[LEADER] Starting turn from {start_heading:.1f}° to {target_heading:.1f}°")
    
    # Calculate shortest rotation direction
    heading_diff = target_heading - start_heading
    if heading_diff > 180:
        heading_diff -= 360
    elif heading_diff < -180:
        heading_diff += 360
    
    # Turn over 3 seconds
    steps = 30
    for i in range(steps + 1):
        progress = i / steps
        current_heading = start_heading + heading_diff * progress
        
        # Normalize to 0-360
        if current_heading < 0:
            current_heading += 360
        elif current_heading >= 360:
            current_heading -= 360
        
        await leader.set_position(
            hover_pos['n'], hover_pos['e'], hover_pos['d'],
            current_heading
        )
        
        if i % 10 == 0:
            print(f"[LEADER] Turning... {current_heading:.1f}° ({progress*100:.0f}%)")
        
        await asyncio.sleep(UPDATE_RATE)
    
    # Phase 3: Hold final heading
    print(f"[LEADER PHASE 3] Holding heading {target_heading}°\n")
    iteration = 0
    while True:
        await leader.set_position(
            hover_pos['n'], hover_pos['e'], hover_pos['d'],
            target_heading
        )
        
        if iteration % 50 == 0:  # Print every 5 seconds
            print(f"[LEADER PHASE 3] Holding formation (heading={target_heading:.1f}°)")
        
        iteration += 1
        await asyncio.sleep(UPDATE_RATE)


async def follower_behavior(leader, follower):
    """
    IMPROVED Follower behavior with collision avoidance:
    
    Phase 1: WAIT - Hover while leader turns to target heading
    Phase 2: POSITION (at lower altitude) - Move behind leader safely
    Phase 3: ALTITUDE - Rise to formation altitude
    Phase 4: HEADING - Align heading with leader
    Phase 5: TRACK - Maintain formation
    """
    print("[FOLLOWER] Initializing...")
    await asyncio.sleep(2)
    
    follower_start_pos, _ = await follower.get_position()
    print(f"\n[FOLLOWER] Initial position N={follower_start_pos['n']:.2f}, "
          f"E={follower_start_pos['e']:.2f}, D={follower_start_pos['d']:.2f}\n")
    
    iteration = 0
    phase = 1
    
    # Phase timing
    LEADER_TURN_TIME = 15  # Wait 15 seconds for leader to turn
    
    while True:
        leader_pos, leader_heading = await leader.get_position()
        follower_pos, follower_heading = await follower.get_position()
        
        # Calculate desired position
        target_n, target_e, target_d = calculate_follower_position(
            leader_pos, leader_heading,
            FORMATION_DISTANCE, FORMATION_HEIGHT_OFFSET
        )

        # Important step: Update the target position, so that it is expressed in follower's drone frame
        # The two drones have their own starting location as their coordinate origin when using the position_velocity_ned() function to get their position, which can cause issue if neglected
        # Is hardcoded version for now
        target_n -= 3
        target_e -= 0

        # Calculate errors
        pos_error = math.sqrt(
            (follower_pos['n'] - target_n) ** 2 +
            (follower_pos['e'] - target_e) ** 2
        )
        height_error = abs(follower_pos['d'] - target_d)
        heading_error = abs(follower_heading - leader_heading)
        if heading_error > 180:
            heading_error = 360 - heading_error
        
        # PHASE 1: WAIT - Hover in LOWER altitude while leader turns
        if phase == 1:
            await follower.set_position(
                follower_start_pos['n'], 
                follower_start_pos['e'], 
                follower_start_pos['d'],
                follower_heading  # Keep current heading
            )
            
            if iteration % 10 == 0:
                time_remaining = max(0, LEADER_TURN_TIME - iteration * UPDATE_RATE)
                print(f"[FOLLOWER PHASE 1] Hovering while leader turns... ({time_remaining:.1f}s remaining)")
            
            # Move to next phase after leader finishes turning
            if iteration * UPDATE_RATE >= LEADER_TURN_TIME:
                print(f"\n✓ [FOLLOWER PHASE 1] Complete, Leader turned, starting positioning\n")
                phase = 2
        
        # PHASE 2: POSITION - Move to position at LOWER altitude (collision avoidance)
        elif phase == 2:
            
            await follower.set_position(
                target_n, target_e, target_d - HEIGHT_OFFSET,
                follower_heading  # Keep current heading
            )
            # Reminder: .set_position() use NED frame, and target_d here is a -ve value

            if iteration % 10 == 0:
                print(f"[FOLLOWER PHASE 2] Current target position: N={target_n:.2f}, E={target_e:.2f}, D={target_d:.2f}\n")
                print(f"[FOLLOWER PHASE 2] Moving to leader's back: "
                      f"Error={pos_error:.2f}m, Current_alt={follower_pos['d']:.2f}m")
            
            # Check if in position (horizontally)
            if pos_error < POSITION_TOLERANCE:
                print(f"\n✓ [FOLLOWER PHASE 2] Complete, in position behind leader (at safe altitude)\n")
                phase = 3
        
        # PHASE 3: ALTITUDE - Rise to formation altitude
        elif phase == 3:
            await follower.set_position(
                target_n, target_e, target_d,  # Rise to formation altitude
                follower_heading
            )
            
            if iteration % 10 == 0:
                print(f"[FOLLOWER PHASE 3] Rising to formation altitude: "
                      f"Height_error={height_error:.2f}m")
            
            # Check if at correct altitude
            if height_error < 0.3:  # Within 30cm
                print(f"\n✓ [FOLLOWER PHASE 3] Complete, at formation altitude\n")
                phase = 4
        
        # PHASE 4: HEADING - Align heading with leader
        elif phase == 4:
            # Smooth heading alignment
            heading_diff = leader_heading - follower_heading
            if heading_diff > 180:
                heading_diff -= 360
            elif heading_diff < -180:
                heading_diff += 360
            
            # Move 20% toward target heading each iteration
            new_heading = follower_heading + heading_diff * 0.2
            if new_heading < 0:
                new_heading += 360
            elif new_heading >= 360:
                new_heading -= 360
            
            await follower.set_position(target_n, target_e, target_d, new_heading)
            
            if iteration % 10 == 0:
                print(f"[FOLLOWER PHASE 4] Aligning heading: "
                      f"Error={heading_error:.1f}°, Current={follower_heading:.1f}°, Target={leader_heading:.1f}°")
            
            # Check if aligned
            if heading_error < HEADING_TOLERANCE:
                print(f"\n✓ [FOLLOWER PHASE 4] Complete,heading aligned!")
                print(f"✓✓✓ FORMATION LOCKED ✓✓✓\n")
                phase = 5
        
        # PHASE 5: TRACK - Maintain formation
        else:
            await follower.set_position(target_n, target_e, target_d, leader_heading)
            
            if iteration % 50 == 0:  # Print every 5 seconds
                print(f"[FOLLOWER PHASE 5] Maintain formation"
                      f"Current target position: N={follower_pos['n']:.2f}, E={follower_pos['e']:.2f}, D={follower_pos['d']:.2f}"
                      f"Pos_error={pos_error:.2f}m, Head_error={heading_error:.1f}°")

        iteration += 1
        await asyncio.sleep(UPDATE_RATE)


async def main():
    leader = DroneController(DRONE_PORTS[0], "Leader", 0)
    follower = DroneController(DRONE_PORTS[1], "Follower", 1)

    await asyncio.gather(leader.connect(), follower.connect())
    await asyncio.gather(leader.setup(0), follower.setup(1))
    await asyncio.gather(leader.arm_and_takeoff(), follower.arm_and_takeoff())

    print("\n✓✓✓ Both drones in air! ✓✓✓\n")

    position_task_leader = asyncio.create_task(leader.start_position_updates())
    position_task_follower = asyncio.create_task(follower.start_position_updates())
    
    print("Waiting for position updates to stabilize...")
    await asyncio.sleep(3)

    await asyncio.gather(leader.start_offboard(), follower.start_offboard())
    
    print("\n" + "="*60)
    print("STARTING FORMATION SEQUENCE")
    print("="*60)
    print("Workflow:")
    print("1. Leader turns to target heading")
    print("2. Follower moves to position (at safe lower altitude)")
    print("3. Follower rises to formation altitude")
    print("4. Follower aligns heading")
    print("5. Formation locked!")
    print("="*60 + "\n")

    try:
        await asyncio.gather(
            leader_behavior(leader, target_heading=TARGET_HEADING),
            follower_behavior(leader, follower)
        )
    except KeyboardInterrupt:
        print("\nStopping formation control...")
    finally:
        position_task_leader.cancel()
        position_task_follower.cancel()

    await asyncio.gather(leader.land(), follower.land())
    print("\n✓✓✓ Mission complete! ✓✓✓")


if __name__ == "__main__":
    asyncio.run(main())
