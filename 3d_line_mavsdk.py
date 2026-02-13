import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import math

# Configuration

USE_GZ=1
if USE_GZ:
    DRONE_PORTS = [14581,14582,14583]
else:
    LEADER_PORTS = [14561,14562,14563]

HOVER_ALT = 2.5 # NED: negative = up (0.5m above current)
FOLLOWER1_DISTANCE = 1.0  # meters behind leader
FOLLOWER2_DISTANCE = 2.0 # meteres behind 
HEIGHT_OFFSET = 1.0 # meters above the leader
FORMATION_HEIGHT_OFFSET = 0.0  # same altitude as leader
UPDATE_RATE = 1.0 / 10.0  # 10Hz update rate
POSITION_TOLERANCE = 0.3 #meters, considered in position
HEADING_TOLERANCE = 5.0 #degrees, considered aligned
TARGET_HEADING = 45 #degrees

class DroneController:
    def __init__(self, port, name, i):
        self.drone = System(port=50051+i)
        self.port = port
        self.name = name
        self.position = {'n': 0, 'e': 0, 'd': 0}
        self.heading = 0.0
        self._position_task = None
        
    async def connect(self):
        await self.drone.connect(system_address=f"udp://127.0.0.1:{self.port}")
        print(f"Waiting for {self.name} to connect...")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"{self.name} connected!")
                break
    
    async def setup(self,i):
        # Wait for drone to be ready
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"{self.name} is ready!")
                break
        
        # Set takeoff altitude
        if i==0:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT)
        elif i==1:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT + HEIGHT_OFFSET)
        elif i==2:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT + 2*HEIGHT_OFFSET)
        
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
                self.position['n'] = pos_vel_ned.position.north_m
                self.position['e'] = pos_vel_ned.position.east_m
                self.position['d'] = pos_vel_ned.position.down_m
            
        # Run concurrently
        async def update_heading():
            async for heading in self.drone.telemetry.heading():
                self.heading = heading.heading_deg

        await asyncio.gather(update_position(),update_heading())

    async def get_position(self):
        return self.position.copy(), self.heading
    
    async def start_offboard(self,i):
        print(f"Starting offboard mode for {self.name}...")
        # Send initial setpoint before starting offboard
        if i==0:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT, 0.0)
            )
        elif i==1:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT -HEIGHT_OFFSET, 0.0)
            )
        elif i==2:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT -2*HEIGHT_OFFSET, 0.0)
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


def calculate_follower_position(leader_pos, leader_heading, distance, height_offset):
    """
    Calculate follower position behind the leader

    Args:
        leader_pos: dict with 'n', 'e', 'd' keys (NED coordinates)
        leader_heading: heading in degrees (0-360, 0=North)
        distance: distance behind leader in meters
        height_offset: height difference from leader
    
    Returns:
        tuple: (north, east, down) position for follower
    """
    # Convert heading to radians
    heading_rad = math.radians(leader_heading)
    
    # Calculate position behind leader
    # Behind means opposite direction of heading
    follower_n = leader_pos['n'] - distance * math.cos(heading_rad)
    follower_e = leader_pos['e'] - distance * math.sin(heading_rad)
    follower_d = leader_pos['d'] + height_offset
    
    return follower_n, follower_e, follower_d


async def leader_behavior(leader,target_heading):
    """
    Leader drone behavior: hover then turn to desired heading
    """
    # Wait for position updates to stabilize
    await asyncio.sleep(1)

    # Hover at current position for 5 seconds
    print("Leader hovering...")
    hover_pos,current_heading = await leader.get_position()
    print(f"Leader starting position: N={hover_pos['n']:.2f}, E={hover_pos['e']:.2f}, D={hover_pos['d']:.2f}")
    print(f"Leader starting heading: {current_heading:.1f}°")

    for _ in range(30):
        await leader.set_position(
            hover_pos['n'], 
            hover_pos['e'], 
            hover_pos['d'], 
            current_heading
        )
        await asyncio.sleep(UPDATE_RATE)
    
    # Gradually turn to target heading over 3 seconds
    _, start_heading = await leader.get_position()
    print(f"Leader current heading before turn: {start_heading:.1f}°")

    heading_diff = target_heading - start_heading
    if heading_diff > 180:
        heading_diff -= 360
    elif heading_diff < -180:
        heading_diff += 360

    steps = 30  # 3 seconds at 10Hz
    
    for i in range(steps + 1):
        progress = i / steps
        current_heading = start_heading + heading_diff * progress

        if current_heading < 0:
            current_heading += 360
        elif current_heading >=360:
            current_heading -= 360

        await leader.set_position(
            hover_pos['n'],
            hover_pos['e'],
            hover_pos['d'],
            current_heading
        )

        if i%10==0:
            print(f"Leader turning: {current_heading:.1f}° (progress: {progress*100:.0f}%)")
        await asyncio.sleep(UPDATE_RATE)
    
    # Hold final heading
    print(f"Leader holding final heading {target_heading}°...")
    while True:
        await leader.set_position(
            hover_pos['n'],
            hover_pos['e'],
            hover_pos['d'],
            target_heading
        )
        await asyncio.sleep(UPDATE_RATE)


async def follower_behavior(leader, follower, i):
    """
    IMPROVED Follower behavior with collision avoidance:

    Phase 1: WAIT - Hover while leader turns to target heading
    Phase 2: POSITION (at lower altitude) - Move behind leader safely
    Phase 3: ALTITUDE - Rise to formation altitude
    Phase 4: HEADING - Align heading with leader
    Phase 5: TRACK - Maintain formation
    """
    print(f"[FOLLOWER {i}] Initializing...")
    await asyncio.sleep(1)

    follower_start_pos, _ = await follower.get_position()
    print(f"\n[FOLLOWER {i}] Initial position N={follower_start_pos['n']:.2f}, "
          f"E={follower_start_pos['e']:.2f}, D={follower_start_pos['d']:.2f}\n")

    iteration = 0
    phase = 1

    # Phase timing
    LEADER_TURN_TIME = 10  # Wait 15 seconds for leader to turn

    while True:
        leader_pos, leader_heading = await leader.get_position()
        follower_pos, follower_heading = await follower.get_position()

        # Calculate followers' target positions
        if i==1:
            target_n, target_e, target_d = calculate_follower_position(
                leader_pos, leader_heading,
                FOLLOWER1_DISTANCE, FORMATION_HEIGHT_OFFSET
            )
            target_n -= 3
            target_e -= 0
        elif i==2:
            target_n, target_e, target_d = calculate_follower_position(
                leader_pos, leader_heading,
                FOLLOWER2_DISTANCE, FORMATION_HEIGHT_OFFSET
            )
            target_n -= 6
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
                print(f"[FOLLOWER {i} PHASE 1] Hovering while leader turns... ({time_remaining:.1f}s remaining)")

            # Move to next phase after leader finishes turning
            if iteration * UPDATE_RATE >= LEADER_TURN_TIME:
                print(f"\n✓ [FOLLOWER {i} PHASE 1] Complete, Leader turned, starting positioning\n")
                phase = 2

        # PHASE 2: POSITION - Move to position at LOWER altitude (collision avoidance)
        elif phase == 2:
            if i==1:
                await follower.set_position(
                    target_n, target_e, target_d - HEIGHT_OFFSET,
                    follower_heading  # Keep current heading
                )
            elif i==2:
                await follower.set_position(
                    target_n, target_e, target_d - 2*HEIGHT_OFFSET,
                    follower_heading # Keep current heading
                )
            # Reminder: .set_position() use NED frame, and target_d here is a -ve value

            if iteration % 10 == 0:
                print(f"[FOLLOWER {i} PHASE 2] Current target position: N={target_n:.2f}, E={target_e:.2f}, D={target_d:.2f}\n")
                print(f"[FOLLOWER {i} PHASE 2] Moving to leader's back: "
                      f"Error={pos_error:.2f}m, Current_alt={follower_pos['d']:.2f}m")

            # Check if in position (horizontally)
            if pos_error < POSITION_TOLERANCE:
                print(f"\n✓ [FOLLOWER {i} PHASE 2] Complete, in position behind leader (at safe altitude)\n")
                phase = 3

        # PHASE 3: ALTITUDE - Rise to formation altitude
        elif phase == 3:
            await follower.set_position(
                target_n, target_e, target_d,  # Rise to formation altitude
                follower_heading
            )

            if iteration % 10 == 0:
                print(f"[FOLLOWER {i} PHASE 3] Rising to formation altitude: "
                      f"Height_error={height_error:.2f}m")

            # Check if at correct altitude
            if height_error < 0.3:  # Within 30cm
                print(f"\n✓ [FOLLOWER {i} PHASE 3] Complete, at formation altitude\n")
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
                print(f"[FOLLOWER {i} PHASE 4] Aligning heading: "
                      f"Error={heading_error:.1f}°, Current={follower_heading:.1f}°, Target={leader_heading:.1f}°")

            # Check if aligned
            if heading_error < HEADING_TOLERANCE:
                print(f"\n✓ [FOLLOWER {i} PHASE 4] Complete,heading aligned!")
                print(f"✓✓✓ FORMATION LOCKED ✓✓✓\n")
                phase = 5

        # PHASE 5: TRACK - Maintain formation
        else:
            await follower.set_position(target_n, target_e, target_d, leader_heading)

            if iteration % 50 == 0:  # Print every 5 seconds
                print(f"[FOLLOWER {i} PHASE 5] Maintain formation"
                      f"Current target position: N={follower_pos['n']:.2f}, E={follower_pos['e']:.2f}, D={follower_pos['d']:.2f}"
                      f"Pos_error={pos_error:.2f}m, Head_error={heading_error:.1f}°")

        iteration += 1
        await asyncio.sleep(UPDATE_RATE)

async def main():
    # Initialize drones
    leader = DroneController(DRONE_PORTS[0], "Leader",0)
    follower1 = DroneController(DRONE_PORTS[1], "Follower1",1)
    follower2 = DroneController(DRONE_PORTS[2], "Follower2",2)
    
    # Connect both drones
    await asyncio.gather(
        leader.connect(),
        follower1.connect(),
        follower2.connect()
    )
    
    # Setup both drones
    await asyncio.gather(
        leader.setup(0),
        follower1.setup(1),
        follower2.setup(2)
    )
    
    # Arm and takeoff
    await asyncio.gather(
        leader.arm_and_takeoff(),
        follower1.arm_and_takeoff(),
        follower2.arm_and_takeoff()
    )
    
    print("Both drones in air now!")
    
    # Start position update tasks
    position_task_leader = asyncio.create_task(leader.start_position_updates())
    position_task_follower1 = asyncio.create_task(follower1.start_position_updates())
    position_task_follower2 = asyncio.create_task(follower2.start_position_updates())
    # Start offboard mode for both
    await asyncio.gather(
        leader.start_offboard(0),
        follower1.start_offboard(1),
        follower2.start_offboard(2)
    )
    
    # Run formation control
    try:
        await asyncio.gather(
            leader_behavior(leader,TARGET_HEADING),
            follower_behavior(leader, follower1,1),
            follower_behavior(leader, follower2,2)
            )
    except KeyboardInterrupt:
        print("\nStopping formation control...")
    finally:
        position_task_leader.cancel()
        position_task_follower1.cancel()
        position_task_follower2.cancel()
    
    # Land both drones
    await asyncio.gather(
        leader.land(),
        follower1.land(),
        follower2.land()
    )
    
    print("Mission complete!")


if __name__ == "__main__":
    asyncio.run(main())
