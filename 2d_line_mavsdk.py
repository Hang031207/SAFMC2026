import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import math

# Configuration
USE_GZ=1
if USE_GZ:
    DRONE_PORTS = [14581,14582]
else:
    LEADER_PORTS = [14561,14562]

HOVER_ALT = -2.5 # NED: negative = up (0.5m above current)
FORMATION_DISTANCE = 1  # meters behind leader
FORMATION_HEIGHT_OFFSET = 0.0  # same altitude as leader
UPDATE_RATE = 1.0 / 10.0  # 10Hz update rate
POSITION_TOLERANCE = 0.3 #meters, considered in position
HEADING_TOLERANCE = 5.0 #degrees, considered aligned

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
    
    async def setup(self):
        # Wait for drone to be ready
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"{self.name} is ready!")
                break
        
        # Set takeoff altitude
        await self.drone.action.set_takeoff_altitude(3.0)
        
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
    
    async def start_offboard(self):
        print(f"Starting offboard mode for {self.name}...")
        # Send initial setpoint before starting offboard
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, HOVER_ALT, 0.0)
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


async def leader_behavior(leader):
    """
    Leader drone behavior: hover then turn to desired heading
    """
    # Wait for position updates to stabilize
    await asyncio.sleep(2)

    # Hover at current position for 5 seconds
    print("Leader hovering...")
    hover_pos,current_heading = await leader.get_position()
    print(f"Leader starting position: N={hover_pos['n']:.2f}, E={hover_pos['e']:.2f}, D={hover_pos['d']:.2f}")
    print(f"Leader starting heading: {current_heading:.1f}°")

    for _ in range(50):  # 5 seconds at 10Hz
        await leader.set_position(
            hover_pos['n'], 
            hover_pos['e'], 
            hover_pos['d'], 
            current_heading
        )
        await asyncio.sleep(UPDATE_RATE)
    
    # Turn to desired heading
    target_heading = 225.0 # in degree
    print(f"Leader's target heading is {target_heading}°")
    
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


async def follower_behavior(leader, follower):
    """
    Phase 1: Move to position behind leader (maintain original heading)
    Phase 2: Align with leader's heading (maintain position)
    Phase 3: Track leader continuously (maintain position and heading)
    """
    print("Follower entering formation...")
    await asyncio.sleep(2)

    iteration = 0
    phase = 1
    position_locked = False
    heading_locked = False

    while True:
        # Get leader and follower position and heading
        leader_pos, leader_heading = await leader.get_position()
        follower_pos, follower_heading = await follower.get_position()

        # Calculate desired follower position
        target_n, target_e, target_d = calculate_follower_position(
            leader_pos,
            leader_heading,
            FORMATION_DISTANCE,
            FORMATION_HEIGHT_OFFSET
        )

        # Calculate current position deviation
        pos_error = math.sqrt((follower_pos['n']-target_n)**2+
                          (follower_pos['e']-target_e)**2+
                          (follower_pos['d']-target_d)**2)
        # Calculate current heading deviation
        heading_error = abs(follower_heading - leader_heading)
        if heading_error > 180:
            heading_error = 360 - heading_error        

        # Phase 1: Get into position first
        if phase == 1:
            if not position_locked:
                await follower.set_position(target_n,target_e,target_d,follower_heading)
                # Print debug info every second
                if iteration % 10 == 0:
                    print(f"[PHASE 1]: Target=(N:{target_n:.2f}, E:{target_e:.2f}, D:{target_d:.2f}), "
                    f"Current=(N:{follower_pos['n']:.2f}, E:{follower_pos['e']:.2f}, D:{follower_pos['d']:.2f}), "
                    f"Error={pos_error:.2f}m, Heading={heading_error:.1f}°")
                if pos_error < POSITION_TOLERANCE: 
                    position_locked = True
                    print("PHASE 1 COMPLETE: Follower in position!")
                    print(f"Current error: {pos_error:.2f}")
                    phase = 2
        elif phase == 2:
            if not heading_locked:
                heading_diff = leader_heading - follower_heading
                if heading_diff > 180:
                    heading_diff -= 360
                elif heading_diff < -180:
                    heading_diff += 360
                
                new_heading = follower_heading + heading_diff*0.2
                if new_heading < 0:
                    new_heading += 360
                elif new_heading >= 360:
                    new_heading -= 360

                # Send heading command to follower
                await follower.set_position(target_n, target_e, target_d, new_heading)

                if iteration % 10 == 0:
                    print(f"[PHASE 2] Current={follower_heading:.1f}°,"
                          f"Error={heading_error:.1f}°, Target={leader_heading:.1f}°")
                    
                if heading_error < HEADING_TOLERANCE: 
                    heading_locked = True
                    print("PHASE 2 COMPLETE: Follower heading aligned!")
                    print(f"Current error: {heading_error:.1f}")
                    phase = 3
        else:
            await follower.set_position(target_n,target_e,target_d,leader_heading)
            if iteration % 20 ==0:
                print(f"[TRACKING] pos_error={pos_error:.2f}m, heading_error={heading_error:.1f}°")
                    
        iteration += 1
        await asyncio.sleep(UPDATE_RATE)

async def main():
    # Initialize drones
    leader = DroneController(DRONE_PORTS[0], "Leader",0)
    follower = DroneController(DRONE_PORTS[1], "Follower",1)
    
    # Connect both drones
    await asyncio.gather(
        leader.connect(),
        follower.connect()
    )
    
    # Setup both drones
    await asyncio.gather(
        leader.setup(),
        follower.setup()
    )
    
    # Arm and takeoff
    await asyncio.gather(
        leader.arm_and_takeoff(),
        follower.arm_and_takeoff()
    )
    
    print("Both drones in air now!")
    
    # Start position update tasks
    position_task_leader = asyncio.create_task(leader.start_position_updates())
    position_task_follower = asyncio.create_task(follower.start_position_updates())
    await asyncio.sleep(3)
    
    # Start offboard mode for both
    await asyncio.gather(
        leader.start_offboard(),
        follower.start_offboard()
    )
    
    # Run formation control
    try:
        await asyncio.gather(
            leader_behavior(leader),
            follower_behavior(leader, follower)  # Follower tracks leader
        )
    except KeyboardInterrupt:
        print("\nStopping formation control...")
    finally:
        position_task_leader.cancel()
        position_task_follower.cancel()
    
    # Land both drones
    await asyncio.gather(
        leader.land(),
        follower.land()
    )
    
    print("Mission complete!")


if __name__ == "__main__":
    asyncio.run(main())
