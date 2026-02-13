#v2 upgrade: Based on the constraint of starting pad, add a leader assign mechanism
#After 3 drones stabilized on air, the drone closest to the mid-line of the starting gate will be assign leader
#This is to make minimize the maneuvers require to form the formation, which I think is necessary for the given starting pad
#Leader will move to the front of the gate, while the other drones will follow at the back, based on the space constraint I think can only do a 1m separation

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

# Coordination
START_GATE_COOR = [2.5, 5.0]
DRONE_1_COOR = [0.0, 3.0]
DRONE_2_COOR = [0.0, 6.0]
DRONE_3_COOR = [0.0, 9.0]
GATE_FRONT_COOR = [1.5, 5.0]

#Parameters
HOVER_ALT = 1.0 # NED: -ve = up
FOLLOWER1_DISTANCE = 1.5  # meters behind leader
FOLLOWER2_DISTANCE = 3.0 # meters behind
HEIGHT_OFFSET = 0.5 # meters above the leader
FORMATION_HEIGHT_OFFSET = 0.0  # same altitude as leader
UPDATE_RATE = 1.0 / 10.0  # 10Hz update rate
POSITION_TOLERANCE = 0.15 #meters, considered in position
HEADING_TOLERANCE = 5.0 #degrees, considered aligned
TARGET_HEADING = 90 #degrees

class DroneController:
    def __init__(self, port, name, i):
        self.drone = System(port=50051+i)
        self.port = port
        self.name = name
        self.position = {'n': 0, 'e': 0, 'd': 0}
        self.heading = 0.0
        self._position_task = None
        self.id = 0
        
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

def leader_assignment(start_gate_coor,drone_1_coor,drone_2_coor,drone_3_coor):
    #Only calculate based on horizontal separation
    drone_1_to_start_gate =  math.sqrt(
            (drone_1_coor[0] - start_gate_coor[0]) ** 2 +
            (drone_1_coor[1] - start_gate_coor[1]) ** 2
        )
    drone_2_to_start_gate =  math.sqrt(
            (drone_2_coor[0] - start_gate_coor[0]) ** 2 +
            (drone_2_coor[1] - start_gate_coor[1]) ** 2
        )
    drone_3_to_start_gate =  math.sqrt(
            (drone_3_coor[0] - start_gate_coor[0]) ** 2 +
            (drone_3_coor[1] - start_gate_coor[1]) ** 2
        )

    #Determine the drone closest to the start gate
    min = drone_1_to_start_gate
    if drone_2_to_start_gate < drone_1_to_start_gate:
        min = drone_2_to_start_gate
        if drone_3_to_start_gate < drone_2_to_start_gate:
            min = drone_3_to_start_gate
    elif drone_2_to_start_gate > drone_1_to_start_gate:
        if drone_3_to_start_gate < drone_1_to_start_gate:
           min = drone_3_to_start_gate
    
    if min == drone_1_to_start_gate:
        print(f"Drone 1 is closest to the start gate: {min}m.")
        print(f"Assign drone 1 as leader")
        return 1
    elif min == drone_2_to_start_gate:
        print(f"Drone 2 is closest to the start gate: {min}m.")
        print(f"Assign drone 2 as leader")
        return 2
    elif min == drone_3_to_start_gate:
        print(f"Drone 3 is closest to the start gate: {min}m.")
        print(f"Assign drone 3 as leader")
        return 3

async def leader_behavior(leader,target_heading,id):
    """
    Leader drone behavior: go to gate front, then turn to facing the gate
    """
    
    # Calculate the gate front coordinate the leader (Global coordinate is (1.5, 5))
    if id==1:
        gate_front_local_e = GATE_FRONT_COOR[0] - DRONE_1_COOR[0]
        gate_front_local_n = GATE_FRONT_COOR[1] - DRONE_1_COOR[1]
    elif id==2:
        gate_front_local_e = GATE_FRONT_COOR[0] - DRONE_2_COOR[0]
        gate_front_local_n = GATE_FRONT_COOR[1] - DRONE_2_COOR[1]
    elif id==3:
        gate_front_local_e = GATE_FRONT_COOR[0] - DRONE_3_COOR[0]
        gate_front_local_n = GATE_FRONT_COOR[1] - DRONE_3_COOR[1]
    # Wait for position updates to stabilize
    await asyncio.sleep(2)
    
    print(f"Leader assigned to drone {id}")
    print("Leader hovering...")
    hover_pos,current_heading = await leader.get_position()
    print(f"Leader starting position: N={hover_pos['n']:.2f}, E={hover_pos['e']:.2f}, D={hover_pos['d']:.2f}")
    print(f"Leader starting heading: {current_heading:.1f}°")
    print(f"Leader going to local coordinate: N={gate_front_local_n:.1f}, E={gate_front_local_e:.1f}")

    # Start moving to the gate front (1.5,5):
    for step in range(120):
        await leader.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], current_heading
        )
        # Note: Gazebo coordinates are expressed in E first, then N. But for function defined in NED frame, it is N first, then E
        if step % 10 == 0:
            leader_pos,_ = await leader.get_position()
            pos_error = math.sqrt(
                (leader_pos['n'] - gate_front_local_n) ** 2 +
                (leader_pos['e'] - gate_front_local_e) ** 2
            )
            if pos_error > POSITION_TOLERANCE:
                print(f"Hang on, leader still got this error margin: {pos_error:.2f}m")
            else:
                print("Leader successfully shifted to the gate front")
                print(f"Leader position before stabilizing: N={leader_pos['n']:.2f}m, E={leader_pos['e']:.2f}m")
                break
        await asyncio.sleep(UPDATE_RATE)

    print("Leader Stabilizing...")
    for _ in range (100):
        await leader.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], current_heading
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

    steps = 30
    for i in range(steps + 1):
        progress = i / steps
        current_heading = start_heading + heading_diff * progress

        if current_heading < 0:
            current_heading += 360
        elif current_heading >=360:
            current_heading -= 360

        await leader.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], current_heading
        )

        if i%10==0:
            print(f"Leader turning: {current_heading:.1f}° (progress: {progress*100:.0f}%)")
        await asyncio.sleep(UPDATE_RATE)
    
    # Hold final heading
    print(f"Leader holding final heading {target_heading}°...")
    while True:
        await leader.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], target_heading
        )
        await asyncio.sleep(UPDATE_RATE)


async def follower_behavior(leader, follower,id,i):
    """
    id: To specify the compensation on target position required based on the drone location
    i : To identify the drone as follower 1 or follower 2
    In the line formation, the leader will be followed by follower 1, then follower 2
    """

    print(f"[FOLLOWER {i}] Initializing...")
    await asyncio.sleep(2)

    follower_start_pos, _ = await follower.get_position()
    print(f"\n[FOLLOWER {i}] Initial position N={follower_start_pos['n']:.2f}, "
          f"E={follower_start_pos['e']:.2f}, D={follower_start_pos['d']:.2f}")
    print(f"Follower {i} assigned to drone {id}\n")

    iteration = 0
    phase = 1

    # Phase timing
    LEADER_TIME = 25  # Wait until leader done its maneuver

    # Calculate relative vector between the drone spawn points
    # This will be used for calculation of compensation later
    vec_1to2 = [ DRONE_2_COOR[0]-DRONE_1_COOR[0], DRONE_2_COOR[1]-DRONE_1_COOR[1] ]
    vec_1to3 = [ DRONE_3_COOR[0]-DRONE_1_COOR[0], DRONE_3_COOR[1]-DRONE_1_COOR[1] ]
    vec_2to3 = [ DRONE_3_COOR[0]-DRONE_2_COOR[0], DRONE_3_COOR[1]-DRONE_2_COOR[1] ]

    while True:
        leader_pos, leader_heading = await leader.get_position()
        follower_pos, follower_heading = await follower.get_position()
        
        # Calculate target position based on i
        if i==1:
            target_n, target_e, target_d = calculate_follower_position(
                leader_pos, leader_heading,
                FOLLOWER1_DISTANCE, FORMATION_HEIGHT_OFFSET
            )
        elif i==2:
            target_n, target_e, target_d = calculate_follower_position(
                leader_pos, leader_heading,
                FOLLOWER2_DISTANCE, FORMATION_HEIGHT_OFFSET
            )

        if iteration == 20:
            print(f"[FOLLOWER {i}] Leader's position at 20 seconds: N={leader_pos['n']:.2f}, E={leader_pos['e']:.2f}, D={leader_pos['d']:.2f}")
            print(f"[FOLLOWER {i}] Leader's heading at 20 seconds: {leader_heading:.2f}°")
            print(f"[FOLLOWER {i}] Target position at 20 seconds: N={target_n:.2f}, E={target_e:.2f}, D={target_d:.2f}\n")
        
        # Based on leader's id and follower's id, determine the compensation required
        if leader.id==1:
            if id==2:
                target_n -= vec_1to2[1]
                target_e -= vec_1to2[0]
            elif id==3:
                target_n -= vec_1to3[1]
                target_e -= vec_1to3[0]
        elif leader.id==2:
            if id==1:
                target_n += vec_1to2[1]
                target_e += vec_1to2[0]
            elif id==3:
                target_n -= vec_2to3[1]
                target_e -= vec_2to3[0]
        elif leader.id==3:
            if id==2:
                target_n += vec_2to3[1]
                target_e += vec_2to3[0]
            elif id==1:
                target_n += vec_1to3[1]
                target_e += vec_1to3[0]

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
                time_remaining = max(0, LEADER_TIME - iteration * UPDATE_RATE)
                print(f"[FOLLOWER {i} PHASE 1] Hovering while leader turns... ({time_remaining:.1f}s remaining)")

            # Move to next phase after leader finishes turning
            if (iteration * UPDATE_RATE) >= LEADER_TIME:
                print(f"\n✓ [FOLLOWER {i} PHASE 1] Complete, Leader turned, starting positioning\n")
                phase = 2

        # PHASE 2: POSITION - Move to target while maintaining start altitude (collision avoidance)
        elif phase == 2:
            await follower.set_position(
                target_n, target_e, follower_start_pos['d'],
                follower_heading  # Keep current heading
            )
            # Reminder: .set_position() use NED frame, and target_d here is a -ve value

            if iteration % 10 == 0:
                print(f"[FOLLOWER {i} PHASE 2] Current leader position: N={leader_pos['n']:.2f}, E={leader_pos['e']:.2f}, D={leader_pos['d']:.2f}")
                print(f"[FOLLOWER {i} PHASE 2] Current leader heading: {leader_heading:.2f}°")
                print(f"[FOLLOWER {i} PHASE 2] Current target position: N={target_n:.2f}, E={target_e:.2f}, D={follower_start_pos['d']:.2f}")
                print(f"[FOLLOWER {i} PHASE 2] Moving to leader's back: "
                      f"Error={pos_error:.2f}m, height_error={height_error:.2f}m\n")

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
                print(f"Iteration value now: {iteration}")
                phase = 5

        # PHASE 5: TRACK - Maintain formation
        else:
            await follower.set_position(target_n, target_e, target_d, leader_heading)

            if iteration % 50 == 0:  # Print every 5 seconds
                print(f"[FOLLOWER {i} PHASE 5] Maintain formation\n"
                      f"Current target position: N={follower_pos['n']:.2f}, E={follower_pos['e']:.2f}, D={follower_pos['d']:.2f}\n"
                      f"Pos_error={pos_error:.2f}m, Head_error={heading_error:.1f}°")

        iteration += 1
        await asyncio.sleep(UPDATE_RATE)

async def main():
    # Initialize drones
    drone1 = DroneController(DRONE_PORTS[0], "Drone1",0)
    drone2 = DroneController(DRONE_PORTS[1], "Drone2",1)
    drone3 = DroneController(DRONE_PORTS[2], "Drone3",2)
    
    # Assign drone id
    drone1.id = 1
    drone2.id = 2
    drone3.id = 3
    
    # Connect both drones
    await asyncio.gather(
        drone1.connect(),
        drone2.connect(),
        drone3.connect()
    )
    
    # Setup both drones
    await asyncio.gather(
        drone1.setup(0),
        drone2.setup(1),
        drone3.setup(2)
    )
    
    # Arm and takeoff
    await asyncio.gather(
        drone1.arm_and_takeoff(),
        drone2.arm_and_takeoff(),
        drone3.arm_and_takeoff()
    )
    
    print("Both drones in air now!")
    
    # Start position update tasks
    position_task_drone1 = asyncio.create_task(drone1.start_position_updates())
    position_task_drone2 = asyncio.create_task(drone2.start_position_updates())
    position_task_drone3 = asyncio.create_task(drone3.start_position_updates())
    
    # Start offboard mode for both
    await asyncio.gather(
        drone1.start_offboard(0),
        drone2.start_offboard(1),
        drone3.start_offboard(2)
    )
    
    min = leader_assignment(START_GATE_COOR, DRONE_1_COOR, DRONE_2_COOR, DRONE_3_COOR)
    if min==1:
        role_alloc = {"Leader":drone1,"Follower1":drone2,"Follower2":drone3}
    elif min==2:
        role_alloc = {"Leader":drone2,"Follower1":drone1,"Follower2":drone3}
    elif min==3:
        role_alloc = {"Leader":drone3,"Follower1":drone2,"Follower2":drone1}
    else:
        print(f"Invalid min returned value: {min}") 
    # Run formation control
    try:
        await asyncio.gather(
            leader_behavior(role_alloc["Leader"], TARGET_HEADING, role_alloc["Leader"].id),
            follower_behavior(role_alloc["Leader"], role_alloc["Follower1"], role_alloc["Follower1"].id, 1),
            follower_behavior(role_alloc["Leader"], role_alloc["Follower2"], role_alloc["Follower2"].id, 2)
            )
    except KeyboardInterrupt:
        print("\nStopping formation control...")
    finally:
        position_task_drone1.cancel()
        position_task_drone2.cancel()
        position_task_drone3.cancel()
    
    # Land both drones
    await asyncio.gather(
        drone1.land(),
        drone2.land(),
        drone3.land()
    )
    
    print("Mission complete!")


if __name__ == "__main__":
    asyncio.run(main())
