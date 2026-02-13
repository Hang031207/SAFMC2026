# Built on 3d_line_mavsdk_v5.py, which can already complete the course
# Trying to increase the speed, so the result can be a bit unstable, so I open a new document to edit

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import math

# Port Values
DRONE_PORTS = [14581,14582,14583]

# Global Coordinates
START_GATE_COOR = [2.5, 5.0]
DRONE_1_COOR    = [0.0, 3.0]
DRONE_2_COOR    = [0.0, 6.0]
DRONE_3_COOR    = [0.0, 9.0]
GATE_FRONT_COOR = [1.5, 5.0]

# Waypoints
WAYPOINT1 = [ 7.0,  5.0]
WAYPOINT2 = [11.0,  4.0]
WAYPOINT3 = [19.0,  4.0]
WAYPOINT4 = [19.0, -6.0]
WAYPOINT5 = [-0.5, -6.0]
WAYPOINT6 = [-7.0, -1.5]
WAYPOINT7 = [-4.0,  1.5]
WAYPOINT8 = [-3.5,  5.0]

#Parameters
HOVER_ALT               = 1.0        # NED: -ve = up
FOLLOWER1_DISTANCE      = 1.5        # meters behind drone
FOLLOWER2_DISTANCE      = 3.0        # meters behind
HEIGHT_OFFSET           = 0.5        # meters above the drone
FORMATION_HEIGHT_OFFSET = 0.0        # same altitude as drone
UPDATE_RATE             = 1.0 / 10.0 # 10Hz update rate
POSITION_TOLERANCE      = 0.2        # meters, considered in position
HEADING_TOLERANCE       = 5.0        # degrees, considered aligned
HEIGHT_TOLERANCE        = 0.2        # meters
TARGET_HEADING          = 90         # degrees, for starting formation only
NUM_POINTS              = 80         # for curve maneuver function

class DroneController:
    def __init__(self, port, name, i):
        self.drone          = System(port=50051+i)
        self.port           = port
        self.name           = name
        self.position       = {'n': 0, 'e': 0, 'd': 0}
        self.heading        = 0.0
        self._position_task = None
        self.id             = 0
        
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
        if self.id == 1:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT)
        elif self.id == 2:
            await self.drone.action.set_takeoff_altitude(HOVER_ALT + HEIGHT_OFFSET)
        elif self.id == 3:
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
    
    async def start_offboard(self):
        print(f"Starting offboard mode for {self.name}...")
        # Send initial setpoint before starting offboard
        if self.id == 1:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT, 0.0)
            )
        elif self.id == 2:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -HOVER_ALT -HEIGHT_OFFSET, 0.0)
            )
        elif self.id == 3:
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


def calculate_follower_position(id, drone_pos, drone_heading, distance, height_offset):
    """
    Calculate follower position behind the drone

    Args:
        drone_pos: dict with 'n', 'e', 'd' keys (NED coordinates)
        drone_heading: heading in degrees (0-360, 0=North)
        distance: distance behind drone in meters
        height_offset: height difference from drone
    
    Returns:
        tuple: (north, east, down) position for follower
    """
    # Convert heading to radians
    heading_rad = math.radians(drone_heading)
    
    # Calculate position behind drone
    # Behind means opposite direction of heading
    follower_n = drone_pos['n'] - distance * math.cos(heading_rad)
    follower_e = drone_pos['e'] - distance * math.sin(heading_rad)
    follower_d = drone_pos['d'] + height_offset
    
    follower_global_n, follower_global_e = coordinate_transformation(id, follower_n, follower_e, 1)

    return follower_global_n, follower_global_e, follower_d

def drone_assignment(start_gate_coor,drone_1_coor,drone_2_coor,drone_3_coor):
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
        print(f"\nDrone 1 is closest to the start gate: {min}m.")
        print(f"Assign drone 1 as leader\n")
        return 1
    elif min == drone_2_to_start_gate:
        print(f"\nDrone 2 is closest to the start gate: {min}m.")
        print(f"Assign drone 2 as leader\n")
        return 2
    elif min == drone_3_to_start_gate:
        print(f"\nDrone 3 is closest to the start gate: {min}m.")
        print(f"Assign drone 3 as leader\n")
        return 3

def coordinate_transformation(id, p2_n, p2_e, i):
    """
    Point 1: Origin global coordinate (0,0)
    Point 2: Waypoint global coordinate
    Point 3: Drone (match id) global coordinate

    i argument:
        0 : Convert from global to local coordinate
        1 : Convert from local to global coordinate
    """

    if id == 1:
        p3_n = DRONE_1_COOR[1]
        p3_e = DRONE_1_COOR[0]
    elif id == 2:
        p3_n = DRONE_2_COOR[1]
        p3_e = DRONE_2_COOR[0]
    elif id == 3:
        p3_n = DRONE_3_COOR[1]
        p3_e = DRONE_3_COOR[0]
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
            print(f"{[drone.id]} Heartbeat during Mission: N={cur_n:.2f}, E={cur_e:.2f}, D={cur_d:.2f}, Heading={cur_h:.2f}")
        await drone.set_position(cur_n,cur_e,cur_d,cur_h)
        if (remaining_pos < POSITION_TOLERANCE) and (remaining_h < HEADING_TOLERANCE):
            return end_heading
        await asyncio.sleep(dt)

async def curve_maneuver(drone, start_n, start_e, start_d, start_heading, end_n, end_e, end_d, v_default, turn_default, i, f):
    """
    start_n/e/d : start local coordinate
    end_n/e/d : end local coordinate
    end_heading is defined to be facing the direction of motion
    v_default : An argument to tune the drone maneuver speed
    turn_default: An argument to tune the drone turning speed
    i: -1 means CCW half, +1 means CW half
    f: An argument to specify if want to exit the semi-circle maneuver early (stands for fraction)
        Default value is 1, which is to go through the whole semicircle
    """
    center_local_n = (start_n + end_n)/2
    center_local_e = (start_e + end_e)/2
    delta_n = end_n - start_n
    delta_e = end_e - start_e
    delta_d = end_d - start_d
    
    distance = math.sqrt( delta_n**2 + delta_e**2 )
    if distance < POSITION_TOLERANCE: 
        print("[Error] The starting point is too closed to the ending point, curve maneuver aborted!\n")
        return
    
    R = distance/2.0
    start_angle = math.atan2( start_e-center_local_e, start_n-center_local_n ) # in radian
    # Note: if the first argument is 0, atan2 will assign it as 1st or 4th quadrant, which is +0 and +pi
    # Based on f, the actual endpoint might be different, updated here
    real_theta = start_angle + f * math.pi * i
    real_local_n = center_local_n + R * math.cos(real_theta)
    real_local_e = center_local_e + R * math.sin(real_theta)
    real_local_d = start_d + f * delta_d
    real_local_h = math.degrees( real_theta + (math.pi/2) * i) #in degree
    real_local_h = smooth_turn(real_local_h)
    
    """
    print(f"Curve path center of the maneuver: N={center_local_n:.2f}, E={center_local_e:.2f}")
    print(f"Distance: {distance:.2f}m")
    print(f"Start angle: {start_angle:.1f} rad")
    print(f"Real theta: {real_theta:.1f}")
    print(f"real_local_n: {real_local_n:.2f}")
    print(f"real_local_e: {real_local_e:.2f}")
    print(f"real_local_d: {real_local_d:.2f}")
    print(f"real_local_h: {real_local_h:.2f}")
    """
    iteration = 0
    for j in range(NUM_POINTS + 1):
        theta = start_angle + f * math.pi * (j / NUM_POINTS) * i # in radian
        cur_n = center_local_n + R * math.cos(theta)
        cur_e = center_local_e + R * math.sin(theta)
        cur_d = start_d + f * delta_d * (j / NUM_POINTS)
        
        tgt_theta_deg = math.degrees( theta + (math.pi/2) * i ) #in degree
        cur_h = smooth_turn(tgt_theta_deg)
        
        iteration += 1
        if iteration % 10 == 0:
            print(f"[drone] Heartbeart during Mission: N={cur_n:.2f}, E={cur_e:.2f}, D={cur_d:.2f}, Heading={cur_h:.2f}")

        await drone.set_position(cur_n, cur_e, cur_d, cur_h)

        remaining_pos = math.sqrt((real_local_n - cur_n)**2+
                                  (real_local_e - cur_e)**2+
                                  (real_local_d - cur_d)**2)
        remaining_h = real_local_h - cur_h
        remaining_h = abs( smooth_heading_diff(remaining_h) )

        if (remaining_pos < POSITION_TOLERANCE) and (remaining_h < HEADING_TOLERANCE):
            return real_local_n, real_local_e, real_local_d, real_local_h
        await asyncio.sleep(UPDATE_RATE)

async def drone_behavior(drone,target_heading):
    """
    drone drone behavior: go to gate front, then turn to facing the gate
    """
    
    # Calculate the gate front coordinate local to the drone (Global coordinate is (1.5, 5))
    gate_front_local_n, gate_front_local_e = coordinate_transformation(drone.id, GATE_FRONT_COOR[1], GATE_FRONT_COOR[0], 0)
    if drone.id == 2:
        gate_front_local_e -= 1.5
    elif drone.id == 3:
        gate_front_local_e -= 3.0

    # Wait for position updates to stabilize
    if drone.id == 1:
        await asyncio.sleep(2)
    elif drone.id == 2:
        await asyncio.sleep(5)
    elif drone.id == 3:
        await asyncio.sleep(8)
    
    print(f"{drone.name} hovering...")
    hover_pos,current_heading = await drone.get_position()
    print(f"{drone.name} starting position: N={hover_pos['n']:.2f}, E={hover_pos['e']:.2f}, D={hover_pos['d']:.2f}")
    print(f"{drone.name} starting heading: {current_heading:.1f}°")
    print(f"{drone.name} going to local coordinate: N={gate_front_local_n:.1f}, E={gate_front_local_e:.1f}")

    # Start moving to the start gate front:
    start_altitude = -HOVER_ALT -HEIGHT_OFFSET
    await asyncio.sleep(5)
    for step in range(120):
        await drone.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], current_heading
        )
        # Note: Gazebo coordinates are expressed in E first, then N. But for function defined in NED frame, it is N first, then E
        if step % 10 == 0:
            drone_pos,_ = await drone.get_position()
            pos_error = math.sqrt(
                (drone_pos['n'] - gate_front_local_n) ** 2 +
                (drone_pos['e'] - gate_front_local_e) ** 2
            )
            if pos_error > POSITION_TOLERANCE:
                print(f"Hang on, {drone.name} still got this error margin: {pos_error:.2f}m")
            else:
                print(f"<-- {drone.name} FORMATION LOCKED -->")
                print(f"drone position before stabilizing: N={drone_pos['n']:.2f}m, E={drone_pos['e']:.2f}m")
                t = step/10
                print(f"Time spent on shifting the drone to the gate front: {t}s")
                break
        await asyncio.sleep(UPDATE_RATE)

    print(f"{drone.name} Stabilizing...")
    for _ in range (50):
        await drone.set_position(
            gate_front_local_n, gate_front_local_e, start_altitude, current_heading
        )
        await asyncio.sleep(UPDATE_RATE)
    
    # Gradually turn to target heading over 3 seconds
    _, start_heading = await drone.get_position()
    print(f"{drone.name} current heading before turn: {start_heading:.1f}°")

    heading_diff = target_heading - start_heading
    heading_diff = smooth_heading_diff(heading_diff)

    steps = 30
    for i in range(steps + 1):
        progress = i / steps
        current_heading = start_heading + heading_diff * progress
        current_heading = smooth_turn(current_heading)

        await drone.set_position(
            gate_front_local_n, gate_front_local_e, start_altitude, current_heading
        )

        if i%10==0:
            print(f"{drone.name} turning: {current_heading:.1f}° (progress: {progress*100:.0f}%)")
        await asyncio.sleep(UPDATE_RATE)
    
    # Hold current attitude before starting to move
    print(f"{drone.name} holding attitude, wait for followers to align")
    if drone.id == 1:
        await asyncio.sleep(2)
    elif drone.id == 2:
        await asyncio.sleep(8)
    elif drone.id == 3:
        await asyncio.sleep(5)

    # Transform target coordinate into drone drone's coordinate
    endpoint1_local_n, endpoint1_local_e = coordinate_transformation(drone.id, WAYPOINT1[1], WAYPOINT1[0], 0)

    # Start mission 1: Fly pass the start gate
    print("\n<-- Start mission 1, move pass start gate -->\n"
          f"[{drone.name}] Current local position: N={gate_front_local_n:.2f}, E={gate_front_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {current_heading:.1f}°\n"
          f"Target endpoint: N={endpoint1_local_n:.2f}, E={endpoint1_local_e:.2f}, D={start_altitude:.2f}\n")
    end_heading_1 = await straight_maneuver(drone,
                            gate_front_local_n, gate_front_local_e, start_altitude, current_heading,
                            endpoint1_local_n, endpoint1_local_e, start_altitude,
                            2.0, 1
    )
    print(f"\n<-- [{drone.name}] Mission 1 completed -->")
    await asyncio.sleep(1)


    # Transform target coordinate into drone drone's coordinate
    endpoint2_local_n, endpoint2_local_e = coordinate_transformation(drone.id, WAYPOINT2[1], WAYPOINT2[0], 0)

    # Start mission 2: Align to first gate
    print(f"\n<-- [{drone.name}] Start mission 2, align to first gate -->\n"
          f"[{drone.name}] Current local position: N={endpoint1_local_n:.2f}, E={endpoint1_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_1:.1f}°\n"
          f"Target endpoint: N={endpoint2_local_n:.2f}, E={endpoint2_local_e:.2f}, D={start_altitude:.2f}\n")
    end_heading_2 = await straight_maneuver(drone,
                            endpoint1_local_n, endpoint1_local_e, start_altitude, end_heading_1,
                            endpoint2_local_n, endpoint2_local_e, start_altitude,
                            2.0, 1
    )
    print(f"<-- [{drone.name}] Mission 2 completed -->")
    await asyncio.sleep(1)


    # Transform target coordinate into drone drone's coordinate
    endpoint3_local_n, endpoint3_local_e = coordinate_transformation(drone.id, WAYPOINT3[1], WAYPOINT3[0], 0)

    # Start mission 3: Fly pass first gate
    print(f"\n<-- [{drone.name}] Start mission 3, fly pass first gate -->\n"
          f"[{drone.name}] Current local position: N={endpoint2_local_n:.2f}, E={endpoint2_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_2:.1f}°\n"
          f"Target endpoint: N={endpoint3_local_n:.2f}, E={endpoint3_local_e:.2f}, D={start_altitude:.2f}\n")
    end_heading_3 = await straight_maneuver(drone,
                            endpoint2_local_n, endpoint2_local_e, start_altitude, end_heading_2,
                            endpoint3_local_n, endpoint3_local_e, start_altitude,
                            2.0, 1
    )
    print(f"<-- [{drone.name}] Mission 3 completed -->")
    await asyncio.sleep(1)

    
    # Transform target coordinate into drone drone's coordinate
    endpoint4_local_n, endpoint4_local_e = coordinate_transformation(drone.id, WAYPOINT4[1], WAYPOINT4[0], 0)

    # Start mission 4: First curve path
    print(f"\n<-- [{drone.name}] Start mission 4, curve path to the second gate -->\n"
          f"[{drone.name}] Current local position: N={endpoint3_local_n:.2f}, E={endpoint3_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_3:.1f}°\n"
          f"Target endpoint: N={endpoint4_local_n:.2f}, E={endpoint4_local_e:.2f}, D={start_altitude:.2f}\n")
    real_local_n, real_local_e, real_local_d, end_heading_4 = await curve_maneuver(drone,
                            endpoint3_local_n, endpoint3_local_e, start_altitude, end_heading_3,
                            endpoint4_local_n, endpoint4_local_e, start_altitude,
                            1.5, 1, 1, 1.0
    )
    print("<-- [{drone.name}] Mission 4 completed -->")
    await asyncio.sleep(2)

    
    # Transform target coordinate into drone drone's coordinate
    endpoint5_local_n, endpoint5_local_e = coordinate_transformation(drone.id, WAYPOINT5[1], WAYPOINT5[0], 0)

    # Start mission 5: Fly pass second gate
    print(f"\n<-- [{drone.name}] Start mission 5, fly pass second gate -->\n"
          f"[{drone.name}] Current local position: N={endpoint4_local_n:.2f}, E={endpoint4_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_4:.1f}°\n"
          f"Target endpoint: N={endpoint5_local_n:.2f}, E={endpoint5_local_e:.2f}, D={start_altitude:.2f}\n")
    end_heading_5 = await straight_maneuver(drone,
                            endpoint4_local_n, endpoint4_local_e, start_altitude, end_heading_4,
                            endpoint5_local_n, endpoint5_local_e, start_altitude,
                            2.0, 1
    )
    print(f"<-- [{drone.name}] Mission 5 completed -->")
    await asyncio.sleep(1)

    
    # Transform target coordinate into drone drone's coordinate
    endpoint6_local_n, endpoint6_local_e = coordinate_transformation(drone.id, WAYPOINT6[1], WAYPOINT6[0], 0)

    # Start mission 6: Fly pass third gate
    print(f"\n<-- [{drone.name}] Start mission 6, fly pass third gate -->\n"
          f"[{drone.name}] Current local position: N={endpoint5_local_n:.2f}, E={endpoint5_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_5:.1f}°\n"
          f"Target endpoint: N={endpoint6_local_n:.2f}, E={endpoint6_local_e:.2f}, D={start_altitude:.2f}\n")
    end_heading_6 = await straight_maneuver(drone,
                            endpoint5_local_n, endpoint5_local_e, start_altitude, end_heading_5,
                            endpoint6_local_n, endpoint6_local_e, start_altitude,
                            2.0, 1
    )
    print(f"<-- [{drone.name}] Mission 6 completed -->")
    await asyncio.sleep(1)

    
    # Transform target coordinate into drone drone's coordinate
    endpoint7_local_n, endpoint7_local_e = coordinate_transformation(drone.id, WAYPOINT7[1], WAYPOINT7[0], 0)

    # Start mission 7: Move to end gate
    print(f"\n<-- [{drone.name}] Start mission 7, move to end gate -->\n"
          f"[{drone.name}] Current local position: N={endpoint6_local_n:.2f}, E={endpoint6_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_6:.1f}°\n"
          f"Target endpoint: N={endpoint7_local_n:.2f}, E={endpoint7_local_e:.2f}, D={start_altitude:.2f}\n")
    real_local_n, real_local_e, real_local_d, end_heading_7 = await curve_maneuver(drone,
                            endpoint6_local_n, endpoint6_local_e, start_altitude, end_heading_6,
                            endpoint7_local_n, endpoint7_local_e, start_altitude,
                            1.5, 1, 1, 0.5
    )
    print(f"<-- [{drone.name}] Mission 7 completed -->")
    await asyncio.sleep(1)


    # Transform target coordinate into drone drone's coordinate
    endpoint8_local_n, endpoint8_local_e = coordinate_transformation(drone.id, WAYPOINT8[1], WAYPOINT8[0], 0)
    
    # Start mission 8: Move to end gate
    print(f"\n<-- [{drone.name}] Start mission 8, move to end gate -->\n"
          f"[{drone.name}] Current local position: N={real_local_n:.2f}, E={real_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {end_heading_7:.1f}°\n"
          f"Target endpoint: N={endpoint8_local_n:.2f}, E={endpoint8_local_e:.2f}, D={start_altitude:.2f}\n")
    end_heading_8 = await straight_maneuver(drone,
                            real_local_n, real_local_e, start_altitude, end_heading_7,
                            endpoint8_local_n, endpoint8_local_e, start_altitude,
                            2.0, 1
    )
    print(f"<-- [{drone.name}] Mission 8 completed -->")
    await asyncio.sleep(1)
    
    # Gradually turn to target heading over 3 seconds
    print("Align the drone with the end gate before mission")
    print(f"drone current heading before turn: {end_heading_8:.1f}°")

    heading_diff = target_heading - end_heading_8
    heading_diff = smooth_heading_diff(heading_diff)

    steps = 30
    for i in range(steps + 1):
        progress = i / steps
        current_heading = end_heading_8 + heading_diff * progress
        current_heading = smooth_turn(current_heading)

        await drone.set_position(
            endpoint8_local_n, endpoint8_local_e, start_altitude, current_heading
        )

        if i%10==0:
            print(f"drone turning: {current_heading:.1f}° (progress: {progress*100:.0f}%)")
        await asyncio.sleep(UPDATE_RATE)
    
    # Start mission 9: Fly back to starting position
    print(f"\n<-- [{drone.name}] Start mission 9, fly back to starting position -->\n"
          f"[{drone.name}] Current local position: N={endpoint8_local_n:.2f}, E={endpoint8_local_e:.2f}, D={start_altitude:.2f}\n"
          f"[{drone.name}] Current heading: {current_heading:.1f}°\n"
          f"Target endpoint: N={gate_front_local_n:.2f}, E={gate_front_local_e:.2f}, D={start_altitude:.2f}\n")
    
    
    end_heading_9 = await straight_maneuver(drone,
                            endpoint8_local_n, endpoint8_local_e, start_altitude, current_heading,
                            gate_front_local_n, gate_front_local_e, start_altitude,
                            2.0, 1
    )
    print(f"<-- [{drone.name}] Mission 9 completed -->")
    print(f"<-- [{drone.name}] All missions completed -->")
    
    # Hold drone position after finish maneuvering
    while True:
        await drone.set_position(
            gate_front_local_n, gate_front_local_e, start_altitude, target_heading
        )

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
        drone1.setup(),
        drone2.setup(),
        drone3.setup()
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
        drone1.start_offboard(),
        drone2.start_offboard(),
        drone3.start_offboard()
    )
    
    min = drone_assignment(START_GATE_COOR, DRONE_1_COOR, DRONE_2_COOR, DRONE_3_COOR)
    if min==1:
        role_alloc = {"drone":drone1,"Follower1":drone2,"Follower2":drone3}
    elif min==2:
        role_alloc = {"drone":drone2,"Follower1":drone1,"Follower2":drone3}
    elif min==3:
        role_alloc = {"drone":drone3,"Follower1":drone2,"Follower2":drone1}
    else:
        print(f"Invalid min returned value: {min}") 

    # Run formation control
    try:
        await asyncio.gather(
            drone_behavior(role_alloc["drone"], TARGET_HEADING),
            drone_behavior(role_alloc["Follower1"], TARGET_HEADING),
            drone_behavior(role_alloc["Follower2"], TARGET_HEADING)
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
