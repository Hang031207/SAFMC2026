#Built on 3d_line_mavsdk_v2.py, now with the drones lining up in front the gate, can start to add in maneuvers to pass the gates
#Start from straight line maneuver, then curve path maneuver
#The idea is to define async function, which given starting point and ending point, as well as other necessary parameters to tune the maneuver behaviour, can control the leader to move to the setpoint
#Main things to evaluate: leader motion smoothness when maneuvering the path, followers' performance when following

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import math

# Configuration
USE_GZ=1
if USE_GZ:
    DRONE_PORTS = [14581,14582,14583]
else:
    DRONE_PORTS = [14561,14562,14563]

# Global Coordinates
START_GATE_COOR = [2.5, 5.0]
DRONE_1_COOR    = [0.0, 3.0]
DRONE_2_COOR    = [0.0, 6.0]
DRONE_3_COOR    = [0.0, 9.0]
GATE_FRONT_COOR = [1.5, 5.0]

# Waypoints
WAYPOINT1 = [7.0, 5.0]
WAYPOINT2 = [11.0, 4.0]
WAYPOINT3 = [19.0, 4.0]
WAYPOINT4 = [19.0, -4.0]
WAYPOINT5 = [-2.5, -4.0]
WAYPOINT6 = [-3.5, 5.0]
WAYPOINT7 = GATE_FRONT_COOR

#Parameters
HOVER_ALT               = 1.0        # NED: -ve = up
FOLLOWER1_DISTANCE      = 1.5        # meters behind leader
FOLLOWER2_DISTANCE      = 3.0        # meters behind
HEIGHT_OFFSET           = 0.5        # meters above the leader
FORMATION_HEIGHT_OFFSET = 0.0        # same altitude as leader
UPDATE_RATE             = 1.0 / 10.0 # 10Hz update rate
POSITION_TOLERANCE      = 0.2        # meters, considered in position
HEADING_TOLERANCE       = 5.0        # degrees, considered aligned
HEIGHT_TOLERANCE        = 0.2        # meters
TARGET_HEADING          = 90         # degrees
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


def calculate_follower_position(id, leader_pos, leader_heading, distance, height_offset):
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
    
    follower_global_n, follower_global_e = coordinate_transformation(id, follower_n, follower_e, 1)

    return follower_global_n, follower_global_e, follower_d

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
    if delta_h > 180:
        delta_h -= 360
    elif delta_h < -180:
        delta_h += 360

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

        if cur_h < 0:
            cur_h += 360
        elif cur_h >= 360:
            cur_h -= 360

        iteration += 1
        if iteration % 10 == 0:
            print(f"[Leader] Heartbeart during Mission: N={cur_n:.2f}, E={cur_e:.2f}, D={cur_d:.2f}, Heading={cur_h:.2f}\n")
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
    if real_local_h < 0:
        real_local_h += 360.0
    elif real_local_h >= 360.0:
        real_local_h -= 360.0
    
    print(f"Curve path center of the maneuver: N={center_local_n:.2f}, E={center_local_e:.2f}\n")
    print(f"Distance: {distance:.2f}m")
    print(f"Start angle: {start_angle:.1f} rad")
    print(f"Real theta: {real_theta:.1f}")
    print(f"real_local_n: {real_local_n:.2f}")
    print(f"real_local_e: {real_local_e:.2f}")
    print(f"real_local_d: {real_local_d:.2f}")
    print(f"real_local_h: {real_local_h:.2f}")
    iteration = 0
    for j in range(NUM_POINTS + 1):
        theta = start_angle + f * math.pi * (j / NUM_POINTS) * i # in radian
        cur_n = center_local_n + R * math.cos(theta)
        cur_e = center_local_e + R * math.sin(theta)
        cur_d = start_d + f * delta_d * (j / NUM_POINTS)
        
        tgt_theta_deg = math.degrees( theta + (math.pi/2) * i ) #in degree
        if tgt_theta_deg < 0:
            tgt_theta_deg += 360.0
        elif tgt_theta_deg >= 360.0:
            tgt_theta_deg -= 360.0        
        cur_h = tgt_theta_deg
        
        iteration += 1
        if iteration % 10 == 0:
            print(f"[Leader] Heartbeart during Mission: N={cur_n:.2f}, E={cur_e:.2f}, D={cur_d:.2f}, Heading={cur_h:.2f}\n")

        await drone.set_position(cur_n, cur_e, cur_d, cur_h)

        remaining_pos = math.sqrt((real_local_n - cur_n)**2+
                                  (real_local_e - cur_e)**2+
                                  (real_local_d - cur_d)**2)
        remaining_h = real_local_h - cur_h
        if remaining_h > 180: # Can think of the case of real_local_h=359, cur_h=1
            remaining_h -= 360
        elif remaining_h < -180:
            remaining_h += 360
        remaining_h = abs(remaining_h)

        if (remaining_pos < POSITION_TOLERANCE) and (remaining_h < HEADING_TOLERANCE):
            return real_local_n, real_local_e, real_local_d, real_local_h
        await asyncio.sleep(UPDATE_RATE)

async def leader_behavior(leader,target_heading):
    """
    Leader drone behavior: go to gate front, then turn to facing the gate
    """
    
    # Calculate the gate front coordinate local to the leader (Global coordinate is (1.5, 5))
    gate_front_local_n, gate_front_local_e = coordinate_transformation(leader.id, GATE_FRONT_COOR[1], GATE_FRONT_COOR[0], 0)

    # Wait for position updates to stabilize
    await asyncio.sleep(2)
    
    print(f"[LEADER] Leader assigned to drone {leader.id}")
    print("Leader hovering...")
    hover_pos,current_heading = await leader.get_position()
    print(f"Leader starting position: N={hover_pos['n']:.2f}, E={hover_pos['e']:.2f}, D={hover_pos['d']:.2f}")
    print(f"Leader starting heading: {current_heading:.1f}°")
    print(f"Leader going to local coordinate: N={gate_front_local_n:.1f}, E={gate_front_local_e:.1f}")

    # Start moving to the start gate front:
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
                print("<-- LEADER FORMATION LOCKED -->")
                print(f"Leader position before stabilizing: N={leader_pos['n']:.2f}m, E={leader_pos['e']:.2f}m")
                t = step/10
                print(f"Time spent on shifting the drone to the gate front: {t}s")
                break
        await asyncio.sleep(UPDATE_RATE)

    print("Leader Stabilizing...")
    for _ in range (50):
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
    
    # Hold current attitude before starting to move
    print(f"Leader holding attitude, wait for followers to align")
    for _ in range (200):
        await leader.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], current_heading
        )
        await asyncio.sleep(UPDATE_RATE)


    # Transform target coordinate into leader drone's coordinate
    endpoint1_local_n, endpoint1_local_e = coordinate_transformation(leader.id, WAYPOINT1[1], WAYPOINT1[0], 0)

    # Start mission 1: Fly pass the start gate
    print("\n[Leader] Start mission 1, move pass start gate.\n"
          f"[Leader] Current local position: N={gate_front_local_n:.2f}, E={gate_front_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {current_heading:.1f}°\n"
          f"Target endpoint: N={endpoint1_local_n:.2f}, E={endpoint1_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    end_heading_1 = await straight_maneuver(leader,
                            gate_front_local_n, gate_front_local_e, hover_pos['d'], current_heading,
                            endpoint1_local_n, endpoint1_local_e, hover_pos['d'],
                            0.5, 1
    )
    print("<-- Mission 1 completed -->")
    await asyncio.sleep(1)


    # Transform target coordinate into leader drone's coordinate
    endpoint2_local_n, endpoint2_local_e = coordinate_transformation(leader.id, WAYPOINT2[1], WAYPOINT2[0], 0)

    # Start mission 2: Align to first gate
    print("\n[Leader] Start mission 2, align to first gate.\n"
          f"[Leader] Current local position: N={endpoint1_local_n:.2f}, E={endpoint1_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_1:.1f}°\n"
          f"Target endpoint: N={endpoint2_local_n:.2f}, E={endpoint2_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    end_heading_2 = await straight_maneuver(leader,
                            endpoint1_local_n, endpoint1_local_e, hover_pos['d'], end_heading_1,
                            endpoint2_local_n, endpoint2_local_e, hover_pos['d'],
                            0.5, 1
    )
    print("<-- Mission 2 completed -->")
    await asyncio.sleep(1)


    # Transform target coordinate into leader drone's coordinate
    endpoint3_local_n, endpoint3_local_e = coordinate_transformation(leader.id, WAYPOINT3[1], WAYPOINT3[0], 0)

    # Start mission 3: Fly pass first gate
    print("\n[Leader] Start mission 3, fly pass first gate.\n"
          f"[Leader] Current local position: N={endpoint2_local_n:.2f}, E={endpoint2_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_2:.1f}°\n"
          f"Target endpoint: N={endpoint3_local_n:.2f}, E={endpoint3_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    end_heading_3 = await straight_maneuver(leader,
                            endpoint2_local_n, endpoint2_local_e, hover_pos['d'], end_heading_2,
                            endpoint3_local_n, endpoint3_local_e, hover_pos['d'],
                            0.5, 1
    )
    print("<-- Mission 3 completed -->")
    await asyncio.sleep(1)

    
    # Transform target coordinate into leader drone's coordinate
    endpoint4_local_n, endpoint4_local_e = coordinate_transformation(leader.id, WAYPOINT4[1], WAYPOINT4[0], 0)

    # Start mission 4: First curve path
    print("\n[Leader] Start mission 4, curve path to the second gate.\n"
          f"[Leader] Current local position: N={endpoint3_local_n:.2f}, E={endpoint3_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_3:.1f}°\n"
          f"Target endpoint: N={endpoint4_local_n:.2f}, E={endpoint4_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    real_local_n, real_local_e, real_local_d, end_heading_4 = await curve_maneuver(leader,
                            endpoint3_local_n, endpoint3_local_e, hover_pos['d'], end_heading_3,
                            endpoint4_local_n, endpoint4_local_e, hover_pos['d'],
                            0.5, 1, 1, 1.0
    )
    print("<-- Mission 4 completed -->")
    await asyncio.sleep(1)

    
    # Transform target coordinate into leader drone's coordinate
    endpoint5_local_n, endpoint5_local_e = coordinate_transformation(leader.id, WAYPOINT5[1], WAYPOINT5[0], 0)

    # Start mission 5: Fly pass second gate
    print("\n[Leader] Start mission 5, fly pass second gate.\n"
          f"[Leader] Current local position: N={endpoint4_local_n:.2f}, E={endpoint4_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_4:.1f}°\n"
          f"Target endpoint: N={endpoint5_local_n:.2f}, E={endpoint5_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    end_heading_5 = await straight_maneuver(leader,
                            endpoint4_local_n, endpoint4_local_e, hover_pos['d'], end_heading_4,
                            endpoint5_local_n, endpoint5_local_e, hover_pos['d'],
                            0.5, 1
    )
    print("<-- Mission 5 completed -->")
    await asyncio.sleep(1)

    
    # Transform target coordinate into leader drone's coordinate
    endpoint6_local_n, endpoint6_local_e = coordinate_transformation(leader.id, WAYPOINT6[1], WAYPOINT6[0], 0)

    # Start mission 6: Fly pass third gate
    print("\n[Leader] Start mission 6, fly pass third gate.\n"
          f"[Leader] Current local position: N={endpoint5_local_n:.2f}, E={endpoint5_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_5:.1f}°\n"
          f"Target endpoint: N={endpoint6_local_n:.2f}, E={endpoint6_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    real_local_n, real_local_e, real_local_d, end_heading_6 = await curve_maneuver(leader,
                            endpoint5_local_n, endpoint5_local_e, hover_pos['d'], end_heading_5,
                            endpoint6_local_n, endpoint6_local_e, hover_pos['d'],
                            0.5, 1, 1, 0.5
    )
    print("<-- Mission 6 completed -->")
    await asyncio.sleep(1)
    

    # Start mission 7: Move to the end gate
    print("\n[Leader] Start mission 7, Straight path to the end gate.\n"
          f"[Leader] Current local position: N={real_local_n:.2f}, E={real_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_6:.1f}°\n"
          f"Target endpoint: N={endpoint6_local_n:.2f}, E={endpoint6_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    end_heading_7 = await straight_maneuver(leader,
                            real_local_n, real_local_e, hover_pos['d'], end_heading_6,
                            endpoint6_local_n, endpoint6_local_e, hover_pos['d'],
                            0.5, 1
    )
    print("<-- Mission 7 completed -->")    

    
    # Start mission 8: Fly back to starting position
    print("\n[Leader] Start mission 8, fly back to starting position.\n"
          f"[Leader] Current local position: N={endpoint6_local_n:.2f}, E={endpoint6_local_e:.2f}, D={hover_pos['d']:.2f}\n"
          f"[Leader] Current heading: {end_heading_7:.1f}°\n"
          f"Target endpoint: N={gate_front_local_n:.2f}, E={gate_front_local_e:.2f}, D={hover_pos['d']:.2f}\n")
    end_heading_6 = await straight_maneuver(leader,
                            endpoint6_local_n, endpoint6_local_e, hover_pos['d'], end_heading_7,
                            gate_front_local_n, gate_front_local_e, hover_pos['d'],
                            0.5, 1
    )
    print("<-- Mission 8 completed -->")
    await asyncio.sleep(1)

    # Hold leader position after finish maneuvering
    while True:
        await leader.set_position(
            gate_front_local_n, gate_front_local_e, hover_pos['d'], target_heading
        )

async def follower_behavior(leader,follower,i):
    """
    i : To identify the drone as follower 1 or follower 2
    In the line formation, the leader will be followed by follower 1, then follower 2
    """

    print(f"[FOLLOWER {i}] Initializing...")
    await asyncio.sleep(2)

    follower_start_pos, _ = await follower.get_position()
    print(f"\n[FOLLOWER {i}] Initial local position N={follower_start_pos['n']:.2f}, "
          f"E={follower_start_pos['e']:.2f}, D={follower_start_pos['d']:.2f}")
    print(f"Follower {i} assigned to drone {follower.id}\n")

    iteration = 0
    phase = 1

    # Phase timing
    WAIT_TIME = 15  # Wait until leader done its maneuver

    while True:
        leader_pos, leader_heading = await leader.get_position()
        follower_pos, follower_heading = await follower.get_position()
        
        # Calculate target global position based on i
        if i==1:
            target_n, target_e, target_d = calculate_follower_position(
                leader.id, leader_pos, leader_heading,
                FOLLOWER1_DISTANCE, FORMATION_HEIGHT_OFFSET
            )
        elif i==2:
            target_n, target_e, target_d = calculate_follower_position(
                leader.id, leader_pos, leader_heading,
                FOLLOWER2_DISTANCE, FORMATION_HEIGHT_OFFSET
            )
        """
        if iteration == 20:
            print(f"[FOLLOWER {i}] Leader's position at 20 seconds: N={leader_pos['n']:.2f}, E={leader_pos['e']:.2f}, D={leader_pos['d']:.2f}")
            print(f"[FOLLOWER {i}] Leader's heading at 20 seconds: {leader_heading:.2f}°")
            print(f"[FOLLOWER {i}] Target position at 20 seconds: N={target_n:.2f}, E={target_e:.2f}, D={target_d:.2f}\n")
        """

        # Carry out coordinate transformation to get target position local to follower
        target_local_n, target_local_e = coordinate_transformation(follower.id, target_n, target_e, 0)

        # Calculate errors
        pos_error = math.sqrt(
            (follower_pos['n'] - target_local_n) ** 2 +
            (follower_pos['e'] - target_local_e) ** 2
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
                time_remaining = max(0, WAIT_TIME - iteration * UPDATE_RATE)
                print(f"[FOLLOWER {i} PHASE 1] Hovering while leader turns... ({time_remaining:.1f}s remaining)")

            # Move to next phase after leader finishes turning
            if (iteration * UPDATE_RATE) >= WAIT_TIME:
                print(f"\n[FOLLOWER {i} PHASE 1] Complete, Leader turned, starting positioning\n")
                phase = 2

        # PHASE 2: POSITION - Move to target while maintaining start altitude (collision avoidance)
        elif phase == 2:
            """
            Mechanism to slow down follower's speed in phase 2 (still under construction)
            new_local_n = follower_start_pos['n'] + 0.2 * (target_local_n - follower_pos['n'])
            new_local_e = follower_start_pos['e'] + 0.2 * (target_local_e - follower_pos['e'])
            """
            await follower.set_position(
                target_local_n, target_local_e, follower_start_pos['d'],
                follower_heading  # Keep current heading
            )
            # Reminder: .set_position() use NED frame, and target_d here is a -ve value

            if iteration % 10 == 0:
                print(f"[FOLLOWER {i} PHASE 2] Current leader position: N={leader_pos['n']:.2f}, E={leader_pos['e']:.2f}, D={leader_pos['d']:.2f}")
                print(f"[FOLLOWER {i} PHASE 2] Current leader heading: {leader_heading:.2f}°")
                print(f"[FOLLOWER {i} PHASE 2] Current target position: N={target_local_n:.2f}, E={target_local_e:.2f}, D={target_d:.2f}")
                print(f"[FOLLOWER {i} PHASE 2] Moving to leader's back: "
                      f"Error={pos_error:.2f}m, height_error={height_error:.2f}m\n")

            # Check if in position (horizontally)
            if pos_error < POSITION_TOLERANCE:
                print(f"\n[FOLLOWER {i} PHASE 2] Complete, in position behind leader (at safe altitude)\n")
                phase = 3

        # PHASE 3: ALTITUDE - Rise to formation altitude
        elif phase == 3:
            await follower.set_position(
                target_local_n, target_local_e, target_d,  # Rise to formation altitude
                follower_heading
            )

            if iteration % 10 == 0:
                print(f"[FOLLOWER {i} PHASE 3] Rising to formation altitude: "
                      f"Height_error={height_error:.2f}m")

            # Check if at correct altitude
            if height_error < HEIGHT_TOLERANCE:  # Within 30cm
                print(f"\n[FOLLOWER {i} PHASE 3] Complete, at formation altitude\n")
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

            await follower.set_position(target_local_n, target_local_e, target_d, new_heading)

            if iteration % 10 == 0:
                print(f"[FOLLOWER {i} PHASE 4] Aligning heading: "
                      f"Error={heading_error:.1f}°, Current={follower_heading:.1f}°, Target={leader_heading:.1f}°")

            # Check if aligned
            if heading_error < HEADING_TOLERANCE:
                print(f"\n[FOLLOWER {i} PHASE 4] Complete,heading aligned!")
                print(f"<---- STARTING FORMATION LOCKED ---->\n")
                phase = 5

        # Mission phase: TRACK - Maintain formation
        else:
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

            await follower.set_position(target_local_n, target_local_e, target_d, new_heading)

            if iteration % 30 == 0:  # Print every 3 seconds
                print(f"[FOLLOWER {i} PHASE 5] Maintain formation\n"
                      f"Current target position: N={target_local_n:.2f}, E={target_local_e:.2f}, D={target_d:.2f}\n"
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
            leader_behavior(role_alloc["Leader"], TARGET_HEADING),
            follower_behavior(role_alloc["Leader"], role_alloc["Follower1"], 1),
            follower_behavior(role_alloc["Leader"], role_alloc["Follower2"], 2)
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
