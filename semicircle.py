#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

HOVER_ALT_OFFSET = -1.4  # Hover height
LANDING_RATE = 0.1       # m/s descent speed
OFFBOARD_RATE_HZ = 20.0
RADIUS = 2.0             # Semicircle radius
NUM_POINTS = 80          # Smooth curve

async def hover_loop(drone: System, hover_wp: PositionNedYaw, duration_s: float):
    dt = 1.0 / OFFBOARD_RATE_HZ
    print(f'-- Hovering {duration_s}s...')
    for _ in range(int(duration_s * OFFBOARD_RATE_HZ)):
        await drone.offboard.set_position_ned(hover_wp)
        await asyncio.sleep(dt)

async def smooth_landing(drone: System, landing_wp: PositionNedYaw, target_down: float):
    """Gradually descend to target altitude in offboard mode."""
    dt = 1.0 / OFFBOARD_RATE_HZ
    current_down = landing_wp.down_m
    
    print(f'-- Smooth landing from D={current_down:.2f}m to D={target_down:.2f}m')
    
    while current_down < target_down:  # NED: less negative = down
        current_down += LANDING_RATE * dt  # Descend slowly
        if current_down > target_down:
            current_down = target_down
        
        land_wp = PositionNedYaw(
            north_m=landing_wp.north_m,
            east_m=landing_wp.east_m,
            down_m=current_down,
            yaw_deg=0.0
        )
        await drone.offboard.set_position_ned(land_wp)
        await asyncio.sleep(dt)

async def run():
    drone = System()
    await drone.connect(system_address='udpin://0.0.0.0:14561')
    
    print('Waiting for drone...')
    async for state in drone.core.connection_state():
        if state.is_connected: print('-- Connected!'); break
    
    print('Waiting for UWB...')
    async for health in drone.telemetry.health():
        if health.is_local_position_ok: print('-- UWB OK!'); break
        await asyncio.sleep(0.5)
    
    print('-- Arming')
    await drone.action.arm()
    
    print('-- Reading UWB position...')
    async for pos_vel_ned in drone.telemetry.position_velocity_ned():
        print(f'Current NED: N={pos_vel_ned.position.north_m:.2f}m E={pos_vel_ned.position.east_m:.2f}m D={pos_vel_ned.position.down_m:.2f}m')
        takeoff_pos = pos_vel_ned.position  # Save takeoff XY
        hover_wp = PositionNedYaw(
            north_m=takeoff_pos.north_m,
            east_m=takeoff_pos.east_m,
            down_m=takeoff_pos.down_m + HOVER_ALT_OFFSET,  # Hover up
            yaw_deg=0.0
        )
        break
    
    print('-- Starting OFFBOARD')
    await drone.offboard.set_position_ned(hover_wp)
    await drone.offboard.start()
    
    # 1. YOUR PROVEN HOVER (10.0s stable vertical takeoff)
    await hover_loop(drone, hover_wp, 10.0)
    
    # 2. NEW: SEMICIRCLE MANEUVER (from stable hover position)
    print('-- Generating semicircle waypoints...')
    semicircle_points = []
    for i in range(NUM_POINTS + 1):
        theta = math.pi * i / NUM_POINTS  # 0° to 180°
        north = takeoff_pos.north_m + RADIUS * math.cos(theta)
        east = takeoff_pos.east_m + RADIUS * math.sin(theta)
        wp = PositionNedYaw(north, east, hover_wp.down_m, 0.0)  # SAME ALTITUDE
        semicircle_points.append(wp)
    
    print('-- Flying smooth semicircle!')
    for wp in semicircle_points:
        await drone.offboard.set_position_ned(wp)
        await asyncio.sleep(0.1)  # Fast transit = smooth curve
    
    # 3. YOUR PROVEN HOVER RETURN (3s)
    await hover_loop(drone, hover_wp, 3.0)
    
    # 4. YOUR PROVEN SMOOTH LANDING
    landing_wp = PositionNedYaw(
        north_m=takeoff_pos.north_m,
        east_m=takeoff_pos.east_m,
        down_m=takeoff_pos.down_m,  # Back to takeoff altitude
        yaw_deg=0.0
    )
    await smooth_landing(drone, landing_wp, takeoff_pos.down_m)
    
    # Wait on ground
    print('-- Touchdown! Waiting 3s...')
    await asyncio.sleep(3.0)
    
    # Safe shutdown (YOUR PROVEN METHOD)
    print('-- Stopping OFFBOARD')
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass  # Normal during landing
    
    print('-- Final land command')
    await drone.action.land()

if __name__ == '__main__':
    asyncio.run(run())
