#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

#If USE_GZ below is True, means we are running simulation, otherwise we are connecting to the real drones
USE_GZ = True
if USE_GZ:
    DRONE_PORTS = [14581, 14582]
else:
    DRONE_PORTS = [14561, 14562]

HOVER_ALT = -2.5
OFFBOARD_RATE_HZ = 20.0

async def hover_loop(drone: System, north: float, east: float, down: float):
    """Hover at relative position."""
    dt = 1.0 / OFFBOARD_RATE_HZ
    hover_wp = PositionNedYaw(north, east, down, 0.0)
    
    print(f"[{drone}] Starting hover loop...")
    try:
        while True:
            await drone.offboard.set_position_ned(hover_wp)
            await asyncio.sleep(dt)
    except asyncio.CancelledError:
        print(f"[{drone}] Hover loop cancelled")

async def connect_drone(port,i):
    """Connect to single drone."""
    drone = System(port=50051+i)
    await drone.connect(system_address=f"udp://127.0.0.1:{port}")
    
    print(f"[Drone {port}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {port}] Connected!")
            break
    
    # Health check (UWB local position)
    async for health in drone.telemetry.health():
        print(f"[Drone {port}] Health: local_pos_ok={health.is_local_position_ok}")
        if health.is_local_position_ok:
            break
    
    return drone

async def hover_drone(drone: System, drone_id: str):
    """Arm, takeoff, hover for one drone."""
    print(f"[{drone_id}] Arming...")
    await drone.action.arm()
    
    # Get current position for relative hover
    print(f"[{drone_id}] Getting current position...")
    async for pos_vel_ned in drone.telemetry.position_velocity_ned():
        print(f"[{drone_id}] Current: N={pos_vel_ned.position.north_m:.1f}, E={pos_vel_ned.position.east_m:.1f}, D={pos_vel_ned.position.down_m:.1f}")
        hover_wp = PositionNedYaw(
            north_m=pos_vel_ned.position.north_m,
            east_m=pos_vel_ned.position.east_m,
            down_m=pos_vel_ned.position.down_m + HOVER_ALT,  # 0.5m above current
            yaw_deg=0.0
        )
        break
    
    # Set initial setpoint
    print("-- Setting initial setpoints")
    await drone.offboard.set_position_ned(hover_wp)
    
    print(f"[{drone_id}] Starting offboard...")
    await drone.offboard.start()
    
    return hover_wp

async def global_pos_test(drone: System, drone_id: str):
    async for i in drone.telemetry.position():
        print(f"[{drone_id} Global] lat={i.latitude_deg:.6f}, lon={i.longitude_deg:.6f}, alt={i.relative_altitude_m:.1f}m")
        break

async def run():
    """Connect both drones and hover simultaneously."""
    print("=== Multi-Drone Hover Test ===")
    
    # Connect both drones concurrently
    drones = await asyncio.gather(
        connect_drone(DRONE_PORTS[0],0),
        connect_drone(DRONE_PORTS[1],1)
    )
    drone1, drone2 = drones
    
    # Hover both (concurrent)
    hover_wp1 = await hover_drone(drone1, "Drone1")
    hover_wp2 = await hover_drone(drone2, "Drone2")
    
    if hover_wp1 is None or hover_wp2 is None:
        print("One drone failed - aborting")
        return
    
    # Start hover loops
    hover_task1 = asyncio.create_task(
        hover_loop(drone1, hover_wp1.north_m, hover_wp1.east_m, hover_wp1.down_m)
    )
    hover_task2 = asyncio.create_task(
        hover_loop(drone2, hover_wp2.north_m, hover_wp2.east_m, hover_wp2.down_m)
    )
    
    # Hover 10 seconds (Change accordingly)
    try:
        await asyncio.sleep(10.0)
        await global_pos_test(drone1, "Drone1")
        await global_pos_test(drone2, "Drone2")
    finally:
        hover_task1.cancel()
        hover_task2.cancel()
        await asyncio.gather(hover_task1, hover_task2, return_exceptions=True)
    
    # Land both
    print("-- Landing both drones --")
    await asyncio.gather(
        drone1.offboard.stop(),
        drone2.offboard.stop(),
        return_exceptions=True
    )
    await asyncio.gather(
        drone1.action.land(),
        drone2.action.land()
    )
    
    print("=== Multi-drone hover complete! ===")

if __name__ == "__main__":
    asyncio.run(run())

