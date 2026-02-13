#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

HOVER_ALT_OFFSET = -0.5  # 0.5m up
OFFBOARD_RATE_HZ = 20.0

async def hover_loop(drone: System, hover_wp: PositionNedYaw):
    dt = 1.0 / OFFBOARD_RATE_HZ
    print('-- Hovering 15 seconds...')
    for _ in range(int(15 * OFFBOARD_RATE_HZ)):
        await drone.offboard.set_position_ned(hover_wp)
        await asyncio.sleep(dt)

async def run():
    drone = System()
    await drone.connect(system_address='udp://127.0.0.1:14581')
    
    print('Waiting for drone...')
    async for state in drone.core.connection_state():
        if state.is_connected: print('-- Connected!'); break
    
    print('Waiting for UWB...')
    async for health in drone.telemetry.health():
        if health.is_local_position_ok: print('-- UWB OK!'); break
        await asyncio.sleep(0.5)
    
    print('-- Arming')
    await drone.action.arm()
    
    # FIXED: Use position_velocity_ned() → has local NED data
    print('-- Reading UWB position...')
    async for pos_vel_ned in drone.telemetry.position_velocity_ned():
        print(f'Current NED: N={pos_vel_ned.position.north_m:.2f}m E={pos_vel_ned.position.east_m:.2f}m D={pos_vel_ned.position.down_m:.2f}m')
        hover_wp = PositionNedYaw(
            north_m=pos_vel_ned.position.north_m,
            east_m=pos_vel_ned.position.east_m,
            down_m=pos_vel_ned.position.down_m + HOVER_ALT_OFFSET,
            yaw_deg=0.0
        )
        break
    
    print('-- Setting hover setpoint')
    await drone.offboard.set_position_ned(hover_wp)
    
    print('-- Starting OFFBOARD')
    await drone.offboard.start()
    await hover_loop(drone, hover_wp)
    
    print('-- Stopping OFFBOARD')
    await drone.offboard.stop()
    
    print('-- Landing')
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
