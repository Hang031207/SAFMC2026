#!/usr/bin/env python3

import asyncio
from mavsdk import System

async def sniff_port(port: int):
    """Connect and report what system IDs are visible."""
    drone = System()
    await drone.connect(system_address=f"udp://127.0.0.1:{port}")
    
    print(f"\n=== Port {port} ===")
    print(f"Waiting for connection...")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✓ Connected!")
            break
    
    # Wait a bit for system discovery
    await asyncio.sleep(2)
    
    # Try to get system info
    try:
        info = await drone.info.get_identification()
        print(f"Hardware UID: {info.hardware_uid}")
    except Exception as e:
        print(f"Could not get info: {e}")
    
    # Get position to see which drone this is
    try:
        async for pos in drone.telemetry.position_velocity_ned():
            print(f"Position: N={pos.position.north_m:.2f}, E={pos.position.east_m:.2f}, D={pos.position.down_m:.2f}")
            break
    except Exception as e:
        print(f"Could not get position: {e}")
    
    print(f"=== End Port {port} ===\n")

async def main():
    print("MAVLink Port Sniffer - Checking what's on each port")
    print("="*60)
    
    # Check both ports sequentially
    await sniff_port(14581)
    await sniff_port(14582)
    
    print("="*60)
    print("Done! Both ports checked.")

if __name__ == "__main__":
    asyncio.run(main())
