import asyncio
from mavsdk import System

async def run():
    drone = System()
    print("Connecting udp://:14560 (local bind)...")
    
    # Bind to all interfaces, connect to RasPi
    await drone.connect(system_address="udp://:14560")
    
    print("Waiting heartbeats (20s)...")
    for i in range(20):
        state = await drone.core.connection_state()
        print(f"[{i}s] {state}")
        if state.is_connected:
            print("✅ CONNECTED!")
            break
        await asyncio.sleep(1)
    else:
        print("Timeout - check mavsdk_server?")

asyncio.run(run())
