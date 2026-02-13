import asyncio
from mavsdk import System

async def run():
    drone = System()
    print("Connecting to udp://192.168.0.99:14560...")
    await drone.connect(system_address="udp://192.168.0.99:14560")
    
    print("Waiting 10s for heartbeats...")
    timeout = 10
    start = asyncio.get_event_loop().time()
    
    async for state in drone.core.connection_state():
        elapsed = asyncio.get_event_loop().time() - start
        print(f"[{elapsed:.1f}s] State: {state}")
        
        if state.is_connected:
            print("Connected!")
            return
        if elapsed > timeout:
            print("Timeout")
            return
        await asyncio.sleep(0.5)

asyncio.run(run())
