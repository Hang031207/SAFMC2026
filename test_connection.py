import asyncio
from mavsdk import System

async def run():
    drone = System()
    print("Waiting for connection...")
    await drone.connect(system_address="udp://:14560")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

asyncio.run(run())
