import asyncio
from mavsdk import System

async def main():
    d1 = System(port=50051)
    d2 = System(port=50052)

    await d1.connect(system_address="udp://127.0.0.1:14581")
    await d2.connect(system_address="udp://127.0.0.1:14582")

    print("Waiting for drone 1...")
    async for s in d1.core.connection_state():
        if s.is_connected:
            print("Drone 1 connected")
            break

    print("Waiting for drone 2...")
    async for s in d2.core.connection_state():
        if s.is_connected:
            print("Drone 2 connected")
            break

asyncio.run(main())
