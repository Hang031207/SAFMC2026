import asyncio
from mavsdk import System

USE_GZ = True
if USE_GZ:
    DRONE_PORTS = [14581, 14582] 
else: 
    DRONE_PORTS = [14561, 14562]

async def control_drone(name, port):
    print(f"\n=== {name} ===")
    drone = System()

    # Connect
    await drone.connect(system_address=f"udp://127.0.0.1:{port}")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✅ {name} connected")
            break
    # Skip health checks for Gazebo
    await drone.action.set_takeoff_altitude(2.5)
    
    # Arm → Takeoff → Hover → Land
    print("Arming...")
    await drone.action.arm()
    
    print("Taking off...")
    await drone.action.takeoff()
    
    print(f"Hovering for a few seconds...")
    await asyncio.sleep(8)
    
    print("Landing...")
    await drone.action.land()
    
    print(f"✅ {name} done")

async def main():
    await control_drone("Drone 1", DRONE_PORTS[0])
    await asyncio.sleep(2)
    await control_drone("Drone 2", DRONE_PORTS[1])

if __name__ == "__main__":
    asyncio.run(main())
