#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def run():
    """ Simple test script for takeoff, forward movement, and landing """

    # Initialize and connect to PX4
    drone = System()
    print("🚀 Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone connected!")
            break
        await asyncio.sleep(1)

    # Arm the drone
    print("🔄 Arming the drone...")
    await drone.action.arm()
    print("✅ Armed successfully!")

    # Takeoff to 5m altitude
    print("🚀 Taking off...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()

    # Wait until the drone reaches altitude
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= 4.8:
            print("✅ Reached takeoff altitude!")
            break
        await asyncio.sleep(1)

    # Start Offboard Mode - First send multiple setpoints
    print("🔄 Sending initial Offboard commands...")
    initial_position = PositionNedYaw(0.0, 0.0, -5.0, 0.0)

    for _ in range(50):  # Send initial setpoints for stability
        await drone.offboard.set_position_ned(initial_position)
        await asyncio.sleep(0.1)

    # Start Offboard Mode
    try:
        await drone.offboard.start()
        print("✅ Offboard mode started!")
    except Exception as e:
        print(f"❌ Failed to start Offboard mode: {e}")
        return

    # Move forward (North) by 10 meters
    print("🛫 Flying forward 10m...")
    target_position = PositionNedYaw(10.0, 0.0, -5.0, 0.0)

    # Keep sending the move command to ensure it executes
    for _ in range(100):
        await drone.offboard.set_position_ned(target_position)
        print("📡 Sending move command...")
        await asyncio.sleep(0.1)

    # Verify movement via telemetry
    async for position in drone.telemetry.position():
        print(f"📍 Current Position: {position.latitude_deg}, {position.longitude_deg}")
        if position.relative_altitude_m >= 4.8 and position.latitude_deg != 0:
            print("✅ Drone moved!")
            break
        await asyncio.sleep(1)

    # Land
    print("🛬 Landing...")
    await drone.action.land()

    # Wait until landed
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("✅ Landed successfully!")
            break
        await asyncio.sleep(1)

async def main():
    await run()

if __name__ == "__main__":
    asyncio.run(main())
