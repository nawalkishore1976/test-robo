import asyncio
from bleak import BleakScanner, BleakClient

async def wait_for_robo():
    print("Waiting for robo device...")
    print("Turn on your Arduino now...")
    
    while True:
        devices = await BleakScanner.discover(timeout=3.0)
        
        for device in devices:
            if device.name and 'robo' in device.name.lower():
                print(f"Found: {device.name}")
                return device
        
        print(".", end="", flush=True)  # Show we're still looking
        await asyncio.sleep(1)

async def monitor_robo():
    device = await wait_for_robo()
    
    async with BleakClient(device.address) as client:
        print(f"Connected to {device.name}!")
        
        # Find first notify characteristic
        services = await client.get_services()
        for service in services:
            for char in service.characteristics:
                if 'notify' in char.properties:
                    print(f"Monitoring {char.uuid}...")
                    
                    def handler(sender, data):
                        try:
                            print(f"Robo: {data.decode('utf-8').strip()}")
                        except:
                            print(f"Data: {data.hex()}")
                    
                    await client.start_notify(char.uuid, handler)
                    
                    # Wait forever (until Ctrl+C)
                    try:
                        while True:
                            await asyncio.sleep(1)
                    except KeyboardInterrupt:
                        print("\nStopping...")
                        return

if __name__ == "__main__":
    try:
        asyncio.run(monitor_robo())
    except KeyboardInterrupt:
        print("Bye!")