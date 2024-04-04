import asyncio
from bleak import BleakClient

async def main():
    async with BleakClient("D8:BC:38:E5:2A:66") as client:
    	print(client.services)
    	
    	
    	
asyncio.run(main())
