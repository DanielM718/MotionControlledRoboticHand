import asyncio
from bleak import BleakClient


async def main(address):
    async with BleakClient(address) as client:
    	for service in client.services:
    		print(service)
    		for char in service.chracteristics:
    			print(char)
    	
    	
    	
asyncio.run(main("D8:BC:38:E5:2A:66"))
