import asyncio

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

CHARACTERISTIC_UUID = "c122a3d6-8d8a-4b89-b21f-4d91a260aeee"
CHARACTERISTIC_UUID2 = "bcd9f8bf-b4da-4301-9c38-20fe24e9efe7"

def callback(sender: BleakGATTCharacteristic, data: bytearray):
    print(f"{sender}: {data}")
    
async def main(address, UUID1, UUID2):
    async with BleakClient(address) as client:
    	await client.start_notify(UUID1, callback)
    	await asyncio.sleep(5.0)
    	await client.start_notify(UUID2, callback)
    	await asyncio.sleep(5.0)  	
    	
    	
asyncio.run(main("D8:BC:38:E5:2A:66", CHARACTERISTIC_UUID, CHARACTERISTIC_UUID2))
