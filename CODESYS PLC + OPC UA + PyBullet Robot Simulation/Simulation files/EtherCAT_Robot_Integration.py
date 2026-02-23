from asyncua import Client
import asyncio

PLC_URL = "opc.tcp://localhost:4840"

async def main():
    async with Client(PLC_URL) as client:
        
        part_node = client.get_node("ns=4;s=GVL.Part_Detected")
        pick_req  = client.get_node("ns=4;s=GVL.Pick_Request")
        pick_done = client.get_node("ns=4;s=GVL.Pick_Done")

        while True:
            part = await part_node.read_value()

            if part:
                print("Part detected → stopping conveyor")
                await pick_req.write_value(True)

                while not await pick_done.read_value():
                    await asyncio.sleep(0.1)

                await pick_req.write_value(False)

            await asyncio.sleep(0.1)

asyncio.run(main())