# plc_controller.py
from asyncua import Client
import asyncio
import logging

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

PLC_URL = "opc.tcp://localhost:4840"
NS = "ns=4;s=|var|CODESYS Control Win V3 x64.Application.PLC_PRG."

NODES = {
    "Part_Detected": NS + "Part_Detected",
    "Pick_Request":  NS + "Pick_Request",
    "Pick_Done":     NS + "Pick_Done",
    "Motor":         NS + "Motor",
}

class PLCController:
    def __init__(self):
        self.client = None
        self.nodes = {}

    async def connect(self):
        self.client = Client(PLC_URL)
        self.client.set_user("")
        self.client.set_password("")
        await self.client.connect()
        for name, nodeid in NODES.items():
            self.nodes[name] = self.client.get_node(nodeid)
        log.info("✅ Connected to PLC")

    async def read(self, name):
        return await self.nodes[name].read_value()

    async def write(self, name, value):
        await self.nodes[name].write_value(value)

    async def run_pick_cycle(self, robot_callback):
        """Main coordination loop"""
        log.info("🚀 Controller running...")
        cycle_count = 0

        while True:
            part = await self.read("Part_Detected")

            if part:
                cycle_count += 1
                log.info(f"📦 Part detected! Cycle #{cycle_count}")

                # 1. Signal PLC to stop conveyor + request pick
                await self.write("Pick_Request", True)

                # 2. Trigger robot simulation
                await robot_callback(cycle_count)

                # 3. Wait for PLC to confirm done
                timeout = 0
                while not await self.read("Pick_Done") and timeout < 50:
                    await asyncio.sleep(0.1)
                    timeout += 1

                if timeout >= 50:
                    log.warning("⚠️ Pick_Done timeout!")

                # 4. Reset
                await self.write("Pick_Request", False)
                log.info(f"✅ Cycle #{cycle_count} complete")

            await asyncio.sleep(0.1)

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()