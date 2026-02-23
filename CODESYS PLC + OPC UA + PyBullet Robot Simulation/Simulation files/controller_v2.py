async def connect(self):
    self.client = Client(self.plc_url)
    
    # ✅ FIX: Sessionless connection
    await self.client.connect()
    print("✅ Connected to CODESYS OPC UA Server")
    
    # === Register OPC UA nodes ===
    try:
        self.node_part_detected = self.client.get_node("ns=4;s=GVL.Part_Detected")
        self.node_pick_request = self.client.get_node("ns=4;s=GVL.Pick_Request")
        self.node_pick_done = self.client.get_node("ns=4;s=GVL.Pick_Done")
        print("✅ All nodes registered")
    except Exception as e:
        print(f"❌ Node error: {e}")
        raise

    await analyzer.client.disconnect()