from asyncua import Client
import asyncio

async def browse_all():
    client = Client("opc.tcp://localhost:4840")
    client.set_user("")
    client.set_password("")
    await client.connect()
    print("✅ Connected\n")

    # Check namespaces first
    ns_array = await client.get_namespace_array()
    print("📋 Namespaces:")
    for i, ns in enumerate(ns_array):
        print(f"   [{i}] {ns}")

    # Browse from root to find your variables
    print("\n🔍 Searching for Pick_Request and Pick_Done...\n")

    async def browse_recursive(node, depth=0, max_depth=5):
        if depth > max_depth:
            return
        try:
            children = await node.get_children()
            for child in children:
                try:
                    name = await child.read_display_name()
                    nodeid = child.nodeid
                    name_str = name.Text if hasattr(name, 'Text') else str(name)
                    
                    if any(kw in name_str for kw in ["Pick_Request", "Pick_Done", "GVL", "Part_Detected"]):
                        print(f"{'  '*depth}✅ FOUND: {name_str}")
                        print(f"{'  '*depth}   NodeID: {nodeid}")
                        try:
                            val = await child.read_value()
                            print(f"{'  '*depth}   Value: {val}")
                        except:
                            pass
                    
                    # Keep browsing into GVL/Application containers
                    if any(kw in name_str for kw in ["GVL", "Application", "PLC", "Device"]):
                        await browse_recursive(child, depth+1, max_depth)
                except:
                    pass
        except:
            pass

    root = client.get_root_node()
    objects = client.get_objects_node()
    await browse_recursive(objects)

    await client.disconnect()
    print("\n🔌 Done")

asyncio.run(browse_all())