from asyncua import Client
import asyncio

async def full_dump():
    client = Client("opc.tcp://localhost:4840")
    client.set_user("")
    client.set_password("")
    await client.connect()
    print("✅ Connected\n")

    async def browse_recursive(node, depth=0, max_depth=6):
        if depth > max_depth:
            return
        try:
            children = await node.get_children()
            for child in children:
                try:
                    name = await child.read_display_name()
                    nodeid = child.nodeid
                    name_str = name.Text if hasattr(name, 'Text') else str(name)
                    print(f"{'  ' * depth}[{nodeid}] {name_str}")
                    await browse_recursive(child, depth + 1, max_depth)
                except Exception as e:
                    print(f"{'  ' * depth}⚠️ child error: {e}")
        except Exception as e:
            print(f"{'  ' * depth}⚠️ browse error: {e}")

    objects = client.get_objects_node()
    print("📂 Full OPC UA Tree:\n")
    await browse_recursive(objects)

    await client.disconnect()
    print("\n🔌 Done")

asyncio.run(full_dump())