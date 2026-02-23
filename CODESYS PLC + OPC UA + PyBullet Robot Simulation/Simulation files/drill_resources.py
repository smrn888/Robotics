from asyncua import Client
import asyncio

async def drill_resources():
    client = Client("opc.tcp://localhost:4840")
    client.set_user("")
    client.set_password("")
    await client.connect()
    print("✅ Connected\n")

    async def browse_recursive(node, depth=0, max_depth=10):
        if depth > max_depth:
            return
        try:
            children = await node.get_children()
            for child in children:
                try:
                    name = await child.read_display_name()
                    nodeid = child.nodeid
                    name_str = name.Text if hasattr(name, 'Text') else str(name)
                    indent = '  ' * depth
                    
                    # Try to read value
                    value_str = ""
                    try:
                        val = await child.read_value()
                        value_str = f" = {val}"
                    except:
                        pass
                    
                    print(f"{indent}[{nodeid}] {name_str}{value_str}")
                    await browse_recursive(child, depth + 1, max_depth)
                except Exception as e:
                    pass
        except Exception as e:
            pass

    # Start directly from the Resources node
    resources_node = client.get_node("ns=4;i=1001")
    print("📂 Drilling into Resources...\n")
    await browse_recursive(resources_node)

    await client.disconnect()
    print("\n🔌 Done")

asyncio.run(drill_resources())