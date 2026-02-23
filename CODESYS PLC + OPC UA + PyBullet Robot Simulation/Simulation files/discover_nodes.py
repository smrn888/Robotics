from asyncua import Client
import asyncio

async def discover():
    client = Client("opc.tcp://localhost:4840")
    client.set_user("")
    client.set_password("")
    
    await client.connect()
    print("✅ متصل شدیم به CODESYS\n")
    
    # === بررسی گره اصلی ===
    root = client.get_root_node()
    print("📍 گره‌های اصلی:")
    
    children = await root.get_children()
    for child in children:
        name = await child.read_display_name()
        print(f"  - {name}")
    
    # === تلاش برای یافتن Objects ===
    try:
        objects = client.get_objects_node()
        print(f"\n📍 گره Objects: {await objects.read_display_name()}")
        
        obj_children = await objects.get_children()
        print(f"\n📍 فرزندان Objects ({len(obj_children)}):")
        for child in obj_children[:15]:
            name = await child.read_display_name()
            print(f"  - {name}")
    except Exception as e:
        print(f"❌ خطا: {e}")
    
    # === جستجو در Namespaces ===
    print("\n\n🔍 جستجو برای متغیرها در Namespaces مختلف:\n")
    
    found_vars = []
    
    for ns in range(0, 8):
        for var_name in ["GVL", "Part_Detected", "Pick_Request", "Pick_Done"]:
            try:
                node = client.get_node(f"ns={ns};s={var_name}")
                try:
                    val = await node.read_value()
                    print(f"  ✅ ns={ns};s={var_name} وجود دارد (مقدار: {val})")
                    found_vars.append(f"ns={ns};s={var_name}")
                except:
                    print(f"  ⚠️  ns={ns};s={var_name} وجود دارد اما نمی‌تونم مقدار بخونم")
                    found_vars.append(f"ns={ns};s={var_name}")
            except:
                pass
    
    if not found_vars:
        print("  ❌ هیچ متغیری پیدا نشد!\n")
        print("💡 راه‌های حل:")
        print("  1. متغیرها رو توی CODESYS GVL اضافه کن")
        print("  2. اطمینان بخش که پروژه CODESYS در حالت RUN است (F5)")
        print("  3. بررسی کن OPC UA فعال است: Tools → Options → OPC UA")
    else:
        print(f"\n\n✅ {len(found_vars)} متغیر پیدا شد!")
        print("\n📝 استفاده از اینها در latency_analyzer.py:")
        for var in found_vars[:3]:
            print(f"  {var}")
    
    await client.disconnect()
    print("\n✅ جستجو تمام شد")

if __name__ == "__main__":
    asyncio.run(discover())