from asyncua import Client
import asyncio

async def browse_structure():
    client = Client("opc.tcp://localhost:4840")
    client.set_user("")
    client.set_password("")
    
    await client.connect()
    print("✅ متصل شدیم\n")
    
    # === تلاش برای یافتن CODESYS Device ===
    try:
        objects = client.get_objects_node()
        
        # === Browse Objects children ===
        print("🔍 البحث في Objects:")
        obj_children = await objects.get_children()
        
        for child in obj_children:
            name = await child.read_display_name()
            node_id = child.nodeid
            print(f"\n  📌 {name}")
            print(f"     NodeID: {node_id}")
            
            # === Browse deeper ===
            try:
                grandchildren = await child.get_children()
                print(f"     فرزندان ({len(grandchildren)}):")
                for grandchild in grandchildren[:5]:
                    gname = await grandchild.read_display_name()
                    gnodeid = grandchild.nodeid
                    print(f"       - {gname}")
                    print(f"         {gnodeid}")
                    
                    # === Try to read value ===
                    try:
                        val = await grandchild.read_value()
                        print(f"         ✅ مقدار: {val}")
                    except Exception as e:
                        print(f"         ❌ نمی‌توان خواند: {str(e)[:50]}")
            except:
                pass
    
    except Exception as e:
        print(f"❌ خطا: {e}")
    
    # === Try alternative: Get all nodes with "GVL" in name ===
    print("\n\n🔍 جستجو برای GVL variables با روش جایگزین:\n")
    
    try:
        # Check if we can access via path
        codesys_node = client.get_node("ns=2;s=CODESYS")
        print(f"✅ CODESYS node found: {codesys_node}")
        
        children = await codesys_node.get_children()
        print(f"   فرزندان: {len(children)}")
        for child in children[:10]:
            name = await child.read_display_name()
            print(f"   - {name}")
            
    except Exception as e:
        print(f"⚠️  CODESYS path خطا: {e}")
    
    print("\n✅ تمام شد")
    await client.disconnect()

if __name__ == "__main__":
    asyncio.run(browse_structure())
