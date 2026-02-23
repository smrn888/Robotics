from asyncua import Client
from asyncua.ua import AttributeIds
import asyncio

async def check_attributes():
    client = Client("opc.tcp://localhost:4840")
    client.set_user("")
    client.set_password("")
    
    await client.connect()
    print("✅ متصل شدیم\n")
    
    # === بررسی یک متغیر محدود ===
    print("🔍 بررسی attributes GVL متغیر:\n")
    
    try:
        node = client.get_node("ns=2;s=GVL")
        
        print(f"NodeID: {node.nodeid}")
        
        # === خواندن تمام attributes ===
        attributes_to_check = [
            (AttributeIds.NodeId, "NodeId"),
            (AttributeIds.NodeClass, "NodeClass"),
            (AttributeIds.BrowseName, "BrowseName"),
            (AttributeIds.DisplayName, "DisplayName"),
            (AttributeIds.Description, "Description"),
            (AttributeIds.Value, "Value"),
            (AttributeIds.DataType, "DataType"),
            (AttributeIds.ValueRank, "ValueRank"),
            (AttributeIds.IsAbstract, "IsAbstract"),
            (AttributeIds.Historizing, "Historizing"),
            (AttributeIds.AccessLevel, "AccessLevel"),
            (AttributeIds.UserAccessLevel, "UserAccessLevel"),
        ]
        
        for attr_id, attr_name in attributes_to_check:
            try:
                result = await node.read_attribute(attr_id)
                print(f"  {attr_name}: {result}")
            except Exception as e:
                print(f"  {attr_name}: ❌ {str(e)[:60]}")
        
        # === Browse children ===
        print(f"\n\n📂 GVL Children:")
        try:
            children = await node.get_children()
            print(f"   تعداد: {len(children)}")
            for child in children[:5]:
                cname = await child.read_display_name()
                cnodeid = child.nodeid
                print(f"   - {cname} ({cnodeid})")
                
                # Try to read value
                try:
                    val = await child.read_value()
                    print(f"     مقدار: {val}")
                except Exception as e:
                    print(f"     ❌ نمی‌توان خواند: {str(e)[:40]}")
        except Exception as e:
            print(f"   ❌ خطا: {e}")
    
    except Exception as e:
        print(f"❌ GVL node خطا: {e}")
    
    print("\n✅ بررسی تمام شد")
    await client.disconnect()

if __name__ == "__main__":
    asyncio.run(check_attributes())
