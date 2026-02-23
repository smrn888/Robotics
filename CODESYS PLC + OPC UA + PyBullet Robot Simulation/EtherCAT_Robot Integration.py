# PLC + EtherCAT + Robot Integration

from opcua import Client

# بدون هیچ authentication
client = Client("opc.tcp://localhost:4840/")
client.application_uri = "urn:DESKTOP-KHEAST4:CODESYS"
client.connect()

root = client.get_root_node()
print("Connected! Root node:", root)

client.disconnect()