from opcua import Client

client = Client("opc.tcp://localhost:4840")
client.connect()

# خواندن وضعیت موتور از PLC
motor_node = client.get_node("ns=4;s=|var|Device.Application.PLC_PRG.Motor")
motor_status = motor_node.get_value()
print(f"Motor: {motor_status}")

client.disconnect()