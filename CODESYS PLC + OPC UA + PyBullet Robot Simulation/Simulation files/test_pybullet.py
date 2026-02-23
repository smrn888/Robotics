# test_pybullet.py
import pybullet as p
import pybullet_data
import time

# اتصال به simulator
physicsClient = p.connect(p.GUI)  # GUI = پنجره گرافیکی باز میشه

# مسیر فایل‌های پیش‌فرض PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# گرانش
p.setGravity(0, 0, -9.81)

# زمین
planeId = p.loadURDF("plane.urdf")

# ربات Kuka (ساده و معروف)
robotId = p.loadURDF("kuka_iiwa/model.urdf",
                      basePosition=[0, 0, 0],
                      useFixedBase=True)

print(f"✅ Robot loaded! ID: {robotId}")
print(f"   Number of joints: {p.getNumJoints(robotId)}")

# نمایش اطلاعات joints
for i in range(p.getNumJoints(robotId)):
    info = p.getJointInfo(robotId, i)
    print(f"   Joint {i}: {info[1].decode()} | type: {info[2]}")

# یه جعبه (قطعه‌ای که باید pick بشه)
box_visual = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[0.05, 0.05, 0.05],
                                  rgbaColor=[1, 0, 0, 1])  # قرمز
box_collision = p.createCollisionShape(p.GEOM_BOX,
                                        halfExtents=[0.05, 0.05, 0.05])
boxId = p.createMultiBody(baseMass=0.1,
                           baseCollisionShapeIndex=box_collision,
                           baseVisualShapeIndex=box_visual,
                           basePosition=[0.5, 0, 0.05])

print(f"✅ Box created! ID: {boxId}")

# اجرای simulation
for i in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
print("🔌 Done")