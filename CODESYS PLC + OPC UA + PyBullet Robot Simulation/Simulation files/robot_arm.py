# robot_arm.py
import pybullet as p
import pybullet_data
import numpy as np
import time

class RobotArm:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # محیط
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("kuka_iiwa/model.urdf",
                                 basePosition=[0, 0, 0],
                                 useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot)
        self.end_effector = 6  # آخرین joint کوکا
        
        # قطعه
        self.box = self._create_box([0.5, 0, 0.05])
        
        print("✅ Robot environment ready")

    def _create_box(self, position):
        visual = p.createVisualShape(p.GEOM_BOX,
                                      halfExtents=[0.04, 0.04, 0.04],
                                      rgbaColor=[1, 0.3, 0, 1])
        collision = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=[0.04, 0.04, 0.04])
        return p.createMultiBody(baseMass=0.1,
                                  baseCollisionShapeIndex=collision,
                                  baseVisualShapeIndex=visual,
                                  basePosition=position)

    def move_to(self, target_pos, steps=100):
        """حرکت دادن ربات به یه موقعیت با IK"""
        joint_poses = p.calculateInverseKinematics(
            self.robot,
            self.end_effector,
            target_pos
        )
        
        # اعمال به همه joints
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot, i,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=500
            )
        
        # صبر کن تا ربات برسه
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(1./240.)

    def pick_and_place(self, pick_pos, place_pos):
        """یه سیکل کامل pick & place"""
        print(f"  → Moving to pick position")
        self.move_to([pick_pos[0], pick_pos[1], pick_pos[2] + 0.2])  # بالای قطعه
        self.move_to(pick_pos)  # پایین رفتن
        
        print(f"  → Picking up")
        # Constraint = چسباندن قطعه به ربات
        constraint = p.createConstraint(
            self.robot, self.end_effector,
            self.box, -1,
            p.JOINT_FIXED,
            [0, 0, 0], [0, 0, 0.05], [0, 0, 0]
        )
        
        print(f"  → Moving to place position")
        self.move_to([pick_pos[0], pick_pos[1], pick_pos[2] + 0.3])  # بالا
        self.move_to([place_pos[0], place_pos[1], place_pos[2] + 0.2])
        self.move_to(place_pos)
        
        print(f"  → Placing")
        p.removeConstraint(constraint)
        self.move_to([place_pos[0], place_pos[1], place_pos[2] + 0.3])  # عقب
        
        print(f"  ✅ Pick & Place complete")

    def home_position(self):
        """برگشت به حالت اولیه"""
        for i in range(self.num_joints):
            p.setJointMotorControl2(self.robot, i,
                                     p.POSITION_CONTROL,
                                     targetPosition=0, force=500)
        for _ in range(200):
            p.stepSimulation()
            time.sleep(1./240.)

    def disconnect(self):
        p.disconnect()


# تست مستقیم
if __name__ == "__main__":
    arm = RobotArm()
    time.sleep(1)
    
    arm.pick_and_place(
        pick_pos=[0.5, 0, 0.05],   # جایی که قطعه هست
        place_pos=[0.3, 0.4, 0.05]  # جایی که باید گذاشته بشه
    )
    
    time.sleep(2)
    arm.disconnect()


