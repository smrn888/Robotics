# vision_test.py - نسخه fixed با نمایش واضح
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time

CAM_WIDTH  = 640
CAM_HEIGHT = 480
TARGET_COLOR = "red"

COLOR_RANGES = {
    "red": (np.array([0, 120, 70]),    np.array([10, 255, 255])),
    "green": (np.array([40, 50, 50]),  np.array([80, 255, 255])),
    "blue": (np.array([100, 50, 50]),  np.array([130, 255, 255])),
    "yellow": (np.array([20, 100, 100]), np.array([35, 255, 255])),
}

def setup_scene():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5, cameraYaw=0,
        cameraPitch=-60, cameraTargetPosition=[0.5, 0, 0]
    )

    configs = [
        ([0.5,  0.0,  0.05], [1, 0,   0,   1], "red"),
        ([0.4,  0.3,  0.05], [0, 0.5, 1,   1], "blue"),
        ([0.6, -0.2,  0.05], [0, 0.8, 0,   1], "green"),
        ([0.3, -0.3,  0.05], [1, 1,   0,   1], "yellow"),
        ([0.55, 0.25, 0.05], [1, 0,   0,   1], "red"),
    ]

    boxes = []
    for pos, color, name in configs:
        vis = p.createVisualShape(p.GEOM_BOX,
                                   halfExtents=[0.04, 0.04, 0.04],
                                   rgbaColor=color)
        col = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[0.04, 0.04, 0.04])
        bid = p.createMultiBody(0.1, col, vis, pos)
        boxes.append({"id": bid, "color": name, "pos": pos})
    return boxes

def get_camera_image():
    view = p.computeViewMatrix(
        cameraEyePosition=[0.5, 0.0, 1.2],
        cameraTargetPosition=[0.5, 0.0, 0.0],
        cameraUpVector=[0, 1, 0]
    )
    proj = p.computeProjectionMatrixFOV(
        fov=60, aspect=CAM_WIDTH/CAM_HEIGHT,
        nearVal=0.1, farVal=10
    )
    _, _, rgb, depth, _ = p.getCameraImage(
        CAM_WIDTH, CAM_HEIGHT, view, proj,
        renderer=p.ER_TINY_RENDERER
    )
    return (np.array(rgb,    dtype=np.uint8)[:,:,:3],
            np.array(depth).reshape(CAM_HEIGHT, CAM_WIDTH),
            view, proj)

def detect_and_visualize(rgb, depth, view, proj):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # ── Mask قرمز ──────────────────────────────────
    lo1, hi1 = COLOR_RANGES["red"]
    lo2, hi2 = np.array([170,120,70]), np.array([180,255,255])
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lo1, hi1),
        cv2.inRange(hsv, lo2, hi2)
    )
    kernel = np.ones((5,5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ── تصویر خروجی ────────────────────────────────
    output = bgr.copy()

    # نوار اطلاعات بالا
    cv2.rectangle(output, (0,0), (CAM_WIDTH, 35), (30,30,30), -1)
    cv2.putText(output, "Vision System - Looking for RED boxes",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255,255,255), 1)

    red_boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 200:
            continue

        M  = cv2.moments(cnt)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        x, y, w, h = cv2.boundingRect(cnt)

        # ── pixel → world ───────────────────────
        x_ndc =  (2.0*cx/CAM_WIDTH)  - 1.0
        y_ndc = -((2.0*cy/CAM_HEIGHT) - 1.0)
        z_ndc =  2.0*depth[cy,cx] - 1.0

        proj_m = np.array(proj).reshape(4,4).T
        view_m = np.array(view).reshape(4,4).T
        clip   = np.array([x_ndc, y_ndc, z_ndc, 1.0])
        eye    = np.linalg.inv(proj_m) @ clip
        eye   /= eye[3]
        world  = np.linalg.inv(view_m) @ eye
        wx, wy = world[0], world[1]

        red_boxes.append({"pixel": (cx,cy), "world": (wx, wy, 0.05)})

        # ── رسم روی تصویر ───────────────────────
        # کادر ضخیم قرمز
        cv2.rectangle(output, (x,y), (x+w,y+h), (0,0,255), 3)
        # دایره مرکز
        cv2.circle(output, (cx,cy), 8, (0,255,0), -1)
        # خط از مرکز
        cv2.line(output, (cx,cy), (cx,cy-30), (0,255,0), 2)

        # برچسب با پس‌زمینه تیره
        label = f"RED  x:{wx:.2f} y:{wy:.2f}"
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(output,
                      (x, y-th-12), (x+tw+6, y),
                      (0,0,180), -1)
        cv2.putText(output, label,
                    (x+3, y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255,255,255), 2)

    # تعداد در گوشه
    count_label = f"Detected: {len(red_boxes)} red box(es)"
    cv2.putText(output, count_label,
                (10, CAM_HEIGHT-15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0,255,255), 2)

    # ── نمایش mask هم کنار ──────────────────────
    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(mask_colored, "RED mask",
                (10,25), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0,255,255), 2)

    combined = np.hstack([output, mask_colored])
    cv2.imshow("Vision System", combined)
    cv2.waitKey(30)  # ← 30ms صبر کن تا پنجره render بشه

    return red_boxes

def main():
    print("🎥 Vision Test Starting...")
    boxes = setup_scene()
    for _ in range(50):
        p.stepSimulation()

    print(f"🔍 Detecting RED boxes...\n")

    for frame in range(300):
        p.stepSimulation()
        time.sleep(1./60.)

        if frame % 15 == 0:
            rgb, depth, vm, pm = get_camera_image()
            red_boxes = detect_and_visualize(rgb, depth, vm, pm)

            print(f"Frame {frame:3d} | {len(red_boxes)} red box(es) found:")
            for i, b in enumerate(red_boxes):
                print(f"   → Box {i+1}: "
                      f"world=({b['world'][0]:.3f}, "
                      f"{b['world'][1]:.3f})")

    cv2.destroyAllWindows()
    p.disconnect()
    print("\n✅ Done")

if __name__ == "__main__":
    main()