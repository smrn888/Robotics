# integrated_system_v6.py
# Fix 1: Blue box overlap detection — scan PyBullet state directly, not just vision
# Fix 2: PLC auto-reconnect + keepalive ping during robot execution
# Author: Moein | KU Leuven Portfolio Project

import asyncio
import pybullet as p
import pybullet_data
import time
import logging
import json
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from asyncua import Client
from datetime import datetime

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s'
)
log = logging.getLogger(__name__)

# ─── Config ───────────────────────────────────────────
PLC_URL = "opc.tcp://localhost:4840"
NS      = "ns=4;s=|var|CODESYS Control Win V3 x64.Application.PLC_PRG."

VIDEO_FPS    = 30
VIDEO_WIDTH  = 640
VIDEO_HEIGHT = 480
RECORD_VIDEO = True

BASKET_POSITIONS = {
    "red":    [ 0.65,  0.50, 0.0],
    "blue":   [-0.10,  0.55, 0.0],
    "green":  [ 0.65, -0.50, 0.0],
    "yellow": [-0.10, -0.55, 0.0],
}
BASKET_DROP_HEIGHT = 0.12

# ─── Trajectory ───────────────────────────────────────
RRT_MAX_ITER   = 200
RRT_STEP_SIZE  = 0.20
RRT_GOAL_BIAS  = 0.30
SPLINE_WPT     = 25
SIM_STEPS_SLOW = 6
SIM_STEPS_FAST = 3

# ─── PLC keepalive ────────────────────────────────────
# هر چند ثانیه یه ping به PLC بزنه تا session زنده بمونه
PLC_KEEPALIVE_INTERVAL = 8.0   # seconds

latency_log = []


# ══════════════════════════════════════════════════════
# TRAJECTORY PLANNER
# ══════════════════════════════════════════════════════
class TrajectoryPlanner:
    def __init__(self, robot_id, num_joints, ee_idx):
        self.robot = robot_id
        self.n     = num_joints
        self.ee    = ee_idx
        self.q_min = np.array([-2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-3.05])
        self.q_max = np.array([ 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05])

    def plan(self, q_start, target_pos, n_points=SPLINE_WPT):
        q_goal = self._ik(target_pos)
        if q_goal is None:
            return []
        if np.linalg.norm(q_goal - q_start) < 0.3:
            return self._lerp(q_start, q_goal, n_points)
        raw = self._rrt(q_start, q_goal)
        return self._smooth(raw, n_points)

    def get_joints(self):
        return np.array([p.getJointState(self.robot, i)[0]
                         for i in range(self.n)])

    def _ik(self, pos):
        j = p.calculateInverseKinematics(
            self.robot, self.ee, pos,
            maxNumIterations=100, residualThreshold=1e-3)
        if j is None or len(j) < self.n:
            return None
        return np.clip(np.array(j[:self.n]), self.q_min, self.q_max)

    def _rrt(self, q0, qg):
        nodes  = [q0]; parent = [-1]
        for _ in range(RRT_MAX_ITER):
            qr = (qg if np.random.rand() < RRT_GOAL_BIAS
                  else self.q_min + np.random.rand(self.n)*(self.q_max-self.q_min))
            dists = np.linalg.norm(np.array(nodes)-qr, axis=1)
            nn    = int(np.argmin(dists)); qn = nodes[nn]
            diff  = qr - qn; d = np.linalg.norm(diff)
            if d < 1e-6: continue
            qnew = np.clip(qn + (diff/d)*min(RRT_STEP_SIZE,d), self.q_min, self.q_max)
            nodes.append(qnew); parent.append(nn)
            if np.linalg.norm(qnew-qg) < RRT_STEP_SIZE*1.2:
                nodes.append(qg); parent.append(len(nodes)-2)
                break
        path=[]; idx=len(nodes)-1
        while idx!=-1: path.append(nodes[idx]); idx=parent[idx]
        path.reverse(); return path

    def _smooth(self, raw, n):
        arr = np.array(raw)
        t   = np.linspace(0,1,len(arr))
        cs  = CubicSpline(t, arr, bc_type='clamped')
        ts  = self._trap(n)
        w   = cs(ts)
        return [w[i] for i in range(n)]

    @staticmethod
    def _lerp(q0, q1, n):
        return [q0+(q1-q0)*t for t in np.linspace(0,1,n)]

    @staticmethod
    def _trap(n, acc=0.25, dec=0.25):
        s = np.linspace(0,1,n); t = np.zeros(n)
        vm = 1.0/(1.0-0.5*acc-0.5*dec)
        for i,si in enumerate(s):
            if si<acc:        t[i]=0.5*vm*si**2/acc
            elif si<1-dec:    t[i]=vm*acc/2+vm*(si-acc)
            else:
                ds=si-(1-dec); t[i]=vm*acc/2+vm*(1-acc-dec)+vm*ds-0.5*vm*ds**2/dec
        t/=t[-1]+1e-9; return np.clip(t,0,1)


# ══════════════════════════════════════════════════════
# VISION SYSTEM
# ══════════════════════════════════════════════════════
class VisionSystem:
    def __init__(self):
        self.W = VIDEO_WIDTH; self.H = VIDEO_HEIGHT
        self.view = p.computeViewMatrix([0.3,0,1.5],[0.3,0,0],[0,1,0])
        self.proj = p.computeProjectionMatrixFOV(70,self.W/self.H,0.1,10)
        log.info("✅ Vision ready")

    def find_all(self):
        rgb, depth = self._cap()
        results = []
        for color in ["red","blue","green","yellow"]:
            for det in self._detect(rgb, color):
                cx,cy = det["pixel_center"]
                wp = self._p2w(cx,cy,depth); wp[2]=0.05
                results.append({"color":color,"position":wp})
        if results:
            s={}
            for r in results: s[r["color"]]=s.get(r["color"],0)+1
            log.info(f"👁️ Vision: {s}")
        else:
            log.info("👁️ Vision: nothing")
        return results

    def show(self):
        rgb,depth = self._cap()
        bgr = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
        clrs={"red":(0,0,255),"blue":(255,100,0),
              "green":(0,200,0),"yellow":(0,220,220)}
        for color,dc in clrs.items():
            for det in self._detect(rgb,color):
                cx,cy=det["pixel_center"]; x,y,w,h=det["bbox"]
                wp=self._p2w(cx,cy,depth)
                cv2.rectangle(bgr,(x,y),(x+w,y+h),dc,2)
                cv2.circle(bgr,(cx,cy),5,dc,-1)
                cv2.putText(bgr,f"{color}({wp[0]:.2f},{wp[1]:.2f})",
                            (x,y-8),cv2.FONT_HERSHEY_SIMPLEX,0.4,dc,1)
        cv2.imshow("Vision",bgr); cv2.waitKey(1)

    def _cap(self):
        _,_,rgb,depth,_=p.getCameraImage(self.W,self.H,self.view,self.proj,
                                          renderer=p.ER_TINY_RENDERER)
        return (np.array(rgb,dtype=np.uint8)[:,:,:3],
                np.array(depth).reshape(self.H,self.W))

    def _detect(self,rgb,color):
        bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
        hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
        if color=="red":
            mask=cv2.bitwise_or(
                cv2.inRange(hsv,np.array([0,100,50]),np.array([12,255,255])),
                cv2.inRange(hsv,np.array([165,100,50]),np.array([180,255,255])))
        elif color=="green":
            mask=cv2.inRange(hsv,np.array([35,40,40]),np.array([90,255,255]))
        elif color=="blue":
            mask=cv2.inRange(hsv,np.array([85,40,40]),np.array([135,255,255]))
        elif color=="yellow":
            mask=cv2.inRange(hsv,np.array([18,80,80]),np.array([38,255,255]))
        else: return []
        k=np.ones((5,5),np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,k)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,k)
        cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        dets=[]
        for cnt in cnts:
            if cv2.contourArea(cnt)<150: continue
            M=cv2.moments(cnt)
            if M["m00"]==0: continue
            cx=int(M["m10"]/M["m00"]); cy=int(M["m01"]/M["m00"])
            x,y,w,h=cv2.boundingRect(cnt)
            dets.append({"pixel_center":(cx,cy),"bbox":(x,y,w,h)})
        return dets

    def _p2w(self,px,py,depth):
        xn=(2.0*px/self.W)-1.0; yn=-((2.0*py/self.H)-1.0)
        zn=2.0*depth[py,px]-1.0
        pm=np.array(self.proj).reshape(4,4).T
        vm=np.array(self.view).reshape(4,4).T
        clip=np.array([xn,yn,zn,1.0])
        eye=np.linalg.inv(pm)@clip; eye/=eye[3]
        w=np.linalg.inv(vm)@eye; return list(w[:3])


# ══════════════════════════════════════════════════════
# ROBOT ARM
# ══════════════════════════════════════════════════════
class RobotArm:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.81)
        p.resetDebugVisualizerCamera(2.2,30,-35,[0.3,0,0])
        p.loadURDF("plane.urdf")
        self.robot=p.loadURDF("kuka_iiwa/model.urdf",
                               basePosition=[0,0,0],useFixedBase=True)
        self.n=p.getNumJoints(self.robot); self.ee=6
        self.planner=TrajectoryPlanner(self.robot,self.n,self.ee)
        self.boxes=self._make_boxes()
        self.baskets=self._make_baskets()
        self._make_conveyor()
        self._traj_lines=[]
        self._vid_view=p.computeViewMatrix([2,1.5,1.5],[0.3,0,0.1],[0,0,1])
        self._vid_proj=p.computeProjectionMatrixFOV(60,VIDEO_WIDTH/VIDEO_HEIGHT,0.1,100)
        self.frames=[]; self.vid_path=f"robot_demo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        log.info("✅ Robot ready")

    def _make_boxes(self):
        cfgs=[
            ([0.45, 0.05,0.05],[1,  0,  0,  1],"red"),
            ([0.50,-0.10,0.05],[1,  0,  0,  1],"red"),
            ([0.35, 0.15,0.05],[0,0.5,  1,  1],"blue"),
            ([0.55, 0.20,0.05],[0,0.5,  1,  1],"blue"),   # ← آبی دوم فاصله بیشتر
            ([0.40,-0.20,0.05],[0,0.8,  0,  1],"green"),
            ([0.45,-0.15,0.05],[0,0.8,  0,  1],"green"),
            ([0.30, 0.10,0.05],[1,  1,  0,  1],"yellow"),
        ]
        boxes=[]
        for pos,rgba,name in cfgs:
            v=p.createVisualShape(p.GEOM_BOX,halfExtents=[.04,.04,.04],rgbaColor=rgba)
            c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[.04,.04,.04])
            bid=p.createMultiBody(.1,c,v,pos)
            boxes.append({"id":bid,"color":name,"home":list(pos),"done":False})
        return boxes

    def _make_baskets(self):
        baskets={}
        RGBA={"red":[.9,.1,.1,.85],"blue":[.1,.4,.9,.85],
              "green":[.1,.8,.2,.85],"yellow":[.9,.85,.1,.85]}
        W,H,TH=0.14,0.06,0.01
        for color,bp in BASKET_POSITIONS.items():
            bx,by,_=bp; rgba=RGBA[color]
            fv=p.createVisualShape(p.GEOM_BOX,halfExtents=[W,W,.005],rgbaColor=rgba)
            fc=p.createCollisionShape(p.GEOM_BOX,halfExtents=[W,W,.005])
            p.createMultiBody(0,fc,fv,[bx,by,.005])
            for wpos,he in [([bx,by+W,H/2],[W,TH,H]),([bx,by-W,H/2],[W,TH,H]),
                             ([bx+W,by,H/2],[TH,W,H]),([bx-W,by,H/2],[TH,W,H])]:
                wv=p.createVisualShape(p.GEOM_BOX,halfExtents=he,rgbaColor=rgba)
                wc=p.createCollisionShape(p.GEOM_BOX,halfExtents=he)
                p.createMultiBody(0,wc,wv,wpos)
            p.addUserDebugText(color.upper(),[bx,by,.22],textColorRGB=rgba[:3],textSize=1.2)
            baskets[color]={"position":bp,"count":0}
            log.info(f"🧺 Basket: {color} at ({bx:.2f},{by:.2f})")
        return baskets

    def _make_conveyor(self):
        v=p.createVisualShape(p.GEOM_BOX,halfExtents=[.35,.08,.01],rgbaColor=[.25,.25,.25,1])
        c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[.35,.08,.01])
        p.createMultiBody(0,c,v,[.42,0,.01])

    # ── FIX: build sorted list from PyBullet state, not just vision ──────────

    def build_sort_queue(self, vision_detections: list) -> list:
        """
        ترکیب vision + PyBullet state:
        هر undone box رو مستقیم از PyBullet میخونه، مستقل از vision accuracy.
        خروجی: list of {"color", "real_pos"} مرتب شده برای pick
        """
        queue = []
        seen_ids = set()

        # اول از vision detections استفاده کن (ترتیب detection)
        for det in vision_detections:
            color = det["color"]
            vpos  = det["position"]
            box   = self._get_box(color, vpos)
            if box is None or box["id"] in seen_ids:
                continue
            rp,_ = p.getBasePositionAndOrientation(box["id"])
            queue.append({"color": color, "real_pos": list(rp),
                          "box": box})
            seen_ids.add(box["id"])

        # بعد هر undone box که vision miss کرده رو اضافه کن
        for box in self.boxes:
            if box["done"] or box["id"] in seen_ids:
                continue
            rp,_ = p.getBasePositionAndOrientation(box["id"])
            log.info(f"   👁️ Vision missed {box['color']} box — "
                     f"added from PyBullet state at "
                     f"({rp[0]:.2f},{rp[1]:.2f})")
            queue.append({"color": box["color"], "real_pos": list(rp),
                          "box": box})
            seen_ids.add(box["id"])

        return queue

    def _get_box(self, color, vpos, tol=0.30):
        best,bd=None,float("inf")
        for b in self.boxes:
            if b["color"]!=color or b["done"]: continue
            pos,_=p.getBasePositionAndOrientation(b["id"])
            d=np.linalg.norm(np.array(pos[:2])-np.array(vpos[:2]))
            if d<bd: bd,best=d,b
        return best

    def pending(self): return [b for b in self.boxes if not b["done"]]

    def reset(self):
        for b in self.boxes:
            p.resetBasePositionAndOrientation(b["id"],b["home"],[0,0,0,1])
            b["done"]=False
        for bk in self.baskets.values(): bk["count"]=0
        log.info("🔄 Reset")

    def status(self):
        log.info("🧺 "+" | ".join(f"{c}:{v['count']}"
                                   for c,v in self.baskets.items()))

    # ── Motion ────────────────────────────────────────

    def _exec(self, waypoints, sim_steps=SIM_STEPS_SLOW, draw=False):
        if not waypoints: return
        if draw:
            for lid in self._traj_lines: p.removeUserDebugItem(lid)
            self._traj_lines.clear()
            prev=None
            for q in waypoints[::3]:
                self._apply(q,0)
                ee,_=p.getLinkState(self.robot,self.ee)[:2]
                if prev:
                    lid=p.addUserDebugLine(prev,ee,[0.2,0.8,1.0],2.0,10.0)
                    self._traj_lines.append(lid)
                prev=ee
        for q in waypoints:
            self._apply(q,sim_steps)
            if RECORD_VIDEO: self._cap_frame()

    def _apply(self, q, steps):
        for i in range(self.n):
            p.setJointMotorControl2(self.robot,i,p.POSITION_CONTROL,
                targetPosition=float(q[i]),force=500,maxVelocity=2.5)
        for _ in range(steps):
            p.stepSimulation(); time.sleep(1./240.)

    # ── Pick & Place (موقعیت واقعی از PyBullet، نه vision) ───────────────

    def pick_and_place(self, cycle, color, real_pos, box) -> float:
        bp  = BASKET_POSITIONS[color]
        plc = [bp[0], bp[1], BASKET_DROP_HEIGHT]
        pick= [real_pos[0], real_pos[1], real_pos[2]+0.02]

        log.info(f"🤖 [C{cycle}] {color}: "
                 f"({pick[0]:.2f},{pick[1]:.2f}) → basket")

        t0 = time.perf_counter()
        q  = self.planner.get_joints()

        t1=self.planner.plan(q,[pick[0],pick[1],pick[2]+0.28])
        self._exec(t1,SIM_STEPS_SLOW,draw=True)

        q=self.planner.get_joints()
        t2=self.planner.plan(q,pick,n_points=15)
        self._exec(t2,SIM_STEPS_FAST)

        cst=p.createConstraint(self.robot,self.ee,box["id"],-1,
            p.JOINT_FIXED,[0,0,0],[0,0,.05],[0,0,0])

        q=self.planner.get_joints()
        t3=self.planner.plan(q,[pick[0],pick[1],pick[2]+0.35],n_points=15)
        self._exec(t3,SIM_STEPS_FAST)

        q=self.planner.get_joints()
        above=[plc[0],plc[1],plc[2]+0.30]
        t4=self.planner.plan(q,above)
        self._exec(t4,SIM_STEPS_SLOW,draw=True)

        q=self.planner.get_joints()
        t5=self.planner.plan(q,plc,n_points=15)
        self._exec(t5,SIM_STEPS_FAST)

        p.removeConstraint(cst)

        q=self.planner.get_joints()
        t6=self.planner.plan(q,above,n_points=15)
        self._exec(t6,SIM_STEPS_FAST)

        box["done"]=True; self.baskets[color]["count"]+=1
        dur=(time.perf_counter()-t0)*1000
        total=sum(len(t) for t in [t1,t2,t3,t4,t5,t6])
        log.info(f"   ✅ {color} done {dur:.0f}ms | {total} wpts")
        return dur

    def _cap_frame(self):
        _,_,rgb,_,_=p.getCameraImage(VIDEO_WIDTH,VIDEO_HEIGHT,
            self._vid_view,self._vid_proj,renderer=p.ER_TINY_RENDERER)
        self.frames.append(cv2.cvtColor(
            np.array(rgb,dtype=np.uint8)[:,:,:3],cv2.COLOR_RGB2BGR))

    def save_video(self):
        if not self.frames or not RECORD_VIDEO: return
        out=cv2.VideoWriter(self.vid_path,cv2.VideoWriter_fourcc(*'mp4v'),
            VIDEO_FPS,(VIDEO_WIDTH,VIDEO_HEIGHT))
        for f in self.frames: out.write(f)
        out.release(); log.info(f"🎬 {self.vid_path}")

    def disconnect(self): p.disconnect()


# ══════════════════════════════════════════════════════
# PLC CONTROLLER — با auto-reconnect و keepalive
# ══════════════════════════════════════════════════════
class PLCController:
    def __init__(self):
        self._url  = PLC_URL
        self.client= Client(PLC_URL)
        self.client.set_user(""); self.client.set_password("")
        self.nodes = {}
        self._connected = False

    async def connect(self):
        await self.client.connect()
        for name in ["Part_Detected","Pick_Request","Pick_Done"]:
            self.nodes[name]=self.client.get_node(NS+name)
        self._connected=True
        log.info("✅ PLC connected via OPC UA")

    async def _reconnect(self):
        log.info("🔄 Reconnecting to PLC...")
        try:
            self.client=Client(self._url)
            self.client.set_user(""); self.client.set_password("")
            await self.client.connect()
            for name in ["Part_Detected","Pick_Request","Pick_Done"]:
                self.nodes[name]=self.client.get_node(NS+name)
            self._connected=True
            log.info("✅ PLC reconnected")
        except Exception as e:
            self._connected=False
            log.warning(f"⚠️ Reconnect failed: {e}")

    async def read(self, name:str) -> bool:
        try:
            return await self.nodes[name].read_value()
        except Exception:
            self._connected=False
            return False

    async def write(self, name:str, val:bool):
        try:
            await self.nodes[name].write_value(val)
        except Exception:
            self._connected=False
            log.warning(f"⚠️ PLC write failed ({name}={val}) — will retry after reconnect")

    async def ping(self):
        """keepalive — خوندن یه node کوچیک تا session زنده بمونه"""
        try:
            await self.nodes["Part_Detected"].read_value()
            return True
        except Exception:
            self._connected=False
            return False

    async def ensure_connected(self):
        if not self._connected:
            await self._reconnect()
        return self._connected

    async def disconnect(self):
        try: await self.client.disconnect()
        except Exception: pass


# ══════════════════════════════════════════════════════
# MAIN LOOP
# ══════════════════════════════════════════════════════
async def main():
    robot  = RobotArm()
    vision = VisionSystem()
    plc    = PLCController()
    await plc.connect()

    cycle_count  = 0
    total_sorted = 0
    round_num    = 0
    last_ping    = time.perf_counter()

    log.info("🚀 v6 — Fast RRT+Spline | Full box detection | PLC keepalive\n")

    try:
        while True:
            # ── PLC keepalive ────────────────────────────
            now = time.perf_counter()
            if now - last_ping > PLC_KEEPALIVE_INTERVAL:
                ok = await plc.ping()
                if not ok:
                    await plc.ensure_connected()
                last_ping = now

            t_poll = time.perf_counter()
            if not await plc.ensure_connected():
                await asyncio.sleep(2.0)
                continue

            part = await plc.read("Part_Detected")

            if part:
                if not robot.pending():
                    round_num+=1
                    log.info(f"🏆 Round {round_num} complete!")
                    robot.status(); robot.reset()
                    await asyncio.sleep(1.0); continue

                cycle_count+=1
                opcua_ms=(time.perf_counter()-t_poll)*1000
                log.info(f"\n{'='*54}")
                log.info(f"📦 Cycle #{cycle_count} | "
                         f"{len(robot.pending())} pending | Round {round_num+1}")

                t_vis=time.perf_counter()
                detections=vision.find_all()
                vision.show()
                vision_ms=(time.perf_counter()-t_vis)*1000

                # ── FIX: build queue از vision + PyBullet state ──
                queue = robot.build_sort_queue(detections)
                q_summary = [(item["color"],
                               f"({item['real_pos'][0]:.2f},{item['real_pos'][1]:.2f})")
                              for item in queue]
                log.info(f"   📋 Sort queue: {q_summary}")

                await plc.write("Pick_Request", True)

                cycle_durs=[]
                for item in queue:
                    # keepalive حین robot execution
                    last_ping=time.perf_counter()
                    await plc.ping()

                    dur=robot.pick_and_place(
                        cycle_count,
                        item["color"],
                        item["real_pos"],
                        item["box"])
                    if dur>0:
                        cycle_durs.append(dur)
                        total_sorted+=1

                    # ping بعد از هر pick
                    await plc.ping()
                    last_ping=time.perf_counter()

                robot_ms=sum(cycle_durs)

                # PLC Done handshake
                t_wait=time.perf_counter(); timeout=0
                while timeout<60:
                    if not await plc.ensure_connected():
                        break
                    try:
                        if await plc.read("Pick_Done"): break
                    except Exception:
                        break
                    await asyncio.sleep(0.05); timeout+=1
                plc_ms=(time.perf_counter()-t_wait)*1000

                await plc.write("Pick_Request", False)
                last_ping=time.perf_counter()

                entry={
                    "cycle":             cycle_count,
                    "round":             round_num+1,
                    "timestamp":         datetime.now().isoformat(),
                    "opcua_latency_ms":  round(opcua_ms,2),
                    "vision_ms":         round(vision_ms,2),
                    "robot_duration_ms": round(robot_ms,2),
                    "plc_response_ms":   round(plc_ms,2),
                    "boxes_sorted":      len(cycle_durs),
                    "total_sorted":      total_sorted,
                    "total_cycle_ms":    round(opcua_ms+vision_ms+robot_ms+plc_ms,2),
                }
                latency_log.append(entry)
                log.info(f"📊 OPC:{opcua_ms:.1f} | Vis:{vision_ms:.0f} | "
                         f"Robot:{robot_ms:.0f} | Sorted:{len(cycle_durs)}")
                robot.status()

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        log.info("\n⛔ Stopped")
    finally:
        cv2.destroyAllWindows()
        robot.save_video()
        _plot()
        await plc.disconnect()
        robot.disconnect()


# ══════════════════════════════════════════════════════
# PLOT
# ══════════════════════════════════════════════════════
def _plot():
    if not latency_log:
        log.warning("No data."); return
    with open("cycle_latency_log.json","w") as f:
        json.dump(latency_log,f,indent=2)
    C=[e["cycle"] for e in latency_log]
    O=[e["opcua_latency_ms"] for e in latency_log]
    V=[e["vision_ms"] for e in latency_log]
    R=[e["robot_duration_ms"] for e in latency_log]
    P=[e["plc_response_ms"] for e in latency_log]
    T=[e["total_cycle_ms"] for e in latency_log]
    S=[e["boxes_sorted"] for e in latency_log]
    fig,axes=plt.subplots(3,1,figsize=(13,11))
    fig.suptitle("Industry 4.0 v6 — Fast RRT+Spline | Full Detection\n"
                 "CODESYS PLC → OPC UA → Python → PyBullet Kuka IIWA",
                 fontsize=13,fontweight='bold')
    b1=np.array(O); b2=b1+np.array(V); b3=b2+np.array(R)
    axes[0].bar(C,O,label="OPC UA",color="#2196F3",width=0.7)
    axes[0].bar(C,V,label="Vision",color="#9C27B0",bottom=b1,width=0.7)
    axes[0].bar(C,R,label="Robot(RRT+Spline)",color="#4CAF50",bottom=b2,width=0.7)
    axes[0].bar(C,P,label="PLC",color="#FF9800",bottom=b3,width=0.7)
    axes[0].set_title("Per-Cycle Latency"); axes[0].set_ylabel("ms")
    axes[0].legend(); axes[0].grid(axis='y',alpha=.3)
    axes[1].bar(C,S,color="#E91E63",width=0.7)
    if S: axes[1].axhline(sum(S)/len(S),color='k',linestyle='--',
                          label=f"Mean:{sum(S)/len(S):.1f}")
    axes[1].set_title("Boxes Sorted/Cycle"); axes[1].set_ylabel("Count")
    axes[1].legend(); axes[1].grid(axis='y',alpha=.3)
    mt=sum(T)/len(T)
    axes[2].plot(C,T,marker='o',color="#9C27B0",linewidth=1.5,markersize=5)
    axes[2].axhline(mt,color='red',linestyle='--',label=f"Mean:{mt:.0f}ms")
    axes[2].set_title("Total Cycle Time"); axes[2].set_xlabel("Cycle #")
    axes[2].set_ylabel("ms"); axes[2].legend(); axes[2].grid(alpha=.3)
    plt.tight_layout()
    plt.savefig("latency_analysis.png",dpi=150); plt.show()
    print("\n"+"="*54)
    print("📋 SUMMARY v6")
    print("="*54)
    print(f"   Cycles: {len(latency_log)} | Sorted: {latency_log[-1]['total_sorted']}")
    print(f"   OPC UA: {sum(O)/len(O):.2f}ms | Vision: {sum(V)/len(V):.2f}ms")
    print(f"   Robot:  {sum(R)/len(R):.0f}ms | Total: {mt:.0f}ms")
    print("="*54)


if __name__=="__main__":
    asyncio.run(main())