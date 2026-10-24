#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""连杆凸块 -> 障碍物的最小距离（机器人规则运动 + 场景中的立体障碍物）。

默认末端执行器是"铣削电主轴 + Ø10 立铣刀"（URDF/ur5_spindle.urdf）：
ee_link 之后是 spindle_mount_link -> spindle_link -> tool_link，三个关节都是 fixed，
网格由 make_spindle_meshes.py 生成。想换回原来的 Robotiq 2F-140 夹爪，只要
`--urdf URDF/ur5_robotiq_140.urdf`（下面提到 finger_joint / mimic 的逻辑只在夹爪模型上生效）。

距离定义
--------
只算 "机器人自己 vs 障碍物"，不再计算机器人本体各连杆之间的距离
（那些碰撞体在关节/夹爪处本来就互相重叠，会算出负值，没有物理意义）。

    d_piece(t) = 单个凸块到障碍物的最小距离
    d_link(t)  = min over 该连杆所有凸块

* 结果全部为非负数：真发生穿透时截断为 0，并在控制台给出 [warn] 提示；
* 大于 --max-distance 的记为 --max-distance；
* 凸块按 "连杆顺序 + 连杆内凸块编号" 排列（base_link#p0, base_link#p1, ...）。

处理流程
--------
1. 解析 URDF，取每个 link 的 <collision>（ur5 是 STL，夹爪是 STL + 少量 box，
   电主轴/铣刀是 make_spindle_meshes.py 生成的 STL）；
2. 每个碰撞网格做凸分解：
     - 已经凸的（volume/hull_volume >= --convex-ratio）直接用它的凸包；
     - 凹的用 p.vhacd() 切成若干凸块（缓存到 out/cache，可 --recompute 重算）；
     - box 直接展开成 8 个角点（本身就是一个凸块）；
3. 每个凸块写成独立 OBJ（已变换到 link 坐标系），再用 GEOM_MESH 建成独立静态刚体
   （collisionMargin=0，保证测距精确）；障碍物是一个 box 刚体（凸的，1 块）；
4. 按规则运动：q_k(t) = home_k + amp*sin(2*pi*freq*t + k*phase_step)；URDF 里
   有 finger_joint（夹爪）时再按 mimic 关系联动，没有就只动手臂；每个采样点用 FK
   把凸块刚体摆到 link 的世界位姿；
5. 用 p.getClosestPoints() 只测 "凸块 <-> 障碍物"，得到 S 个 N 维距离向量；
6. 输出 pieces.csv / piece_distance_to_obstacle.csv / link_distance_to_obstacle.csv /
   piece_distances_all_samples.csv / distances.npz，并在控制台打印每个连杆到障碍物
   在 t=0 和整个运动过程中的最小距离。

注意
----
凸块是 V-HACD 得到的凸包近似：默认参数下与真实网格测距的平均误差 ~1.5 mm，
最大 ~1 cm；要更精确就调小 --vhacd-concavity（更慢）或提高 --convex-ratio。

GUI 模式（--gui）下按 ESC 或关闭窗口会让 Bullet 直接退出渲染服务，之后的
getLinkState / getClosestPoints 都会失败：脚本会捕获这个情况，保留已算完的
采样点并正常退出（不会打 traceback）。批量计算请用无窗口模式。

用法示例
--------
    python link_piece_distance_matrix.py              # 无窗口，30 个采样点
    python link_piece_distance_matrix.py --gui        # 带窗口看机械臂运动
    python link_piece_distance_matrix.py --gui --opengl2 --hold   # 指定渲染器并停在窗口
    python link_piece_distance_matrix.py --samples 60 --amp 0.5
    python link_piece_distance_matrix.py --recompute  # 强制重跑 V-HACD
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import math
import os
import struct
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parent
# 默认末端执行器：铣削电主轴 + Ø10 立铣刀（网格由 make_spindle_meshes.py 生成）
DEFAULT_URDF = ROOT / "URDF" / "ur5_spindle.urdf"
# 旧的末端：Robotiq 2F-140 夹爪（要算夹爪开合时用 --urdf 指到它）
GRIPPER_URDF = ROOT / "URDF" / "ur5_robotiq_140.urdf"


def log(msg: str = "") -> None:
    print(msg, flush=True)


def connected() -> bool:
    """p.isConnected() 在物理服务器已退出（GUI 窗口被关掉）时也可能抛异常。"""
    try:
        return bool(p.isConnected())
    except p.error:
        return False


@contextlib.contextmanager
def quiet_native_stdout(enabled: bool):
    """V-HACD 是 C++ 代码，直接往 fd 1 写进度，用 dup2 临时静音。"""
    if not enabled:
        yield
        return
    sys.stdout.flush()
    saved = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    try:
        yield
    finally:
        os.dup2(saved, 1)
        os.close(devnull)
        os.close(saved)


# ---------------------------------------------------------------------------
# 网格读写 / 凸包工具
# ---------------------------------------------------------------------------
def read_stl(path: Path) -> np.ndarray:
    """读二进制或 ASCII STL，返回 (n, 3, 3) 的三角形数组。"""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if n > 0 and len(data) == 84 + n * 50:
            rec = np.frombuffer(data, dtype=np.uint8, count=n * 50, offset=84).reshape(n, 50)
            return rec[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)

    verts = []
    for line in data.decode("ascii", errors="ignore").splitlines():
        s = line.strip()
        if s[:6].lower() == "vertex":
            verts.append([float(x) for x in s.split()[1:4]])
    if len(verts) < 3 or len(verts) % 3:
        raise ValueError(f"无法解析 STL: {path}")
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


def read_obj(path: Path):
    """读 OBJ，返回 (vertices (n,3), faces (m,3))，面已三角化。"""
    verts, faces = [], []
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "v" and len(parts) >= 4:
                verts.append([float(x) for x in parts[1:4]])
            elif parts[0] == "f" and len(parts) >= 4:
                idx = [int(tok.split("/")[0]) for tok in parts[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1):
                    faces.append([idx[0], idx[k], idx[k + 1]])
    return (
        np.asarray(verts, dtype=np.float64).reshape(-1, 3),
        np.asarray(faces, dtype=int).reshape(-1, 3),
    )


def mesh_triangles(path: Path) -> np.ndarray:
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".stl":
        return read_stl(path)
    if suffix == ".obj":
        verts, faces = read_obj(path)
        if len(faces) == 0:
            raise ValueError(f"{path} 没有三角面")
        return verts[faces]
    raise ValueError(f"不支持的网格格式 {suffix}（{path}），目前支持 STL / OBJ")


def mesh_volume(tris: np.ndarray) -> float:
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    return abs(float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0)


def hull_vertices(points: np.ndarray) -> np.ndarray:
    pts = np.ascontiguousarray(np.asarray(points, dtype=np.float64).reshape(-1, 3))
    pts = np.unique(np.round(pts, 9), axis=0)
    if len(pts) < 4:
        raise ValueError("少于 4 个不重复顶点，构不成凸包")
    return pts[ConvexHull(pts).vertices]


def write_obj(triangles: np.ndarray, path: Path) -> None:
    """把三角形数组写成 OBJ（顶点去重），作为 p.vhacd 的输入。"""
    tris = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
    verts, inv = np.unique(np.round(tris.reshape(-1, 3), 9), axis=0, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    faces = inv.reshape(-1, 3) + 1
    with open(path, "w") as fh:
        fh.write("# generated by link_piece_distance_matrix.py\n")
        for v in verts:
            fh.write("v %.9g %.9g %.9g\n" % (v[0], v[1], v[2]))
        for f in faces:
            fh.write("f %d %d %d\n" % (f[0], f[1], f[2]))


def write_convex_obj(points: np.ndarray, path: Path) -> np.ndarray:
    """把一个点集的凸包写成 OBJ，返回凸包顶点。"""
    pts = hull_vertices(points)
    hull = ConvexHull(pts)
    with open(path, "w") as fh:
        fh.write("o piece\n")
        for v in pts:
            fh.write("v %.9g %.9g %.9g\n" % (v[0], v[1], v[2]))
        for tri in hull.simplices:
            fh.write("f %d %d %d\n" % (int(tri[0]) + 1, int(tri[1]) + 1, int(tri[2]) + 1))
    return pts


def quat_matrix(quat) -> np.ndarray:
    m = p.getMatrixFromQuaternion([float(x) for x in quat])
    return np.asarray(m, dtype=np.float64).reshape(3, 3)


# ---------------------------------------------------------------------------
# URDF 解析
# ---------------------------------------------------------------------------
@dataclass
class Collision:
    kind: str                                            # 'mesh' | 'box'
    path: Path | None = None
    half_extents: np.ndarray | None = None
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    scale: np.ndarray = field(default_factory=lambda: np.ones(3))


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    lower: float = 0.0
    upper: float = 0.0
    mimic: tuple | None = None                           # (parent_joint, multiplier, offset)


def parse_urdf(urdf_path: Path):
    root = ET.parse(urdf_path).getroot()
    links: dict[str, list[Collision]] = {}

    for link_el in root.findall("link"):
        collisions = []
        for col_el in link_el.findall("collision"):
            origin = col_el.find("origin")
            pos = np.zeros(3)
            rpy = np.zeros(3)
            if origin is not None:
                pos = np.array([float(x) for x in origin.get("xyz", "0 0 0").split()])
                rpy = np.array([float(x) for x in origin.get("rpy", "0 0 0").split()])
            quat = np.array(p.getQuaternionFromEuler(rpy.tolist()))

            geo = col_el.find("geometry")
            if geo is None:
                continue
            mesh_el = geo.find("mesh")
            box_el = geo.find("box")
            if mesh_el is not None:
                filename = (urdf_path.parent / mesh_el.get("filename")).resolve()
                scale = np.array([float(x) for x in mesh_el.get("scale", "1 1 1").split()])
                collisions.append(Collision("mesh", filename, None, pos, quat, scale))
            elif box_el is not None:
                half = np.array([float(x) for x in box_el.get("size").split()]) / 2.0
                collisions.append(Collision("box", None, half, pos, quat, np.ones(3)))
        links[link_el.get("name")] = collisions

    joints: list[Joint] = []
    for j_el in root.findall("joint"):
        limit_el = j_el.find("limit")
        lower, upper = -math.inf, math.inf
        if limit_el is not None:
            lower = float(limit_el.get("lower", 0.0))
            upper = float(limit_el.get("upper", 0.0))
        mimic_el = j_el.find("mimic")
        mimic = None
        if mimic_el is not None:
            mimic = (
                mimic_el.get("joint"),
                float(mimic_el.get("multiplier", 1.0)),
                float(mimic_el.get("offset", 0.0)),
            )
        joints.append(
            Joint(
                j_el.get("name"),
                j_el.get("type"),
                j_el.find("parent").get("link"),
                j_el.find("child").get("link"),
                lower,
                upper,
                mimic,
            )
        )

    return links, joints


# ---------------------------------------------------------------------------
# 凸分解：凹 -> V-HACD，凸 -> 直接取凸包
# ---------------------------------------------------------------------------
def parse_vhacd_objects(path: Path) -> list[np.ndarray]:
    """解析 V-HACD 输出 OBJ 中的每个 'o ...' 块，返回各块的凸包顶点。"""
    blocks, cur = [], []
    for line in open(path, "r", errors="ignore"):
        if line.startswith("o "):
            if len(cur) >= 4:
                blocks.append(cur)
            cur = []
        elif line.startswith("v "):
            cur.append([float(x) for x in line.split()[1:4]])
    if len(cur) >= 4:
        blocks.append(cur)

    pieces = []
    for block in blocks:
        try:
            pieces.append(hull_vertices(np.asarray(block, dtype=np.float64)))
        except Exception:
            continue
    return pieces


def decompose_triangles(tris: np.ndarray, key: str, cache_dir: Path, args):
    """返回 (凸块顶点列表, 说明)；顶点都在 link 坐标系下。"""
    tris = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    hull_pts = hull_vertices(tris.reshape(-1, 3))
    hull_vol = float(ConvexHull(hull_pts).volume)
    ratio = min(mesh_volume(tris) / hull_vol, 1.0) if hull_vol > 0 else 1.0

    if ratio >= args.convex_ratio:
        return [hull_pts], f"already convex (ratio={ratio:.3f})"

    obj_in = cache_dir / f"{key}.obj"
    obj_out = cache_dir / f"{key}_vhacd.obj"
    if args.recompute or not obj_in.exists():
        write_obj(tris, obj_in)
    if args.recompute or not obj_out.exists():
        with quiet_native_stdout(not args.vhacd_verbose):
            p.vhacd(
                str(obj_in),
                str(obj_out),
                str(cache_dir / f"{key}_vhacd.log"),
                concavity=args.vhacd_concavity,
                resolution=args.vhacd_resolution,
                depth=args.vhacd_depth,
                planeDownsampling=args.vhacd_plane_downsampling,
                convexhullDownsampling=args.vhacd_hull_downsampling,
                maxNumVerticesPerCH=args.max_vertices_per_ch,
                minVolumePerCH=args.vhacd_min_volume,
            )

    pieces = parse_vhacd_objects(obj_out) if obj_out.exists() else []
    if not pieces:
        return [hull_pts], f"V-HACD failed -> hull fallback (ratio={ratio:.3f})"
    return pieces, f"V-HACD {len(pieces)} pcs (ratio={ratio:.3f})"


@dataclass
class Piece:
    label: str
    link: str
    link_index: int | None        # -1 = base_link, None = 固定不动的障碍物
    obj: Path
    body: int = -1


def build_pieces(links, link_names, link_index, args, cache_dir: Path):
    """把每个 link 的 collision 全部拆成凸块（link 坐标系），返回 (pieces, 报告)。"""
    pieces_dir = cache_dir / "pieces"
    pieces_dir.mkdir(parents=True, exist_ok=True)
    pieces: list[Piece] = []
    report: list[tuple[str, str, str]] = []

    for lname in link_names:
        collisions = links.get(lname, [])
        if not collisions:
            continue
        start = len(pieces)

        for ci, col in enumerate(collisions):
            rot = quat_matrix(col.quat)
            if col.kind == "box":
                hx, hy, hz = col.half_extents
                corners = np.array(
                    [[sx * hx, sy * hy, sz * hz]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
                    dtype=np.float64,
                )
                obj = pieces_dir / f"{lname}__c{ci}_p0.obj"
                write_convex_obj(corners @ rot.T + col.pos, obj)
                pieces.append(Piece("", lname, link_index[lname], obj))
                report.append((lname, f"box {np.round(col.half_extents * 2, 3).tolist()}", "1 pcs (box)"))
                continue

            tris = mesh_triangles(col.path) * col.scale
            tris = tris @ rot.T + col.pos
            key = (
                f"{col.path.stem}_c{args.vhacd_concavity:g}_r{args.vhacd_resolution}"
                f"_d{args.vhacd_depth}_v{args.max_vertices_per_ch}"
            )
            verts_list, how = decompose_triangles(tris, key, cache_dir, args)
            for k, verts in enumerate(verts_list):
                obj = pieces_dir / f"{lname}__c{ci}_p{k}.obj"
                write_convex_obj(verts, obj)
                pieces.append(Piece("", lname, link_index[lname], obj))
            report.append((lname, col.path.name, how))

        for k in range(start, len(pieces)):
            pieces[k].label = f"{pieces[k].link}#p{k - start}"

    return pieces, report


def create_piece_bodies(pieces: list[Piece]) -> None:
    """每个凸块一个独立的静态刚体；collisionMargin=0 保证 getClosestPoints 精确。"""
    for pc in pieces:
        shape = p.createCollisionShape(p.GEOM_MESH, fileName=str(pc.obj))
        pc.body = p.createMultiBody(0, shape)
        p.changeDynamics(pc.body, -1, collisionMargin=0.0)


# ---------------------------------------------------------------------------
# 规则运动
# ---------------------------------------------------------------------------
# 末端执行器上"自己会动"的关节名关键字：夹爪手指/指节、电主轴夹头、铣刀轴……
EE_JOINT_KEYWORDS = ("finger", "knuckle", "spindle", "collet", "chuck", "mill", "cutter", "tool")


def is_gripper_joint(name: str) -> bool:
    """末端执行器自己的关节（不算"手臂关节"）。

    本仓库默认的电主轴 + 铣刀全是 fixed 关节，一个都匹配不到；但换成带夹头/换刀轴
    的模型时，这些关节绝不能混进手臂正弦摆动（否则 --home 的 6 个值就对不上了）。
    """
    low = name.lower()
    return any(k in low for k in EE_JOINT_KEYWORDS)


def link_chain(joints, root: str) -> list[str]:
    """root 之后（下游）所有 link 的名字，按 BFS 顺序；用来旁白末端工具链。"""
    children: dict[str, list[str]] = {}
    for j in joints:
        children.setdefault(j.parent, []).append(j.child)
    out: list[str] = []
    queue = [root]
    while queue:
        for child in children.get(queue.pop(0), []):
            if child not in out:
                out.append(child)
                queue.append(child)
    return out


class Motion:
    """q_k(t) = home_k + amp * sin(2*pi*freq*t + k*phase_step)，夹爪按 mimic 联动。"""

    def __init__(self, joints, args):
        self.by_name = {j.name: j for j in joints}
        self.arm = [j for j in joints
                    if j.type in ("revolute", "continuous") and not is_gripper_joint(j.name)]
        if not self.arm:
            raise RuntimeError("URDF 中找不到可动的手臂关节")

        home = list(args.home)
        if len(home) != len(self.arm):
            log(f"[warn] --home 给了 {len(home)} 个值，但手臂有 {len(self.arm)} 个关节，改用关节中值")
            home = [0.5 * (j.lower + j.upper) if math.isfinite(j.lower) and math.isfinite(j.upper) else 0.0
                    for j in self.arm]
        self.home = np.asarray(home, dtype=np.float64)
        self.amp = args.amp
        self.freq = args.freq
        self.phase = math.radians(args.phase_step)
        self.gripper = args.gripper

        self.finger = None
        if "finger_joint" in self.by_name:
            self.finger = "finger_joint"
        else:
            for j in joints:
                if j.type in ("revolute", "continuous") and is_gripper_joint(j.name) and j.mimic is None:
                    self.finger = j.name
                    break

        self.mimic = {j.name: j.mimic for j in joints if j.mimic is not None}

    def sample(self, t: float) -> dict[str, float]:
        cfg: dict[str, float] = {}
        for k, j in enumerate(self.arm):
            val = self.home[k] + self.amp * math.sin(2.0 * math.pi * self.freq * t + k * self.phase)
            if math.isfinite(j.lower) and math.isfinite(j.upper):
                val = min(max(val, j.lower), j.upper)
            cfg[j.name] = float(val)

        if self.finger is not None:
            fj = self.by_name[self.finger]
            lo, hi = fj.lower, fj.upper
            if not (math.isfinite(lo) and math.isfinite(hi)):
                lo, hi = 0.0, 0.7
            if self.gripper == "cycle":
                frac = 0.5 * (1.0 - math.cos(2.0 * math.pi * self.freq * t))
                cfg[self.finger] = float(lo + frac * (hi - lo))
            elif self.gripper == "closed":
                cfg[self.finger] = float(hi)
            else:
                cfg[self.finger] = float(lo)

        for child, (parent, mult, offset) in self.mimic.items():
            if parent in cfg:
                cfg[child] = float(mult * cfg[parent] + offset)
        return cfg


# ---------------------------------------------------------------------------
# 位姿 / 距离矩阵
# ---------------------------------------------------------------------------
def apply_config(robot, cfg: dict[str, float], joint_index: dict[str, int], pieces: list[Piece]) -> None:
    if not connected():
        raise p.error("PyBullet connection lost (GUI window closed?)")
    for name, value in cfg.items():
        p.resetJointState(robot, joint_index[name], float(value))

    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    link_poses: dict[int, tuple] = {}
    for pc in pieces:
        if pc.link_index is None:          # 固定障碍物，不动
            continue
        if pc.link_index == -1:
            pos, orn = base_pos, base_orn
        else:
            if pc.link_index not in link_poses:
                st = p.getLinkState(robot, pc.link_index, computeForwardKinematics=True)
                link_poses[pc.link_index] = (st[4], st[5])   # worldLinkFramePosition / Orientation
            pos, orn = link_poses[pc.link_index]
        p.resetBasePositionAndOrientation(pc.body, pos, orn)

    p.performCollisionDetection()


def distance_to_obstacle(robot_pieces: list[Piece], obstacle_pieces: list[Piece],
                        max_distance: float) -> tuple[np.ndarray, int]:
    """P[i][k] = 机器人凸块 i 到障碍物凸块 k 的最小距离。

    只测 "机器人 vs 障碍物"：机器人自身各连杆的碰撞体在关节/夹爪处本来就互相重叠，
    那类距离没有意义，故不再计算。返回值全部 >= 0：真穿透时截断为 0 并计数返回；
    没有接触且距离超过 max_distance 时记为 max_distance。
    """
    P = np.full((len(robot_pieces), len(obstacle_pieces)), float(max_distance), dtype=np.float64)
    n_clamped = 0
    for i, pc in enumerate(robot_pieces):
        for k, ob in enumerate(obstacle_pieces):
            contacts = p.getClosestPoints(pc.body, ob.body, max_distance)
            if not contacts:
                continue
            d = min(c[8] for c in contacts)
            if d < 0.0:
                n_clamped += 1
                d = 0.0
            P[i, k] = d

    return P, n_clamped


def link_index_groups(link_of: list[str], link_order: list[str]) -> list[np.ndarray]:
    """每个连杆包含的凸块下标（用于按连杆取最小距离）。"""
    return [np.array([i for i, name in enumerate(link_of) if name == ln], dtype=np.intp)
            for ln in link_order]


# ---------------------------------------------------------------------------
# 场景：立体图形（障碍物）与地面
# ---------------------------------------------------------------------------
def add_ground() -> None:
    vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[2.0, 2.0, 0.005], rgbaColor=[0.92, 0.92, 0.92, 1.0]
    )
    p.createMultiBody(0, -1, vis, basePosition=[0.0, 0.0, -0.005])


def add_obstacle(args) -> Piece:
    """在场景中放一个立体障碍物（长方块）；box 本身是凸的，算 1 个凸块。"""
    half = (np.asarray(args.obstacle_size, dtype=np.float64) / 2.0).tolist()
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.85, 0.55, 0.15, 1.0])
    body = p.createMultiBody(
        0,
        shape,
        vis,
        basePosition=list(args.obstacle_pos),
        baseOrientation=p.getQuaternionFromEuler(list(args.obstacle_rpy)),
    )
    p.changeDynamics(body, -1, collisionMargin=0.0)
    return Piece("obstacle#p0", "obstacle", None, Path(""), body)


def write_piece_csv(path: Path, rows: list[tuple]) -> None:
    """index,piece,link,d_t000,min_over_samples,argmin_t"""
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["index", "piece", "link", "d_t000", "min_over_samples", "argmin_t"])
        for idx, label, link, d0, dmin, tmin in rows:
            writer.writerow([idx, label, link, "%.6f" % d0, "%.6f" % dmin, "%.4f" % tmin])


def write_samples_csv(path: Path, labels: list[str], link_of: list[str],
                      d: np.ndarray, t_axis: np.ndarray) -> None:
    """行 = 凸块，列 = 各采样时刻的最小距离。"""
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["piece", "link"] + ["t=%.3f" % t for t in t_axis])
        for label, link, row in zip(labels, link_of, d):
            writer.writerow([label, link] + ["%.4f" % v for v in row])


def write_link_csv(path: Path, rows: list[tuple]) -> None:
    """link,n_pieces,d_t000,min_over_samples,argmin_t,closest_piece"""
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["link", "n_pieces", "d_t000", "min_over_samples", "argmin_t", "closest_piece"])
        for link, n, d0, dmin, tmin, piece in rows:
            writer.writerow([link, n, "%.6f" % d0, "%.6f" % dmin, "%.4f" % tmin, piece])


def print_link_table(rows: list[tuple], title: str) -> None:
    """rows: (link, n_pieces, d_t000, min_over_samples, argmin_t, closest_piece)"""
    log("-" * 92)
    log(title)
    log(f"  {'link':<26}{'pcs':>4}{'d(t=0) [m]':>13}{'min [m]':>11}{'t_min':>8}  closest piece")
    for link, n, d0, dmin, tmin, piece in rows:
        log(f"  {link:<26}{n:>4}{d0:>13.4f}{dmin:>11.4f}{tmin:>8.3f}  {piece}")
    if rows:
        best = min(rows, key=lambda r: r[3])
        log(f"  -> 全程最近: {best[0]}  {best[3]:.4f} m @ t={best[4]:.3f}（{best[5]}）")
    log("-" * 92)


# ---------------------------------------------------------------------------
# 命令行 / 主流程
# ---------------------------------------------------------------------------
def parse_floats(text: str) -> list[float]:
    return [float(x) for x in str(text).replace(";", ",").replace(" ", ",").split(",") if x.strip()]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="计算 UR5 + 末端执行器（默认电主轴 + 铣刀，也可换 Robotiq 夹爪）各连杆凸块"
                    "之间的最小距离矩阵（含规则运动与立体障碍物）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--urdf", default=str(DEFAULT_URDF),
                    help=f"URDF 路径；默认电主轴 + 铣刀，夹爪模型用 {GRIPPER_URDF}")
    ap.add_argument("--out-dir", default=str(ROOT / "distance_matrix_out"), help="输出目录")
    ap.add_argument("--samples", type=int, default=30, help="一个运动周期内的采样点数")
    ap.add_argument("--home", type=parse_floats, default="0,-1.2,1.2,-1.57,-1.57,0",
                    help="6 个手臂关节的基准角 (rad)")
    ap.add_argument("--amp", type=float, default=0.35, help="正弦幅值 (rad)")
    ap.add_argument("--freq", type=float, default=1.0, help="一个采样周期内的正弦周期数")
    ap.add_argument("--phase-step", type=float, default=60.0, help="相邻手臂关节的相位差 (deg)")
    ap.add_argument("--gripper", choices=["cycle", "open", "closed"], default="cycle",
                    help="夹爪运动方式（URDF 里没有夹爪关节时无效，例如电主轴）")
    ap.add_argument("--max-distance", type=float, default=5.0, help="测距上限，更远的距离记为该值 (m)")
    ap.add_argument("--convex-ratio", type=float, default=0.99,
                    help="mesh 体积/凸包体积 >= 该值时视为已经凸，不再 V-HACD")
    ap.add_argument("--vhacd-concavity", type=float, default=0.002, help="V-HACD 最大凹度")
    ap.add_argument("--vhacd-resolution", type=int, default=100000, help="V-HACD 体素分辨率")
    ap.add_argument("--vhacd-depth", type=int, default=20, help="V-HACD 最大分割层数")
    ap.add_argument("--vhacd-plane-downsampling", type=int, default=4, help="V-HACD 分割平面降采样")
    ap.add_argument("--vhacd-hull-downsampling", type=int, default=4, help="V-HACD 凸包降采样")
    ap.add_argument("--max-vertices-per-ch", type=int, default=64, help="每个凸块最大顶点数")
    ap.add_argument("--vhacd-min-volume", type=float, default=0.0001, help="V-HACD 最小凸块体积")
    ap.add_argument("--recompute", action="store_true", help="忽略缓存，重新跑 V-HACD")
    ap.add_argument("--vhacd-verbose", action="store_true", help="显示 V-HACD 自己的进度输出")
    ap.add_argument("--obstacle-size", type=parse_floats, default="0.12,0.12,0.30", help="障碍物长宽高 (m)")
    ap.add_argument("--obstacle-pos", type=parse_floats, default="0.45,-0.30,0.15", help="障碍物中心 (m)")
    ap.add_argument("--obstacle-rpy", type=parse_floats, default="0,0,0", help="障碍物姿态 (rad)")
    ap.add_argument("--no-ground", action="store_true", help="不画地面")
    ap.add_argument("--gui", action="store_true", help="打开 GUI 看机械臂运动")
    renderer = ap.add_mutually_exclusive_group()
    renderer.add_argument("--opengl2", action="store_true",
                          help="GUI 用 --opengl2 渲染器（默认；单独写也会隐式打开 GUI）")
    renderer.add_argument("--gl3", action="store_true",
                          help="GUI 用 --gl3 渲染器（需要 GL3 插件；默认 --opengl2）")
    ap.add_argument("--fps", type=float, default=20.0, help="GUI 回放帧率")
    ap.add_argument("--hold", action="store_true", help="结束后一直保持 GUI 窗口（Ctrl+C 退出）")
    ap.add_argument("--hold-seconds", type=float, default=2.0,
                    help="不加 --hold 时，运动结束后窗口停留的秒数")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    urdf = Path(args.urdf).expanduser().resolve()
    if not urdf.is_file():
        log(f"[error] URDF 不存在: {urdf}")
        return 2

    out_dir = Path(args.out_dir).expanduser().resolve()
    cache_dir = out_dir / "cache"
    cache_dir.mkdir(parents=True, exist_ok=True)

    links, joints = parse_urdf(urdf)

    if args.opengl2 or args.gl3:
        args.gui = True  # 显式指定渲染器时默认就是想看窗口

    # 无 X11 显示时 p.connect(GUI) 会直接把进程带走（不会返回 <0），所以先判断。
    if args.gui and sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        log("[warn] 检测不到 $DISPLAY，GUI 不可用，改用 DIRECT（无窗口）")
        args.gui = False
        args.hold = False

    options = "--gl3" if args.gl3 else "--opengl2"
    if args.gui:
        cid = p.connect(p.GUI, options=options)
        if cid < 0:
            alt = "--opengl2" if options == "--gl3" else "--gl3"
            log(f"[warn] GUI({options}) 连接失败，改用 {alt} 再试一次")
            cid = p.connect(p.GUI, options=alt)
        if cid < 0:
            log("[warn] GUI 连接失败（检查 $DISPLAY / OpenGL 驱动），退回 DIRECT（无窗口）")
            args.gui = False
            args.hold = False
            cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.DIRECT)
    if cid < 0:
        log("[error] 无法连接 PyBullet 物理服务器")
        return 3

    try:
        robot = p.loadURDF(str(urdf), useFixedBase=True)

        # link 顺序 = PyBullet 的 link index 顺序（base_link 为 -1）
        link_names = [p.getBodyInfo(robot)[0].decode()]
        link_index = {link_names[0]: -1}
        joint_index: dict[str, int] = {}
        for i in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, i)
            jname = info[1].decode()
            child = info[12].decode()
            joint_index[jname] = i
            link_names.append(child)
            link_index[child] = i

        log("=" * 100)
        log(f"URDF     : {urdf}")
        log(f"PyBullet : {p.getAPIVersion()}  模式: {'GUI' if args.gui else 'DIRECT'}")

        # 1) 凸分解
        pieces, report = build_pieces(links, link_names, link_index, args, cache_dir)
        if not pieces:
            log("[error] 没有解析到任何 collision 凸块")
            return 4

        n_collisions = sum(len(v) for v in links.values())
        log("")
        log(f"[凸分解] {n_collisions} 个 collision 元素 -> {len(pieces)} 个凸块")
        for lname, src, how in report:
            log(f"  {lname:<26} {src:<40} {how}")

        # 2) 凸块刚体 + 场景
        create_piece_bodies(pieces)
        if not args.no_ground:
            add_ground()
        obstacle = add_obstacle(args)
        obstacles = [obstacle]

        labels = [pc.label for pc in pieces]
        n_pieces = len(pieces)
        link_of = [pc.link for pc in pieces]
        link_order = list(dict.fromkeys(link_of))            # 保持 URDF 里的连杆顺序
        groups = link_index_groups(link_of, link_order)
        counts = {ln: len(idx) for ln, idx in zip(link_order, groups)}

        size_cm = np.round(np.asarray(args.obstacle_size, dtype=np.float64) * 100.0, 1).tolist()
        log("")
        log(f"[场景] 机器人凸块 {n_pieces} 个；障碍物 {size_cm} cm @ "
            f"{np.round(np.asarray(args.obstacle_pos, dtype=np.float64), 3).tolist()} m"
            f"（{len(obstacles)} 个凸块）")
        log("[测距] 只计算 机器人凸块 <-> 障碍物；机器人本体各连杆之间不测距")
        log("       连杆/凸块数: " + ", ".join(f"{k}:{v}" for k, v in counts.items()))

        # 3) 规则运动
        motion = Motion(joints, args)
        log("")
        log("[运动] 手臂关节: " + ", ".join(j.name for j in motion.arm))
        log(f"       q_k(t) = home_k + {args.amp} * sin(2*pi*{args.freq}*t + k*{args.phase_step}deg), "
            f"t = 0..1 均匀取 {max(1, args.samples)} 点")
        ee_links = link_chain(joints, "ee_link")
        if ee_links:
            n_ee = sum(counts.get(ln, 0) for ln in ee_links)
            log(f"[末端] ee_link 下游（固定工具）: {', '.join(ee_links)}（{n_ee} 个凸块）")
        if motion.finger:
            log(f"       夹爪: {args.gripper}（{motion.finger} + {len(motion.mimic)} 个 mimic 关节联动）")
        else:
            log(f"       末端没有可开合的夹爪关节，--gripper {args.gripper} 不生效"
                f"（要算夹爪开合请 --urdf {GRIPPER_URDF.name}）")

        if args.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(1.8, 45.0, -30.0, [0.35, 0.0, 0.35])
            log("[GUI] 采样期间请勿关闭窗口或按 ESC：Bullet 会直接退出渲染服务，采样随之中断")

        samples = max(1, args.samples)
        piece_d = np.zeros((samples, n_pieces), dtype=np.float64)        # 凸块 -> 障碍物
        link_d = np.zeros((samples, len(link_order)), dtype=np.float64)  # 连杆 -> 障碍物
        q_names = [n for n in sorted(joint_index, key=lambda k: joint_index[k])
                   if p.getJointInfo(robot, joint_index[n])[2] != p.JOINT_FIXED]
        q_records = np.zeros((samples, len(q_names)), dtype=np.float64)
        ee_poses = np.zeros((samples, 7), dtype=np.float64)
        ee_index = link_index.get("ee_link", -1)
        n_clamped_total = 0

        log("")
        t_start = time.time()
        n_done = 0
        try:
            for s in range(samples):
                t = s / samples
                cfg = motion.sample(t)
                apply_config(robot, cfg, joint_index, pieces)
                P, n_clamped = distance_to_obstacle(pieces, obstacles, args.max_distance)
                n_clamped_total += n_clamped
                piece_d[s] = P.min(axis=1)
                link_d[s] = [float(P[idx].min()) for idx in groups]
                q_records[s] = [cfg.get(name, 0.0) for name in q_names]

                if ee_index >= 0:
                    st = p.getLinkState(robot, ee_index, computeForwardKinematics=True)
                    ee_poses[s] = [*st[4], *st[5]]

                k = int(np.argmin(link_d[s]))
                log(f"[{s + 1:3d}/{samples}] t={t:5.3f}  最近连杆: "
                    f"{link_order[k]:<26} {link_d[s, k]:.4f} m")
                if n_clamped:
                    log(f"            [warn] {n_clamped} 个凸块与障碍物判定为穿透，距离已置 0")

                n_done = s + 1
                if args.gui:
                    time.sleep(1.0 / max(args.fps, 1e-3))
        except p.error as exc:
            # GUI 模式下按 ESC 或直接关窗口，Bullet 的 ExampleBrowser 会整个退出
            # （先打印 "btShutDownExampleBrowser stopping threads" 之类日志），
            # 之后 getLinkState / getClosestPoints 都会失败。这里优雅收尾。
            log("")
            log(f"[warn] PyBullet 连接已断开：{exc}")
            log("       常见原因：GUI 窗口被关闭 / 按了 ESC —— Bullet 会直接关掉渲染服务。"
                "计算阶段请勿关闭窗口；只想看结果就别加 --gui。")
            log(f"       已完成 {n_done}/{samples} 个采样点，只输出这部分结果。")
            if n_done == 0:
                return 3

        n_out = n_done if n_done < samples else samples
        if n_out < samples:
            piece_d = piece_d[:n_out]
            link_d = link_d[:n_out]
            q_records = q_records[:n_out]
            ee_poses = ee_poses[:n_out]

        extra = (f"；[warn] 累计 {n_clamped_total} 次穿透判定已截断为 0"
                 if n_clamped_total else "")
        log(f"[耗时] {time.time() - t_start:.2f} s / {n_out} 个采样点{extra}")
        if n_clamped_total:
            log("[warn] 出现负距离说明机器人穿进了障碍物，请检查 --obstacle-pos / --home / --amp")


        # 4) 输出
        t_axis = np.arange(n_out) / float(samples)   # 若中途断开，仍按原采样网格取时刻
        argmin_link = link_d.argmin(axis=0)
        min_link = link_d.min(axis=0)
        link_rows = []
        for k, ln in enumerate(link_order):
            ts = int(argmin_link[k])
            best_local = int(np.argmin(piece_d[ts, groups[k]]))
            link_rows.append((
                ln, len(groups[k]),
                float(link_d[0, k]), float(min_link[k]), float(t_axis[ts]),
                labels[int(groups[k][best_local])],
            ))

        argmin_piece = piece_d.argmin(axis=0)
        min_piece = piece_d.min(axis=0)
        piece_rows = [
            (i, labels[i], link_of[i],
             float(piece_d[0, i]), float(min_piece[i]), float(t_axis[int(argmin_piece[i])]))
            for i in range(n_pieces)
        ]

        with open(out_dir / "pieces.csv", "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["index", "label", "link", "convex_piece_obj"])
            for idx, pc in enumerate(pieces):
                writer.writerow([idx, pc.label, pc.link, str(pc.obj)])

        write_piece_csv(out_dir / "piece_distance_to_obstacle.csv", piece_rows)
        write_link_csv(out_dir / "link_distance_to_obstacle.csv", link_rows)
        write_samples_csv(out_dir / "piece_distances_all_samples.csv", labels, link_of,
                          piece_d.T, t_axis)
        np.savez_compressed(
            out_dir / "distances.npz",
            piece_distances=piece_d,          # (S, N) 每个凸块 -> 障碍物
            link_distances=link_d,            # (S, L) 每个连杆 -> 障碍物（凸块取 min）
            labels=np.array(labels),
            link_names=np.array(link_order),
            joint_names=np.array(q_names),
            joint_positions=q_records,
            sample_t=t_axis,
            ee_poses=ee_poses,
            obstacle_size=np.asarray(args.obstacle_size, dtype=np.float64),
            obstacle_pos=np.asarray(args.obstacle_pos, dtype=np.float64),
            obstacle_rpy=np.asarray(args.obstacle_rpy, dtype=np.float64),
        )

        log("")
        log(f"[输出目录] {out_dir}")
        log("  pieces.csv                          凸块索引 / 标签 / 所属连杆")
        log("  piece_distance_to_obstacle.csv      每个凸块 -> 障碍物：t=0 距离 / 全程最小 / 时刻")
        log("  link_distance_to_obstacle.csv       每个连杆 -> 障碍物：t=0 距离 / 全程最小 / 时刻")
        log("  piece_distances_all_samples.csv     行=凸块、列=各采样时刻的最小距离")
        log("  distances.npz                       piece_distances (S,N) / link_distances (S,L) + ...")
        log("")
        log("[采样点 0] t=0.000 关节角: "
            + ", ".join(f"{n}={v:+.3f}" for n, v in zip(q_names, q_records[0])))
        print_link_table(link_rows, "[各连杆 -> 障碍物 的最小距离]")
        log(f"[全程最近凸块] {labels[int(np.argmin(min_piece))]}  {float(min_piece.min()):.4f} m")

        if args.gui and connected():
            if args.hold:
                log("[GUI] 运动结束，窗口保持打开（Ctrl+C 关闭），可用鼠标拖动视角")
                try:
                    while connected():
                        time.sleep(0.2)
                except KeyboardInterrupt:
                    pass
            elif args.hold_seconds > 0:
                log(f"[GUI] 运动结束，窗口停留 {args.hold_seconds:.0f} s"
                    f"（--hold 一直保持，--fps {args.fps:.0f} → 更小更慢）")
                time.sleep(args.hold_seconds)

        return 0
    finally:
        try:
            if connected():
                p.disconnect()
        except p.error:
            pass       # 服务器已经自己退出了（GUI 窗口被关闭），无需再 disconnect


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[中断] 用户按了 Ctrl+C，已退出", flush=True)
        raise SystemExit(130)

