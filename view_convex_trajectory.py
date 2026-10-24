#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""凸包运动轨迹查看器：把机器人拆成凸块（V-HACD），沿轨迹摆动并画出扫过的轨迹。

和 link_piece_distance_matrix.py 的分工
---------------------------------------
后者负责 "测距"（凸块 <-> 障碍物），只在 --gui 时顺便让你看一眼机器人在动；
本脚本专门用来 "看"：

* 凸块是纯视觉刚体（baseCollisionShapeIndex=-1，不参与碰撞、不测距），帧率高；
* 凸包按连杆上色，原始 URDF 网格可做半透明底图（--robot-mode ghost），
  方便换机器人后检查 V-HACD 的分块是否贴合原网格；
* 一次把整个周期的若干时刻 "定影" 成半透明残影（--ghosts），
  再画连杆原点 / 凸块质心的轨迹线（--trail），一眼能看出扫过的空间；
* 整机会自动抬高（--clearance），保证正弦摆动的最低点不会穿到地面以下，
  相机也按整段扫掠范围自动取景，所以整个摆动都看得见；抬高后底座会悬空，
  默认画一个灰色线框支架到地面（--no-pedestal 去掉），想再抬高加 --lift；
* 静止不动的凸块（基座、绕自身轴转的圆柱…）不会生成残影，否则几份半透明
  残影叠在同一个位置，会糊成一根看着很奇怪的"柱子"；
* 夹爪"张开 / 闭合"对应哪个关节值，运行时用指尖间距实测标定（不同 URDF 的
  正负约定不一样），默认从张开开始，不是一上来就合拢；
* --labels 在每个连杆上标出名字，--hide link1,link2 可以把指定连杆的凸块藏掉，
  用来确认"画面里这块到底是什么"；
* 关节数量、限位、mimic、夹爪全部从 URDF 读，换机器人只改 --urdf。

复用关系
--------
import link_piece_distance_matrix，复用 parse_urdf() / build_pieces() /
is_gripper_joint() / log() / connected() / parse_floats()，凸分解结果与它共用
同一个缓存目录（默认 distance_matrix_out/cache），所以两边的凸块完全一致，
不会重复跑 V-HACD（要重算就加 --recompute）。

任意 URDF 的适配规则
--------------------
* 可动关节 = URDF 里 type 为 revolute / continuous / prismatic 的关节；
* "夹爪/末端关节" 自动识别：是某条 <mimic> 的父关节，或名字里带 finger / knuckle；
  它不参与正弦摆动，由 --gripper cycle|open|closed 驱动，mimic 关节按
  multiplier/offset 联动（支持 mimic 套 mimic）；
* 基准位姿 --home 缺省为每个关节的 clamp(0, lower, upper)，不是写死的 UR5 角度；
* 正弦幅值默认 --amp（rad，prismatic 关节是 m），也可以 --amp-frac 0.4 让每个
  关节按自己行程的 40% 自动取幅值，这样不同机器人都不用调参数。

轨迹来源
--------
1. 默认：q_k(t) = home_k + amp_k * sin(2*pi*freq*t + k*phase_step)，t = 0..1；
2. --traj xxx.npz：读 joint_names / joint_positions（可选 sample_t），
   直接兼容本仓库 distance_matrix_out/distances.npz，可以回放测距时的运动；
3. --traj xxx.csv：表头是关节名，可选一列 t / time / sample_t（缺省按 0..1 均匀）；
4. --save-traj out.csv：把本次实际用的轨迹导出，之后用 --traj 原样回放。

用法示例
--------
    python view_convex_trajectory.py                              # UR5 + 正弦摆动
    python view_convex_trajectory.py --ghosts 8 --trail piece      # 残影 + 凸块质心轨迹
    python view_convex_trajectory.py --urdf ../other_robot.urdf --amp-frac 0.4
    python view_convex_trajectory.py --traj distance_matrix_out/distances.npz
    python view_convex_trajectory.py --headless --save-traj /tmp/traj.csv   # 无窗口自检
    python view_convex_trajectory.py --no-ground --robot-mode hide --cycle-seconds 20

GUI 操作
--------
    鼠标左键拖动 = 转视角，滚轮 = 缩放，Ctrl+拖动 = 平移；
    SPACE = 暂停/继续；q = 退出（ESC 也行，但 ESC 会让 Bullet 直接关掉渲染服务）。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import link_piece_distance_matrix as lpdm  # noqa: E402  复用凸分解 / 日志 / GUI 逻辑

log = lpdm.log
DEFAULT_URDF = lpdm.DEFAULT_URDF

# 每个连杆一个固定颜色（按 URDF 里的出现顺序取，换机器人也稳定）
PALETTE: list[tuple[float, float, float]] = [
    (0.91, 0.30, 0.24), (0.20, 0.60, 0.86), (0.30, 0.75, 0.35), (0.95, 0.65, 0.15),
    (0.61, 0.40, 0.80), (0.15, 0.75, 0.75), (0.85, 0.45, 0.70), (0.55, 0.55, 0.22),
    (0.35, 0.50, 0.90), (0.80, 0.35, 0.15), (0.25, 0.65, 0.50), (0.72, 0.72, 0.78),
]

EE_CANDIDATES = ("ee_link", "tool0", "tcp", "flange", "end_effector_link",
                 "gripper_link", "tool_frame", "palm")

# 自动取景的默认视角（distance,yaw,pitch,target_x,target_y,target_z）
CAMERA_DEFAULT = [1.8, 50.0, -22.0, 0.3, 0.0, 0.3]


def link_colors(link_order: list[str]) -> dict[str, tuple[float, float, float]]:
    return {name: PALETTE[i % len(PALETTE)] for i, name in enumerate(link_order)}


def joint_limits(joint) -> tuple[float, float] | None:
    """有限且合法的 (lower, upper)；continuous / 没写 limit 的关节返回 None。"""
    lo, hi = float(joint.lower), float(joint.upper)
    if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi:
        return None
    return lo, hi


def zero_pose(joint) -> float:
    """该关节的 "零位"：把 0 夹进 [lower, upper]（continuous 关节就是 0）。"""
    lim = joint_limits(joint)
    return 0.0 if lim is None else float(min(max(0.0, lim[0]), lim[1]))


def rot_matrix(quat) -> np.ndarray:
    return np.asarray(p.getMatrixFromQuaternion([float(x) for x in quat]),
                      dtype=np.float64).reshape(3, 3)


def joint_children(robot) -> dict[int, list[int]]:
    """关节的父子关系（-1 = 基座），用来划出夹爪子树。"""
    children: dict[int, list[int]] = {}
    for i in range(p.getNumJoints(robot)):
        children.setdefault(int(p.getJointInfo(robot, i)[16]), []).append(i)
    return children


def gripper_span(robot, root_joints: list[int]) -> float:
    """夹爪张开度：夹爪这几串关节最末端连杆（指尖）之间的最大距离。

    张开时左右两指分开 -> 变大，闭合时靠拢 -> 变小，和关节值正负约定无关。
    """
    if not root_joints:
        return 0.0
    children = joint_children(robot)
    stack = [j for r in root_joints for j in children.get(r, [])]
    joints: list[int] = []
    seen: set[int] = set()
    while stack:
        j = stack.pop()
        if j in seen:
            continue
        seen.add(j)
        joints.append(j)
        stack.extend(children.get(j, []))
    leaves = [j for j in joints if not children.get(j)]
    picks = leaves if len(leaves) >= 2 else joints
    if len(picks) < 2:
        return 0.0
    pts = [np.asarray(p.getLinkState(robot, j, computeForwardKinematics=True)[4],
                      dtype=np.float64) for j in picks]
    return float(max(np.linalg.norm(a - b)
                     for i, a in enumerate(pts) for b in pts[i + 1:]))


# ---------------------------------------------------------------------------
# 轨迹：程序正弦 / 外部轨迹回放
# ---------------------------------------------------------------------------
class Motion:
    """给出任意 t 的关节配置 cfg = {关节名: 值}。

    * 程序模式：可动关节（去掉夹爪）做正弦摆动，夹爪按 --gripper 动作；
    * 回放模式：按 --traj 的时刻线性插值，轨迹里没写的关节保持基准位姿。
    """

    def __init__(self, joints, args):
        self.joints = joints
        self.by_name = {j.name: j for j in joints}
        self.mimic = {j.name: j.mimic for j in joints if j.mimic is not None}
        mimic_parents = {m[0] for m in self.mimic.values()}

        movable = [j for j in joints if j.type in ("revolute", "continuous", "prismatic")]
        if not movable:
            raise RuntimeError("URDF 里没有可动关节（revolute / continuous / prismatic 都没有）")

        # 夹爪/末端关节：mimic 的父关节，或名字像手指
        self.leader: str | None = None
        for j in movable:
            if j.mimic is None and (j.name in mimic_parents or lpdm.is_gripper_joint(j.name)):
                self.leader = j.name
                break

        self.arm = [j for j in movable if j.mimic is None and j.name != self.leader]
        if not self.arm:                     # 只有夹爪的机构也能看
            self.arm = [j for j in movable if j.mimic is None]
            self.leader = None
        if not self.arm:
            raise RuntimeError("所有可动关节都被 mimic 驱动，没法单独摆姿势")

        self.gripper = args.gripper
        self.grip_open: float | None = None       # 标定后才知道（calibrate_gripper）
        self.grip_closed: float | None = None
        self.freq = float(args.freq)
        self.phase = math.radians(float(args.phase_step))
        self.home = self._home(args)
        self.amp = self._amp(args)
        self.names = [j.name for j in self.arm]

        self.traj_t: np.ndarray | None = None
        self.traj_q: np.ndarray | None = None
        self.traj_names: list[str] = []
        if args.traj:
            self._load_traj(Path(args.traj).expanduser())

    # -- 参数 -------------------------------------------------------------
    def _auto_home(self) -> np.ndarray:
        return np.asarray([zero_pose(j) for j in self.arm], dtype=np.float64)

    def _home(self, args) -> np.ndarray:
        values = list(args.home or [])
        if not values:
            return self._auto_home()
        if len(values) != len(self.arm):
            log(f"[warn] --home 给了 {len(values)} 个值，但可动关节（不含夹爪）有 "
                f"{len(self.arm)} 个，改用自动基准位姿 clamp(0, lower, upper)")
            return self._auto_home()
        return np.asarray(values, dtype=np.float64)

    def _amp(self, args) -> np.ndarray:
        amp = np.full(len(self.arm), float(args.amp), dtype=np.float64)
        if args.amp_frac > 0.0:
            for k, j in enumerate(self.arm):
                lim = joint_limits(j)
                if lim is not None:
                    amp[k] = float(args.amp_frac) * 0.5 * (lim[1] - lim[0])
            log(f"[info] --amp-frac {args.amp_frac:g} -> 各关节幅值 "
                + ", ".join(f"{j.name}={a:.3f}" for j, a in zip(self.arm, amp)))
        return amp

    # -- 外部轨迹 ---------------------------------------------------------
    def _load_traj(self, path: Path) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"轨迹文件不存在: {path}")
        suffix = path.suffix.lower()

        if suffix == ".npz":
            data = np.load(path, allow_pickle=False)
            for key in ("joint_positions", "joint_names"):
                if key not in data:
                    raise ValueError(f"{path} 缺少数组 {key}"
                                     "（distance_matrix_out/distances.npz 里就有）")
            q = np.asarray(data["joint_positions"], dtype=np.float64)
            names = [str(x) for x in data["joint_names"]]
            t = (np.asarray(data["sample_t"], dtype=np.float64)
                 if "sample_t" in data else np.linspace(0.0, 1.0, len(q)))
        elif suffix == ".csv":
            with open(path, "r", newline="", encoding="utf-8-sig") as fh:
                rows = [r for r in csv.reader(fh) if r and any(c.strip() for c in r)]
            if len(rows) < 2:
                raise ValueError(f"{path} 只有表头，没有数据行")
            header = [h.strip() for h in rows[0]]
            try:
                table = np.asarray([[float(x) for x in row] for row in rows[1:]], dtype=np.float64)
            except ValueError as exc:
                raise ValueError(f"{path} 有非数值单元格: {exc}") from exc
            time_col = next((header.index(c) for c in ("t", "time", "sample_t")
                             if c in header), None)
            if time_col is None:
                t = np.linspace(0.0, 1.0, len(table))
                names, q = header, table
            else:
                t = table[:, time_col]
                names = [h for i, h in enumerate(header) if i != time_col]
                q = np.delete(table, time_col, axis=1)
        else:
            raise ValueError(f"不认识的轨迹格式 {suffix}（支持 .npz / .csv）")

        if q.ndim != 2 or len(t) != q.shape[0] or len(names) != q.shape[1]:
            raise ValueError(f"{path} 轨迹形状不自洽: q={q.shape}, t={len(t)}, 关节名={len(names)}")
        order = np.argsort(np.asarray(t, dtype=np.float64))
        self.traj_t = np.asarray(t, dtype=np.float64)[order]
        self.traj_q = q[order]
        self.traj_names = names

        known = {j.name for j in self.joints}
        unknown = [n for n in names if n not in known]
        if unknown:
            log(f"[warn] 轨迹里有 URDF 中不存在的关节，已忽略: {', '.join(unknown[:6])}"
                + (" ..." if len(unknown) > 6 else ""))
        missing = [j.name for j in self.joints
                   if j.type != "fixed" and j.mimic is None and j.name not in names]
        if missing:
            log(f"[warn] 轨迹里没有这些关节，保持基准位姿: {', '.join(missing[:6])}"
                + (" ..." if len(missing) > 6 else ""))
        log(f"[轨迹] {path}  帧数 {len(self.traj_t)}  关节 {len(names)}  "
            f"t = {self.traj_t[0]:.3f} .. {self.traj_t[-1]:.3f}")

    def _traj_row(self, t: float) -> np.ndarray:
        ts, q = self.traj_t, self.traj_q
        assert ts is not None and q is not None
        if len(ts) == 1:
            return q[0]
        t = float(np.clip(t, ts[0], ts[-1]))
        i = int(np.searchsorted(ts, t, side="right") - 1)
        i = min(max(i, 0), len(ts) - 2)
        span = float(ts[i + 1] - ts[i])
        w = 0.0 if span <= 0.0 else (t - float(ts[i])) / span
        return (1.0 - w) * q[i] + w * q[i + 1]

    # -- 采样 -------------------------------------------------------------
    def _gripper_cfg(self, cfg: dict[str, float], t: float) -> None:
        """夹爪取值：张开 / 闭合两端由 calibrate_gripper() 标定，cycle 从 "张开" 开始。"""
        if self.leader is None:
            return
        open_v, closed_v = self._grip_limits()
        if self.gripper == "cycle":
            frac = 0.5 * (1.0 + math.cos(2.0 * math.pi * self.freq * t))
            cfg[self.leader] = float(closed_v + frac * (open_v - closed_v))
        elif self.gripper == "open":
            cfg[self.leader] = float(open_v)
        else:
            cfg[self.leader] = float(closed_v)

    def _grip_limits(self) -> tuple[float, float]:
        """(张开值, 闭合值)；还没标定时退回 (lower, upper)。"""
        if self.grip_open is not None and self.grip_closed is not None:
            return float(self.grip_open), float(self.grip_closed)
        lim = joint_limits(self.by_name[self.leader])
        return lim if lim is not None else (0.0, 0.7)

    def _gripper_roots(self, joint_index: dict[str, int]) -> list[int]:
        """leader 以及所有（间接）mimic 它的关节：夹爪真正会动的那几串关节。"""
        if self.leader is None:
            return []
        seen = {self.leader}
        changed = True
        while changed:
            changed = False
            for child, (parent, _mult, _offset) in self.mimic.items():
                if parent in seen and child not in seen:
                    seen.add(child)
                    changed = True
        return [joint_index[n] for n in seen if n in joint_index]

    def calibrate_gripper(self, robot, joint_index: dict[str, int]) -> None:
        """实测 "张开 / 闭合" 各是哪个关节值：指尖间距大的那一端算张开。

        不同 URDF 的正负约定完全不一样（本仓库的 Robotiq 140 恰好 0=张开、
        0.7=闭合），所以不能猜，直接摆两个极端量一下夹爪子树的张开度。
        """
        if self.leader is None:
            return
        lim = joint_limits(self.by_name[self.leader])
        if lim is None:
            log(f"[夹爪] {self.leader} 没有 limit，按 0..0.7 处理")
            return
        lo, hi = lim
        roots = self._gripper_roots(joint_index)
        spans: dict[float, float] = {}
        for value in (lo, hi, 0.5 * (lo + hi)):
            cfg = {self.leader: float(value)}
            self._fill_mimic(cfg)
            apply_config(robot, cfg, joint_index)
            spans[value] = gripper_span(robot, roots)
        self.grip_open, self.grip_closed = (lo, hi) if spans[lo] >= spans[hi] else (hi, lo)
        log(f"[夹爪] {self.leader}: 张开={self.grip_open:+.3f}"
            f"（指尖间距 {spans[self.grip_open]:.3f} m），"
            f"闭合={self.grip_closed:+.3f}（{spans[self.grip_closed]:.3f} m）")
        if self.traj_q is not None:
            log("       回放外部轨迹，夹爪值由轨迹文件决定（--gripper 不生效）")
        else:
            log("       起始位姿 t=0 -> "
                + ("张开" if self.gripper in ("cycle", "open") else "闭合"))

    def _fill_mimic(self, cfg: dict[str, float], only_missing: bool = False) -> None:
        for _ in range(max(1, len(self.mimic))):        # 支持 mimic 套 mimic
            changed = False
            for child, (parent, mult, offset) in self.mimic.items():
                if only_missing and child in cfg:
                    continue
                if parent in cfg:
                    value = float(mult * cfg[parent] + offset)
                    if cfg.get(child) != value:
                        cfg[child] = value
                        changed = True
            if not changed:
                break

    def sample(self, t: float) -> dict[str, float]:
        if self.traj_q is not None:
            cfg: dict[str, float] = {}
            for j in self.joints:                      # 先给所有可动关节一个确定值
                if j.type != "fixed" and j.mimic is None:
                    cfg[j.name] = zero_pose(j)
            for k, j in enumerate(self.arm):
                cfg[j.name] = float(self.home[k])
            cfg.update({name: float(v) for name, v in zip(self.traj_names, self._traj_row(t))})
            self._fill_mimic(cfg, only_missing=True)
            return cfg

        cfg = {}
        for k, j in enumerate(self.arm):
            value = float(self.home[k]) + float(self.amp[k]) * math.sin(
                2.0 * math.pi * self.freq * t + k * self.phase)
            lim = joint_limits(j)
            if lim is not None:
                value = min(max(value, lim[0]), lim[1])
            cfg[j.name] = float(value)
        self._gripper_cfg(cfg, t)
        self._fill_mimic(cfg)
        return cfg

    def times(self, samples: int) -> np.ndarray:
        """一个周期内要回放的时刻。"""
        if self.traj_q is not None:
            assert self.traj_t is not None
            if samples and samples != len(self.traj_t):
                return np.linspace(float(self.traj_t[0]), float(self.traj_t[-1]), samples)
            return self.traj_t.copy()
        return np.linspace(0.0, 1.0, max(1, int(samples)), endpoint=False)

    @property
    def periodic(self) -> bool:
        """程序生成的正弦运动是闭合周期，轨迹线可以首尾相连。"""
        return self.traj_q is None


# ---------------------------------------------------------------------------
# PyBullet 场景（全部是 "纯视觉" 物体，不测距、不碰撞）
# ---------------------------------------------------------------------------
def connect(gui: bool, gl3: bool) -> bool:
    """连接物理服务器，返回 GUI 是否真的可用（和主程序一样的降级逻辑）。"""
    if gui and sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        log("[warn] 检测不到 $DISPLAY / $WAYLAND_DISPLAY，GUI 不可用 -> 只做无窗口自检")
        gui = False
    if not gui:
        if p.connect(p.DIRECT) < 0:
            raise RuntimeError("无法连接 PyBullet 物理服务器")
        return False

    options = "--gl3" if gl3 else "--opengl2"
    cid = p.connect(p.GUI, options=options)
    if cid < 0:
        alt = "--opengl2" if gl3 else "--gl3"
        log(f"[warn] GUI({options}) 连接失败，改用 {alt} 再试一次")
        cid = p.connect(p.GUI, options=alt)
    if cid < 0:
        log("[warn] GUI 连接失败（检查 $DISPLAY / OpenGL 驱动）-> 只做无窗口自检")
        if p.connect(p.DIRECT) < 0:
            raise RuntimeError("无法连接 PyBullet 物理服务器")
        return False
    return True


def add_ground() -> None:
    """一块地面，纯粹给视角一个参照（--no-ground 可关掉）。"""
    try:
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[3.0, 3.0, 0.004],
                                     rgbaColor=[0.30, 0.32, 0.36, 1.0])
        collision = p.createCollisionShape(p.GEOM_PLANE, planeNormal=[0, 0, 1])
        p.createMultiBody(0.0, collision, visual, basePosition=[0.0, 0.0, -0.004])
    except p.error as exc:
        log(f"[warn] 地面画不出来（{exc}），已跳过")


def create_piece_visuals(pieces, colors, alpha: float) -> list[int]:
    """每个凸块一个纯视觉刚体：没有碰撞形状，因此不参与碰撞也不参与测距。"""
    bodies = []
    for pc in pieces:
        r, g, b = colors[pc.link]
        visual = p.createVisualShape(p.GEOM_MESH, fileName=str(pc.obj),
                                     rgbaColor=[r, g, b, float(alpha)])
        bodies.append(p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1,
                                        baseVisualShapeIndex=visual))
    return bodies


def apply_config(robot, cfg: dict[str, float], joint_index: dict[str, int]) -> None:
    for name, value in cfg.items():
        idx = joint_index.get(name)
        if idx is not None:
            p.resetJointState(robot, idx, float(value))


def link_world_pose(robot, link_index: int | None, cache: dict | None = None):
    """连杆的世界位姿 (pos, quat)；-1 = 基座，None = 这个 link 在 PyBullet 里没有。"""
    if link_index is None:
        return None
    if link_index == -1:
        return p.getBasePositionAndOrientation(robot)
    if cache is not None and link_index in cache:
        return cache[link_index]
    st = p.getLinkState(robot, link_index, computeForwardKinematics=True)
    pose = (st[4], st[5])
    if cache is not None:
        cache[link_index] = pose
    return pose


def piece_vertices(pieces) -> list[np.ndarray]:
    """每个凸块在 link 坐标系下的顶点（凸包 OBJ 里只有 v/f，读一次就够）。"""
    out = []
    for pc in pieces:
        verts, _ = lpdm.read_obj(pc.obj)
        out.append(np.asarray(verts, dtype=np.float64).reshape(-1, 3))
    return out


def swept_bounds(robot, pieces, joint_index, configs, vertices) -> tuple[np.ndarray, np.ndarray]:
    """整段轨迹里所有凸块顶点扫过的世界包围盒（自动抬高 / 自动取景都用它）。"""
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for cfg in configs:
        apply_config(robot, cfg, joint_index)
        cache: dict = {}
        for verts, pc in zip(vertices, pieces):
            pose = link_world_pose(robot, pc.link_index, cache)
            if pose is None or len(verts) == 0:
                continue
            world = verts @ rot_matrix(pose[1]).T + np.asarray(pose[0], dtype=np.float64)
            lo = np.minimum(lo, world.min(axis=0))
            hi = np.maximum(hi, world.max(axis=0))
    return lo, hi


def lift_robot(robot, lo: np.ndarray, clearance: float, extra: float) -> float:
    """把整机沿 +z 抬起，让摆动最低点离地 >= clearance；返回实际抬高量 (m)。

    基座是 fixed base，直接改基座位姿就行，整条运动链会一起上去。
    """
    if not np.all(np.isfinite(lo)):
        return 0.0
    need = max(0.0, float(clearance) - float(lo[2])) + max(0.0, float(extra))
    if need <= 1e-9:
        return 0.0
    pos, orn = p.getBasePositionAndOrientation(robot)
    p.resetBasePositionAndOrientation(robot, [pos[0], pos[1], pos[2] + need], orn)
    return need


def auto_camera(lo: np.ndarray, hi: np.ndarray) -> list[float]:
    """按扫掠包围盒自动取景：distance,yaw,pitch,target_x,target_y,target_z。

    取景距离按包围盒对角线缩放，保证整段摆动的最高点 / 最低点 / 最远点都在画面里。
    """
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))):
        return CAMERA_DEFAULT
    lo = np.asarray(lo, dtype=np.float64)
    hi = np.asarray(hi, dtype=np.float64)
    center = 0.5 * (lo + hi)
    radius = 0.5 * float(np.linalg.norm(hi - lo))
    dist = float(min(max(1.85 * radius, 0.55), 12.0))
    return [dist, CAMERA_DEFAULT[1], CAMERA_DEFAULT[2],
            float(center[0]), float(center[1]), float(center[2])]


def draw_pedestal(base_pos, radius: float = 0.09, color=(0.55, 0.58, 0.62)) -> None:
    """抬高后底座悬空：画个灰色线框 "支架" 到地面，纯参考线（不参与碰撞）。"""
    x, y, z = [float(v) for v in base_pos]
    if z <= 1e-6:
        return
    corners = [(x + dx, y + dy) for dx, dy in ((radius, radius), (radius, -radius),
                                               (-radius, -radius), (-radius, radius))]
    rgb = [float(c) for c in color]
    for cx, cy in corners:
        p.addUserDebugLine([cx, cy, 0.0], [cx, cy, z], rgb, lineWidth=1.5, lifeTime=0)
    for a, b in zip(corners, corners[1:] + corners[:1]):
        p.addUserDebugLine([a[0], a[1], 0.0], [b[0], b[1], 0.0], rgb, lineWidth=1.5, lifeTime=0)
        p.addUserDebugLine([a[0], a[1], z], [b[0], b[1], z], rgb, lineWidth=1.5, lifeTime=0)


def place_pieces(robot, pieces, bodies) -> None:
    """把凸块刚体摆到各自连杆的当前世界位姿（凸块 OBJ 已经在 link 坐标系里）。"""
    cache: dict = {}
    for pc, body in zip(pieces, bodies):
        pose = link_world_pose(robot, pc.link_index, cache)
        if pose is None:
            continue
        p.resetBasePositionAndOrientation(body, pose[0], pose[1])


def style_robot(robot, mode: str, alpha: float) -> None:
    """原始 URDF 网格怎么显示：ghost=半透明底图，hide=隐藏，visual=原样。"""
    if mode == "visual":
        return
    rgba = [0.80, 0.82, 0.86, 0.0 if mode == "hide" else float(alpha)]
    for link in [-1, *range(p.getNumJoints(robot))]:
        try:
            p.changeVisualShape(robot, link, rgbaColor=rgba)
        except p.error:
            pass


def draw_polyline(points, color, width: float, closed: bool, max_segments: int = 120) -> None:
    """把一串世界坐标点画成折线（点太多时抽稀，避免 debug line 数量爆掉）。"""
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 2:
        return
    if len(pts) > max_segments:
        idx = np.linspace(0, len(pts) - 1, max_segments + 1).astype(int)
        pts = pts[idx]
    rgb = [float(color[0]), float(color[1]), float(color[2])]
    for a, b in zip(pts[:-1], pts[1:]):
        p.addUserDebugLine(a.tolist(), b.tolist(), rgb, lineWidth=float(width), lifeTime=0)
    if closed and len(pts) > 2:
        p.addUserDebugLine(pts[-1].tolist(), pts[0].tolist(), rgb, lineWidth=float(width), lifeTime=0)


def draw_axes(origin, quat, size: float, ids: list[int]) -> list[int]:
    """每帧更新的一组 RGB 坐标轴（replaceItemUniqueId 原地替换，不会越画越多）。"""
    m = rot_matrix(quat)
    origin = np.asarray(origin, dtype=np.float64)
    colors = ((1.0, 0.25, 0.25), (0.25, 0.95, 0.25), (0.35, 0.45, 1.0))
    while len(ids) < 3:
        ids.append(-1)
    for k in range(3):
        ids[k] = p.addUserDebugLine(origin.tolist(), (origin + m[:, k] * size).tolist(),
                                    list(colors[k]), lineWidth=2.0, lifeTime=0,
                                    replaceItemUniqueId=ids[k])
    return ids


def draw_text(text: str, position, old_id: int) -> int:
    return p.addUserDebugText(text, [float(x) for x in position], textColorRGB=[1, 1, 1],
                              textSize=1.2, lifeTime=0, replaceItemUniqueId=old_id)


# ---------------------------------------------------------------------------
# 轨迹采样（纯运动学：resetJointState + FK，不做物理仿真）
# ---------------------------------------------------------------------------
def piece_centroids(pieces) -> list[np.ndarray]:
    """凸块在 link 坐标系下的质心（顶点均值近似，画轨迹足够）。"""
    out = []
    for pc in pieces:
        verts, _ = lpdm.read_obj(pc.obj)
        out.append(verts.mean(axis=0) if len(verts) else np.zeros(3))
    return out


def collect_trails(robot, pieces, joint_index, configs, link_ids, track_pieces: bool):
    """逐帧算出 (连杆原点轨迹, 凸块质心轨迹)，单位 m、世界坐标。"""
    link_trail: dict[int, list[np.ndarray]] = {i: [] for i in link_ids}
    piece_trail: list[list[np.ndarray]] = [[] for _ in pieces] if track_pieces else []
    centroids = piece_centroids(pieces) if track_pieces else []

    for cfg in configs:
        apply_config(robot, cfg, joint_index)
        poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        for idx in link_ids:
            st = p.getLinkState(robot, idx, computeForwardKinematics=True)
            poses[idx] = (np.asarray(st[4], dtype=np.float64),
                          np.asarray(st[5], dtype=np.float64))
            link_trail[idx].append(poses[idx][0])

        if not track_pieces:
            continue
        base_pos, base_orn = p.getBasePositionAndOrientation(robot)
        for k, pc in enumerate(pieces):
            if pc.link_index is None:
                continue
            if pc.link_index == -1:
                pos, quat = np.asarray(base_pos, dtype=np.float64), np.asarray(base_orn, dtype=np.float64)
            else:
                if pc.link_index not in poses:
                    st = p.getLinkState(robot, pc.link_index, computeForwardKinematics=True)
                    poses[pc.link_index] = (np.asarray(st[4], dtype=np.float64),
                                            np.asarray(st[5], dtype=np.float64))
                pos, quat = poses[pc.link_index]
            piece_trail[k].append(pos + rot_matrix(quat) @ centroids[k])

    return link_trail, piece_trail


def pick_ee_link(explicit: str | None, link_names: list[str], link_index: dict[str, int]):
    """末端参考连杆：先看 --ee-link，再按常见命名猜，最后取运动链最后一个 link。"""
    if explicit:
        if explicit in link_index:
            return explicit, link_index[explicit]
        log(f"[warn] --ee-link {explicit} 不在 URDF 里，改为自动选择")
    for name in EE_CANDIDATES:
        if name in link_index:
            return name, link_index[name]
    name = link_names[-1]
    return name, link_index[name]


def build_ghosts(robot, pieces, joint_index, colors, motion, args,
                 vertices=None, skip_links=()) -> list[list[int]]:
    """把整个周期的 N 个时刻各 "定影" 一份半透明凸包，用来反映扫过的体积。

    整个周期几乎不动的凸块（基座、绕自身轴转的圆柱…）只画当前帧那一份：
    否则 N 份半透明网格叠在同一处，会在基座旁边糊出一根深色的 "柱子"。
    """
    count = int(args.ghosts)
    if count <= 0:
        return []
    if motion.periodic:
        times = np.linspace(0.0, 1.0, count, endpoint=False)
    else:
        assert motion.traj_t is not None
        times = np.linspace(float(motion.traj_t[0]), float(motion.traj_t[-1]), count, endpoint=False)

    if vertices is None:
        vertices = piece_vertices(pieces)

    # 1) 先算每个时刻所有凸块的世界位姿 / 世界包围盒
    poses_per_t: list[list] = []
    boxes_per_t: list[list] = []
    for t in times:
        apply_config(robot, motion.sample(float(t)), joint_index)
        cache: dict = {}
        poses, boxes = [], []
        for verts, pc in zip(vertices, pieces):
            pose = link_world_pose(robot, pc.link_index, cache)
            poses.append(pose)
            if pose is None or len(verts) == 0:
                boxes.append(None)
                continue
            world = verts @ rot_matrix(pose[1]).T + np.asarray(pose[0], dtype=np.float64)
            boxes.append((world.min(axis=0), world.max(axis=0)))
        poses_per_t.append(poses)
        boxes_per_t.append(boxes)

    # 2) 标记 "会动" 的凸块：世界包围盒变化超过 3 mm
    moving = [False] * len(pieces)
    for k in range(len(pieces)):
        ref = boxes_per_t[0][k]
        if ref is None:
            continue
        for boxes in boxes_per_t[1:]:
            box = boxes[k]
            if box is None:
                continue
            if (np.abs(box[0] - ref[0]).max() > 0.003
                    or np.abs(box[1] - ref[1]).max() > 0.003):
                moving[k] = True
                break

    # 3) 只给会动的凸块生成残影
    ghosts: list[list[int]] = []
    for poses in poses_per_t:
        group: list[int] = []
        for k, pc in enumerate(pieces):
            if not moving[k] or pc.link in skip_links:
                continue
            pose = poses[k]
            if pose is None:
                continue
            r, g, b = colors[pc.link]
            visual = p.createVisualShape(p.GEOM_MESH, fileName=str(pc.obj),
                                         rgbaColor=[r, g, b, float(args.ghost_alpha)])
            group.append(p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1,
                                           baseVisualShapeIndex=visual,
                                           basePosition=list(pose[0]),
                                           baseOrientation=list(pose[1])))
        ghosts.append(group)

    static = list(dict.fromkeys(pc.link for k, pc in enumerate(pieces) if not moving[k]))
    log(f"[残影] {len(ghosts)} 份（t = {', '.join(f'{t:.2f}' for t in times)}），"
        f"每份 {sum(moving)} 个会动的凸块")
    if static:
        log(f"       {len(static)} 个连杆全程几乎不动，只画当前帧那一份（避免残影叠成一根柱子）: "
            + ", ".join(static))
    return ghosts


def link_label_anchors(pieces, link_index: dict[str, int],
                       skip=()) -> list[tuple[str, int, np.ndarray]]:
    """每个连杆取一个标签锚点（该 link 所有凸块质心的平均，link 坐标系）。"""
    groups: dict[str, list[np.ndarray]] = {}
    for pc, c in zip(pieces, piece_centroids(pieces)):
        groups.setdefault(pc.link, []).append(np.asarray(c, dtype=np.float64))
    out = []
    for name, pts in groups.items():
        if name not in link_index or name in skip:
            continue
        out.append((name, link_index[name], np.mean(pts, axis=0)))
    return out


def draw_link_labels(robot, anchors, ids: list[int], size: float = 0.7) -> None:
    """每个连杆名字标在凸块上（replaceItemUniqueId 原地替换，跟随运动）。"""
    cache: dict = {}
    while len(ids) < len(anchors):
        ids.append(-1)
    for k, (name, link_idx, local) in enumerate(anchors):
        pose = link_world_pose(robot, link_idx, cache)
        if pose is None:
            continue
        world = np.asarray(pose[0], dtype=np.float64) + rot_matrix(pose[1]) @ local
        ids[k] = p.addUserDebugText(name, world.tolist(), textColorRGB=[1.0, 1.0, 1.0],
                                    textSize=float(size), lifeTime=0,
                                    replaceItemUniqueId=ids[k])


# ---------------------------------------------------------------------------
# 文本输出 / 轨迹导出 / 回放
# ---------------------------------------------------------------------------
def print_summary(pieces, link_order, link_index, link_trail, ee_name, ee_trail) -> None:
    """无窗口模式（也用于 GUI 的启动日志）打印轨迹自检信息。"""
    counts: dict[str, int] = {}
    for pc in pieces:
        counts[pc.link] = counts.get(pc.link, 0) + 1
    log("")
    log(f"[凸块] 共 {len(pieces)} 个：" + ", ".join(f"{k}:{v}" for k, v in counts.items()))

    log("")
    log("[各连杆原点的世界轨迹] (m)")
    log(f"  {'link':<26}{'xmin':>9}{'xmax':>9}{'ymin':>9}{'ymax':>9}"
        f"{'zmin':>9}{'zmax':>9}{'路程(m)':>10}")
    for name in link_order:
        pts = link_trail.get(link_index[name]) or []
        if len(pts) < 2:
            continue
        arr = np.asarray(pts, dtype=np.float64)
        path = float(np.linalg.norm(np.diff(arr, axis=0), axis=1).sum())
        lo, hi = arr.min(axis=0), arr.max(axis=0)
        cells = (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        log(f"  {name:<26}" + "".join(f"{v:9.3f}" for v in cells) + f"{path:10.3f}")

    if len(ee_trail) >= 2:
        arr = np.asarray(ee_trail, dtype=np.float64)
        path = float(np.linalg.norm(np.diff(arr, axis=0), axis=1).sum())
        lo, hi = arr.min(axis=0), arr.max(axis=0)
        cells = (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        log("")
        log(f"[末端 {ee_name} 的轨迹] (m)")
        log("  " + "".join(f"{v:9.3f}" for v in cells) + f"{path:10.3f}  (路程)")
        log(f"  行程范围: {np.round(hi - lo, 3).tolist()} m")


def save_traj_csv(path: Path, times: np.ndarray, configs, joint_names: list[str]) -> None:
    """把本次使用的轨迹导出成 CSV（表头 t + 关节名），之后可用 --traj 原样回放。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", *joint_names])
        for t, cfg in zip(times, configs):
            writer.writerow([f"{float(t):.6f}"]
                            + [f"{float(cfg.get(n, 0.0)):.6f}" for n in joint_names])
    log(f"[导出] 轨迹 CSV: {path}（{len(configs)} 帧 x {len(joint_names)} 关节）")


def poll_keys(state: dict) -> None:
    """SPACE 暂停/继续；q / ESC 退出。GUI 断开时静默返回。"""
    try:
        events = p.getKeyboardEvents()
    except p.error:
        state["quit"] = True
        return
    for code, value in events.items():
        if not (value & p.KEY_WAS_TRIGGERED):
            continue
        if code in (ord("q"), ord("Q"), 27):
            state["quit"] = True
        elif code == 32:                                  # SPACE
            state["pause"] = not state["pause"]


def run_gui(robot, pieces, joint_index, bodies, configs, times, motion, args,
            ee_index: int, label_anchors=(), text_pos=(0.0, -0.30, 0.02)) -> None:
    """逐帧摆姿势 + 更新坐标系 / 文字，直到用户退出或播完 --loops 遍。"""
    delay = (float(args.cycle_seconds) / max(len(configs), 1)
             if args.cycle_seconds > 0 else 1.0 / max(float(args.fps), 1e-3))
    log(f"[GUI] {len(configs)} 帧，每帧 {delay * 1000:.1f} ms（"
        + (f"--cycle-seconds {args.cycle_seconds:g}" if args.cycle_seconds > 0
           else f"--fps {args.fps:g}")
        + "）; SPACE 暂停/继续，q 退出")

    state = {"pause": False, "quit": False}
    axes_ids: list[int] = []
    label_ids: list[int] = []
    label_id = -1
    moves = list(zip(configs, times))
    n_done = 0

    while not state["quit"]:
        for cfg, t in moves:
            poll_keys(state)
            while state["pause"] and not state["quit"]:
                poll_keys(state)
                time.sleep(0.05)
            if state["quit"]:
                break

            apply_config(robot, cfg, joint_index)
            place_pieces(robot, pieces, bodies)

            if ee_index >= 0:
                st = p.getLinkState(robot, ee_index, computeForwardKinematics=True)
                if args.axes > 0:
                    axes_ids = draw_axes(st[4], st[5], float(args.axes), axes_ids)

            if args.labels and label_anchors:
                draw_link_labels(robot, label_anchors, label_ids, float(args.label_size))

            joint_text = "  ".join(f"{n}={cfg[n]:+.2f}" for n in motion.names[:6])
            label_id = draw_text(f"t={t:.2f}  {joint_text}", list(text_pos), label_id)

            n_done += 1
            time.sleep(delay)

        if args.loops and n_done >= args.loops * len(moves):
            break

    log("[GUI] 回放结束，窗口保持打开（SPACE 暂停/继续，q 或关闭窗口退出）")
    try:
        while lpdm.connected() and not state["quit"]:
            poll_keys(state)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------
def parse_names(text: str) -> list[str]:
    """--hide base_link,shoulder_link -> ['base_link', 'shoulder_link']"""
    return [n.strip() for n in str(text).split(",") if n.strip()]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="凸包运动轨迹查看器：把任意 URDF 的碰撞体拆成凸块，"
                    "沿轨迹摆动并画出扫过的轨迹",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    g_model = ap.add_argument_group("模型 / 缓存")
    g_model.add_argument("--urdf", default=str(DEFAULT_URDF), help="URDF 路径（换机器人只改这个）")
    g_model.add_argument("--out-dir", default=str(ROOT / "distance_matrix_out"),
                         help="缓存根目录（与 link_piece_distance_matrix.py 共用）")
    g_model.add_argument("--cache-dir", default=None, help="凸分解缓存目录；默认 <out-dir>/cache")
    g_model.add_argument("--recompute", action="store_true", help="忽略缓存，重新跑 V-HACD")

    g_motion = ap.add_argument_group("运动 / 轨迹")
    g_motion.add_argument("--samples", type=int, default=60,
                          help="一个周期取多少帧（回放 --traj 时指重采样帧数）")
    g_motion.add_argument("--home", type=lpdm.parse_floats, default=None,
                          help="基准位姿（按可动关节顺序，不含夹爪）；缺省 = clamp(0, lower, upper)")
    g_motion.add_argument("--amp", type=float, default=0.35,
                          help="正弦幅值（rad；prismatic 关节是 m）")
    g_motion.add_argument("--amp-frac", type=float, default=0.0,
                          help=">0 时按每个关节行程的该比例自动取幅值（覆盖 --amp）")
    g_motion.add_argument("--freq", type=float, default=1.0, help="一个周期内的正弦周期数")
    g_motion.add_argument("--phase-step", type=float, default=60.0, help="相邻关节相位差 (deg)")
    g_motion.add_argument("--gripper", choices=["cycle", "open", "closed"], default="cycle",
                          help="夹爪/末端关节的动作方式")
    g_motion.add_argument("--traj", default=None,
                          help="回放外部轨迹：.npz(joint_names+joint_positions) "
                               "或 .csv(表头含关节名，可选 t 列)")
    g_motion.add_argument("--save-traj", default=None,
                          help="把本次使用的轨迹导出成 CSV（之后可用 --traj 回放）")

    g_vhacd = ap.add_argument_group("凸分解（与主程序一致，共用缓存）")
    g_vhacd.add_argument("--convex-ratio", type=float, default=0.99,
                         help="mesh 体积/凸包体积 >= 该值视为已经凸，不再 V-HACD")
    g_vhacd.add_argument("--vhacd-concavity", type=float, default=0.002, help="V-HACD 最大凹度")
    g_vhacd.add_argument("--vhacd-resolution", type=int, default=100000, help="V-HACD 体素分辨率")
    g_vhacd.add_argument("--vhacd-depth", type=int, default=20, help="V-HACD 最大分割层数")
    g_vhacd.add_argument("--vhacd-plane-downsampling", type=int, default=4, help="V-HACD 平面降采样")
    g_vhacd.add_argument("--vhacd-hull-downsampling", type=int, default=4, help="V-HACD 凸包降采样")
    g_vhacd.add_argument("--max-vertices-per-ch", type=int, default=64, help="每个凸块最大顶点数")
    g_vhacd.add_argument("--vhacd-min-volume", type=float, default=0.0001, help="V-HACD 最小凸块体积")
    g_vhacd.add_argument("--vhacd-verbose", action="store_true", help="显示 V-HACD 自己的进度输出")

    g_view = ap.add_argument_group("显示 / 回放")
    g_view.add_argument("--headless", action="store_true",
                        help="不显示窗口，只打印轨迹自检信息（无显示器 / CI 用）")
    g_view.add_argument("--gl3", action="store_true", help="用 --gl3 渲染器（默认 --opengl2）")
    g_view.add_argument("--fps", type=float, default=30.0, help="回放帧率")
    g_view.add_argument("--cycle-seconds", type=float, default=0.0,
                        help=">0 时用「一个周期的墙钟秒数」代替 --fps（想看清楚就用 20~30）")
    g_view.add_argument("--loops", type=int, default=0, help="播放几遍，0 = 一直循环")
    g_view.add_argument("--robot-mode", choices=["ghost", "hide", "visual"], default="ghost",
                        help="原始 URDF 网格：半透明底图 / 隐藏 / 原样显示")
    g_view.add_argument("--robot-alpha", type=float, default=0.12,
                        help="--robot-mode ghost 时原始网格的透明度")
    g_view.add_argument("--piece-alpha", type=float, default=1.0, help="当前帧凸块的透明度")
    g_view.add_argument("--ghosts", type=int, default=6,
                        help="整个周期的残影个数（0 = 不画；全程不动的凸块不会重复生成）")
    g_view.add_argument("--ghost-alpha", type=float, default=0.18, help="残影透明度")
    g_view.add_argument("--trail", choices=["link", "piece", "none"], default="link",
                        help="轨迹线：连杆原点 / 凸块质心 / 不画")
    g_view.add_argument("--trail-width", type=float, default=2.0, help="轨迹线宽")
    g_view.add_argument("--no-trail-closed", dest="trail_closed", action="store_false",
                        help="轨迹线不首尾相连（周期运动默认相连）")
    g_view.add_argument("--axes", type=float, default=0.06,
                        help="每帧在末端画 RGB 坐标轴，参数是轴长 (m)；0 = 不画")
    g_view.add_argument("--ee-link", default=None,
                        help="末端参考连杆名（默认自动：ee_link / tool0 / tcp / ...）")
    g_view.add_argument("--ee-trail", action=argparse.BooleanOptionalAction, default=True,
                        help="画末端参考点的轨迹")
    g_view.add_argument("--clearance", type=float, default=0.02,
                        help="自动抬高：让摆动最低点离地至少这么多米（0 = 不抬高）")
    g_view.add_argument("--lift", type=float, default=0.0,
                        help="在自动抬高之外再整体抬高多少米（想让整机悬空看得更清楚时用）")
    g_view.add_argument("--pedestal", action=argparse.BooleanOptionalAction, default=True,
                        help="抬高后底座悬空时，画灰色线框支架到地面（默认开）")
    g_view.add_argument("--labels", action=argparse.BooleanOptionalAction, default=True,
                        help="在每个连杆的凸块上标出连杆名，方便确认画面里这块是什么")
    g_view.add_argument("--label-size", type=float, default=0.7, help="连杆名文字大小")
    g_view.add_argument("--hide", type=parse_names, default=None,
                        help="把指定连杆的凸块藏掉，逗号分隔：--hide base_link,shoulder_link")
    g_view.add_argument("--camera", type=lpdm.parse_floats, default=None,
                        help="相机 distance,yaw,pitch,target_x,target_y,target_z；"
                             "缺省按整段扫掠范围自动取景")
    g_view.add_argument("--no-ground", action="store_true", help="不画地面")
    ap.set_defaults(trail_closed=True)
    return ap.parse_args(argv)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def run(args) -> int:
    urdf = Path(args.urdf).expanduser().resolve()
    if not urdf.is_file():
        log(f"[error] URDF 不存在: {urdf}")
        return 2

    out_dir = Path(args.out_dir).expanduser().resolve()
    cache_dir = (Path(args.cache_dir).expanduser().resolve() if args.cache_dir
                 else out_dir / "cache")
    cache_dir.mkdir(parents=True, exist_ok=True)

    links, joints = lpdm.parse_urdf(urdf)
    if not any(links.values()):
        log("[error] 这个 URDF 没有任何 <collision>，没有凸包可看")
        return 4

    gui = connect(not args.headless, args.gl3)
    p.setGravity(0.0, 0.0, 0.0)              # 不做物理仿真，只摆姿势，别让重力插手
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    try:
        robot = p.loadURDF(str(urdf), useFixedBase=True)
    except p.error as exc:
        log(f"[error] PyBullet 加载 URDF 失败: {exc}")
        log("        常见原因：<mesh filename> 用了 package:// 或绝对路径，"
            "PyBullet 只能读相对 URDF 的路径")
        return 3

    # link / joint 索引：和 link_piece_distance_matrix.py 完全一致
    link_names = [p.getBodyInfo(robot)[0].decode()]
    link_index = {link_names[0]: -1}
    joint_index: dict[str, int] = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        jname, jtype, child = info[1].decode(), info[2], info[12].decode()
        link_names.append(child)
        link_index[child] = i
        if jtype != p.JOINT_FIXED:
            joint_index[jname] = i

    missing = [n for n in links if n not in link_index]
    if missing:
        log(f"[warn] 这些 URDF link 在 PyBullet 里找不到，已跳过: {', '.join(missing)}")

    # 1) 凸分解（与主程序共用缓存，所以不会重复跑 V-HACD）
    pieces, report = lpdm.build_pieces(links, link_names, link_index, args, cache_dir)
    if not pieces:
        log("[error] 没有解析到任何凸块")
        return 4

    link_order = list(dict.fromkeys(pc.link for pc in pieces))
    colors = link_colors(link_order)

    # 2) 轨迹
    try:
        motion = Motion(joints, args)
    except RuntimeError as exc:
        log(f"[error] {exc}")
        return 4
    motion.calibrate_gripper(robot, joint_index)      # 实测"张开/闭合"是哪个关节值

    times = motion.times(args.samples)
    configs = [motion.sample(float(t)) for t in times]
    q_names = sorted(joint_index, key=lambda k: joint_index[k])

    # 2.5) 自动抬高 + 自动取景：保证整个摆动都看得见、不穿地
    vertices = piece_vertices(pieces)
    sweep_lo, sweep_hi = swept_bounds(robot, pieces, joint_index, configs, vertices)
    low_z = float(sweep_lo[2])
    lift = lift_robot(robot, sweep_lo, args.clearance, args.lift)
    if lift > 0.0:
        sweep_lo = sweep_lo + np.array([0.0, 0.0, lift])
        sweep_hi = sweep_hi + np.array([0.0, 0.0, lift])
    cam = list(args.camera) if args.camera else auto_camera(sweep_lo, sweep_hi)

    log("=" * 100)
    log(f"URDF     : {urdf}")
    log(f"PyBullet : {p.getAPIVersion()}  模式: {'GUI' if gui else 'HEADLESS（只自检）'}")
    log(f"凸分解   : {len(pieces)} 个凸块 / {len(link_order)} 个连杆（缓存 {cache_dir}）")
    for lname, src, how in report:
        log(f"  {lname:<26} {src:<40} {how}")
    log("")
    log("[运动] 摆动关节: " + ", ".join(f"{j.name}({j.type})" for j in motion.arm))
    log("       基准位姿: " + ", ".join(f"{n}={v:+.3f}"
                                      for n, v in zip(motion.names, motion.home)))
    log("       幅值    : " + ", ".join(f"{n}={v:.3f}"
                                      for n, v in zip(motion.names, motion.amp)))
    if motion.leader:
        log(f"       夹爪/末端: {motion.leader}（--gripper {args.gripper}，"
            f"{len(motion.mimic)} 个 mimic 关节联动）")
    log(f"       轨迹来源: {'程序正弦' if motion.periodic else '外部轨迹'}"
        f"，{len(times)} 帧，t = {times[0]:.3f} .. {times[-1]:.3f}")
    if lift > 0.0:
        log(f"[抬高] 摆动最低点 z={low_z:+.3f} m 会穿到地面以下 -> 整机沿 +z 抬高 "
            f"{lift:.3f} m，现在最低点 z={low_z + lift:+.3f} m"
            f"（--clearance {args.clearance:g} m）")
    else:
        log(f"[抬高] 摆动最低点 z={low_z:+.3f} m，没有穿地 -> 不抬高")
    log(f"[相机] {'手动 --camera' if args.camera else '自动取景'}: "
        f"distance={cam[0]:.2f} yaw={cam[1]:.0f} pitch={cam[2]:.0f} "
        f"target=({cam[3]:+.2f}, {cam[4]:+.2f}, {cam[5]:+.2f})"
        f"；扫掠范围 {np.round(sweep_lo, 2).tolist()} .. {np.round(sweep_hi, 2).tolist()}")

    # 3) 轨迹几何：连杆原点 / 凸块质心 / 末端
    ee_name, ee_index = pick_ee_link(args.ee_link, link_names, link_index)
    link_ids = [link_index[n] for n in link_order if link_index[n] >= 0]
    track_pieces = args.trail == "piece"
    link_trail, piece_trail = collect_trails(robot, pieces, joint_index, configs,
                                             link_ids, track_pieces)

    ee_trail: list[np.ndarray] = []
    if ee_index >= 0:
        for cfg in configs:
            apply_config(robot, cfg, joint_index)
            st = p.getLinkState(robot, ee_index, computeForwardKinematics=True)
            ee_trail.append(np.asarray(st[4], dtype=np.float64))

    print_summary(pieces, link_order, link_index, link_trail, ee_name, ee_trail)

    if args.save_traj:
        save_traj_csv(Path(args.save_traj).expanduser(), times, configs, q_names)

    if not gui:
        log("")
        log("[headless] 只做自检，没有画图。要看交互画面请在带 $DISPLAY 的机器上运行本脚本。")
        return 0

    # 4) GUI 场景：地面 / 原始网格 / 凸块 / 残影 / 轨迹线
    if not args.no_ground:
        add_ground()
    base_pos, _ = p.getBasePositionAndOrientation(robot)
    if lift > 0.0 and args.pedestal:
        draw_pedestal(base_pos)
        log(f"[支架] 抬高后底座悬空 {base_pos[2]:.3f} m，画了灰色线框支架（--no-pedestal 去掉）")

    if len(cam) != 6:
        log(f"[warn] --camera 需要 6 个数，收到 {len(cam)} 个，改用自动取景")
        cam = auto_camera(sweep_lo, sweep_hi)
    p.resetDebugVisualizerCamera(float(cam[0]), float(cam[1]), float(cam[2]),
                                 [float(x) for x in cam[3:6]])

    style_robot(robot, args.robot_mode, args.robot_alpha)

    hide = set(args.hide or [])
    unknown = sorted(hide - set(pc.link for pc in pieces))
    if unknown:
        log(f"[warn] --hide 里的这些连杆没有凸块，忽略: {', '.join(unknown)}")
    log("[凸块] 本脚本不添加任何障碍物：画面里所有实心色块都是上表里的连杆凸块"
        "（--labels 会把连杆名标在块上，--hide 可以逐块关掉）")

    bodies = create_piece_visuals(pieces, colors, args.piece_alpha)
    if hide:
        for pc, body in zip(pieces, bodies):
            if pc.link in hide:
                p.changeVisualShape(body, -1, rgbaColor=[0.5, 0.5, 0.5, 0.0])
        log(f"[凸块] 已隐藏: {', '.join(sorted(hide & set(pc.link for pc in pieces)))}")
    build_ghosts(robot, pieces, joint_index, colors, motion, args,
                 vertices=vertices, skip_links=hide)

    idx_to_name = {v: k for k, v in link_index.items()}
    closed = bool(args.trail_closed) and motion.periodic
    if args.trail == "link":
        for idx in link_ids:
            name = idx_to_name.get(idx, "")
            draw_polyline(link_trail[idx], colors.get(name, (0.75, 0.75, 0.78)),
                          args.trail_width, closed)
        log(f"[轨迹线] 连杆原点 {len(link_ids)} 条" + ("（首尾闭合）" if closed else ""))
    elif track_pieces:
        for k, pc in enumerate(pieces):
            if len(piece_trail[k]) >= 2:
                draw_polyline(piece_trail[k], colors[pc.link], args.trail_width * 0.7, closed)
        log(f"[轨迹线] 凸块质心 {len(pieces)} 条" + ("（首尾闭合）" if closed else ""))

    if args.ee_trail and len(ee_trail) >= 2:
        draw_polyline(ee_trail, (1.0, 0.85, 0.2), args.trail_width * 1.4, closed)
        log(f"[轨迹线] 末端参考点 {ee_name} 1 条")

    # 5) 播放
    apply_config(robot, configs[0], joint_index)
    place_pieces(robot, pieces, bodies)
    anchors = link_label_anchors(pieces, link_index, skip=hide) if args.labels else []
    text_pos = (float(sweep_lo[0]), float(sweep_lo[1]) - 0.12, float(sweep_lo[2]))
    run_gui(robot, pieces, joint_index, bodies, configs, times, motion, args,
            ee_index, label_anchors=anchors, text_pos=text_pos)
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        return run(args)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        log(f"[error] {exc}")
        return 2
    except p.error as exc:
        log(f"[error] PyBullet 出错（GUI 窗口被关闭？）: {exc}")
        return 3
    except KeyboardInterrupt:
        log("")
        log("[info] 用户中断")
        return 130
    finally:
        try:
            if lpdm.connected():
                p.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
