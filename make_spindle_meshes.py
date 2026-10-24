#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成「铣削电主轴 + 硬质合金立铣刀」的 STL 网格，供 URDF 使用。

仓库里没有任何 CAD，所以这里用解析几何现场生成网格（只依赖 numpy）：
机身/法兰/刀柄都是同轴圆柱或圆台，几十行就能写清楚，也方便改尺寸。

    python make_spindle_meshes.py                # 写到 meshes/spindle/，覆盖旧文件
    python make_spindle_meshes.py --n 192        # 圆周分段更细（默认 96）
    python make_spindle_meshes.py --out /tmp/m   # 写到别处看看

输出（visual / collision 两份，几何相同，方便以后单独给视觉模型加倒角/刃槽）：

    meshes/spindle/visual/spindle_mount.stl        转接法兰
    meshes/spindle/visual/spindle_body.stl         电主轴机身（Ø80）
    meshes/spindle/visual/spindle_nose.stl         前轴承座（Ø60）
    meshes/spindle/visual/spindle_collet_nut.stl   ER20 夹头螺母（Ø34）
    meshes/spindle/visual/milling_tool.stl         Ø10 立铣刀

电主轴故意拆成"机身 / 前座 / 螺母"三个独立 mesh：每个都是凸的，
URDF 里写在同一个 link 的多个 <collision> 下 -> 凸分解时各成 1 个凸块，
既不用跑 V-HACD，也不会像"整体网格的凸包"那样把 Ø34 的螺母放大成 Ø80。

坐标系约定（和 URDF 里 link 的原点一致）：
    * 每个 mesh 的 z = 0 是与上一节贴合的法兰面，圆柱轴线沿 +z；
    * UR5 的 ee_link 上，这条 +z 正对工具轴线（home 姿态时竖直向下）；
    * 单位 m。

尺寸取舍
--------
按常见的 2.2 kW 水冷 ER20 电主轴（Ø80 × 200）建模：URDF 里给的质量是法兰 0.60 kg +
主轴 3.20 kg + 刀具 0.045 kg ≈ 3.85 kg，在 UR5 的 5 kg 额定负载内。铣刀按 Ø10 四刃立铣刀
简化成"Ø10 圆柱 + 倒角"——螺旋刃在测距里是轴对称为主的细节，真做出来既慢又不影响最小距离。
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
DEFAULT_OUT = ROOT / "meshes" / "spindle"

# ---------------------------------------------------------------------------
# 尺寸（m）—— 想换型号只改这里
# ---------------------------------------------------------------------------
FLANGE_R, FLANGE_T = 0.044, 0.014        # 转接法兰 Ø88 × 14
BODY_R, BODY_T = 0.040, 0.200            # 机身 Ø80 × 200（后 14 mm 藏在法兰里）
NOSE_R, NOSE_T = 0.030, 0.022            # 前轴承座 Ø60 × 22
NUT_R, NUT_T = 0.017, 0.017              # ER20 夹头螺母 Ø34 × 17
TOOL_R = 0.005                           # 铣刀 Ø10
SHANK_T, FLUTE_T = 0.040, 0.032          # 刀柄 40 mm + 刃部 32 mm

CHAMFER = 0.001                          # 各段端面倒角

# 各段沿 z 的位置（body 自己的坐标系，z=0 为法兰背面 = 转接法兰的前端面）
Z_BODY = BODY_T                                  # 0.200  机身结束
Z_NOSE = Z_BODY + NOSE_T                         # 0.222  前轴承座结束
Z_NUT = Z_NOSE + NUT_T                           # 0.239  螺母结束


def log(msg: str = "") -> None:
    print(msg, flush=True)


# ---------------------------------------------------------------------------
# 解析几何：圆台 / 圆柱
# ---------------------------------------------------------------------------
def frustum(r0: float, r1: float, z0: float, z1: float, n: int,
            cap0: bool = True, cap1: bool = True) -> np.ndarray:
    """沿 +z 的圆台（r0=0 或 r1=0 就是圆锥），返回 (m, 3, 3) 三角形。

    法线朝外：侧面按 (底_i, 底_i+1, 顶_i+1) 绕，端面各自成扇。
    """
    ang = np.arange(n) * (2.0 * np.pi / n)
    c, s = np.cos(ang), np.sin(ang)
    p0 = np.stack([r0 * c, r0 * s, np.full(n, z0)], axis=1)
    p1 = np.stack([r1 * c, r1 * s, np.full(n, z1)], axis=1)
    j = np.arange(n)
    j2 = (j + 1) % n

    tris = [
        np.stack([p0[j], p0[j2], p1[j2]], axis=1),
        np.stack([p0[j], p1[j2], p1[j]], axis=1),
    ]
    if cap0 and r0 > 0.0:                     # 朝 -z 的端面
        ctr = np.tile([0.0, 0.0, z0], (n, 1))
        tris.append(np.stack([ctr, p0[j2], p0[j]], axis=1))
    if cap1 and r1 > 0.0:                     # 朝 +z 的端面
        ctr = np.tile([0.0, 0.0, z1], (n, 1))
        tris.append(np.stack([ctr, p1[j], p1[j2]], axis=1))
    return np.concatenate(tris)


def tube(r: float, z0: float, z1: float, n: int,
         cap0: bool = False, cap1: bool = False) -> np.ndarray:
    """等半径的一段圆柱（端面默认不封，交给相邻段）。"""
    return frustum(r, r, z0, z1, n, cap0=cap0, cap1=cap1)


def write_stl(tris: np.ndarray, path: Path, name: str = "mesh") -> int:
    """写二进制 STL（50 字节/三角形），返回三角形数。"""
    tris = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    nor = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nor, axis=1, keepdims=True)
    nor = np.divide(nor, ln, out=np.zeros_like(nor), where=ln > 0.0)

    buf = np.zeros((len(tris), 12), dtype="<f4")
    buf[:, 0:3] = nor
    buf[:, 3:12] = tris.reshape(len(tris), 9)

    path.parent.mkdir(parents=True, exist_ok=True)
    header = f"binary STL {name} - make_spindle_meshes.py".encode("ascii")[:80]
    # 每条记录 50 字节：法线 12 + 3 顶点 36 + 属性 2；属性必须逐条跟在顶点后面
    raw = np.zeros((len(tris), 50), dtype=np.uint8)
    raw[:, 0:48] = buf.view(np.uint8).reshape(len(tris), 48)
    with open(path, "wb") as fh:
        fh.write(header.ljust(80, b" "))
        fh.write(struct.pack("<I", len(tris)))
        fh.write(raw.tobytes())
    return len(tris)


# ---------------------------------------------------------------------------
# 自检：体积 / 包围盒 / 是否封闭（每条边恰好被两个三角形共用）
# ---------------------------------------------------------------------------
def mesh_volume(tris: np.ndarray) -> float:
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def mesh_extent_mm(tris: np.ndarray) -> list[float]:
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    return [round(float(v) * 1000.0, 2) for v in (hi - lo)]


def edge_report(tris: np.ndarray) -> tuple[int, int]:
    """返回 (边的种数, 不是恰好被两个三角形共用的边数)；后者应为 0（网格封闭）。"""
    _verts, inv = np.unique(np.round(tris.reshape(-1, 3), 9), axis=0, return_inverse=True)
    faces = np.asarray(inv).reshape(-1, 3)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    edges = np.sort(edges, axis=1)
    uniq, counts = np.unique(edges, axis=0, return_counts=True)
    return len(uniq), int((counts != 2).sum())


# ---------------------------------------------------------------------------
# 五个零件：转接法兰 / 机身 / 前轴承座 / ER20 螺母 / 铣刀
# ---------------------------------------------------------------------------
def build_mount(n: int) -> np.ndarray:
    """转接法兰：Ø88 × 14 mm，两端 1 mm 倒角（z = 0 贴 ee_link）。"""
    return np.concatenate([
        frustum(FLANGE_R - CHAMFER, FLANGE_R, 0.0, CHAMFER, n, cap0=True, cap1=False),
        tube(FLANGE_R, CHAMFER, FLANGE_T - CHAMFER, n),
        frustum(FLANGE_R, FLANGE_R - CHAMFER, FLANGE_T - CHAMFER, FLANGE_T, n,
                cap0=False, cap1=True),
    ])


def build_body(n: int) -> np.ndarray:
    """电主轴机身：Ø80 × 200，两端 1 mm 倒角（z = 0 贴法兰背面）。

    机身是一个实心圆柱 -> 凸的，V-HACD 直接跳过；后盖藏在 Ø88 法兰里，不单独建模。
    """
    return np.concatenate([
        frustum(BODY_R - CHAMFER, BODY_R, 0.0, CHAMFER, n, cap0=True, cap1=False),
        tube(BODY_R, CHAMFER, Z_BODY - CHAMFER, n),
        frustum(BODY_R, BODY_R - CHAMFER, Z_BODY - CHAMFER, Z_BODY, n,
                cap0=False, cap1=True),
    ])


def build_nose(n: int) -> np.ndarray:
    """前轴承座：Ø60 × 22，坐在机身前端（z = 200..222）。"""
    return np.concatenate([
        frustum(NOSE_R - CHAMFER, NOSE_R, Z_BODY, Z_BODY + CHAMFER, n,
                cap0=True, cap1=False),
        tube(NOSE_R, Z_BODY + CHAMFER, Z_NOSE - CHAMFER, n),
        frustum(NOSE_R, NOSE_R - CHAMFER, Z_NOSE - CHAMFER, Z_NOSE, n,
                cap0=False, cap1=True),
    ])


def build_nut(n: int) -> np.ndarray:
    """ER20 夹头螺母：Ø34 × 17（z = 222..239），前端 1 mm 倒角。"""
    return np.concatenate([
        frustum(NUT_R - CHAMFER, NUT_R, Z_NOSE, Z_NOSE + CHAMFER, n,
                cap0=True, cap1=False),
        tube(NUT_R, Z_NOSE + CHAMFER, Z_NUT - CHAMFER, n),
        frustum(NUT_R, NUT_R - CHAMFER, Z_NUT - CHAMFER, Z_NUT, n,
                cap0=False, cap1=True),
    ])


def build_tool(n: int) -> np.ndarray:
    """Ø10 立铣刀：刀柄 40 mm + 刃部 32 mm + 端面 0.5 mm 倒角（z = 0 夹在螺母里）。"""
    z_end = SHANK_T + FLUTE_T
    return np.concatenate([
        frustum(TOOL_R - 0.0005, TOOL_R, 0.0, 0.0005, n, cap0=True, cap1=False),
        tube(TOOL_R, 0.0005, z_end - 0.0005, n),
        frustum(TOOL_R, TOOL_R - 0.0005, z_end - 0.0005, z_end, n,
                cap0=False, cap1=True),
    ])

PARTS = (
    ("spindle_mount", build_mount, "转接法兰 Ø88×14"),
    ("spindle_body", build_body, "机身 Ø80×200"),
    ("spindle_nose", build_nose, "前轴承座 Ø60×22"),
    ("spindle_collet_nut", build_nut, "ER20 夹头螺母 Ø34×17"),
    ("milling_tool", build_tool, "Ø10 立铣刀 72 mm（柄 40 + 刃 32）"),
)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="生成电主轴 + 铣刀的 STL 网格")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="输出根目录")
    ap.add_argument("--n", type=int, default=96, help="圆周分段数")
    ap.add_argument("--no-check", action="store_true", help="不做封闭性自检")
    args = ap.parse_args(argv)

    n = max(8, int(args.n))
    out = Path(args.out).expanduser().resolve()
    log(f"[网格] 圆周分段 n={n}，输出 {out}/{{visual,collision}}/")
    log("")

    bad = 0
    for name, builder, desc in PARTS:
        tris = builder(n)
        for kind in ("visual", "collision"):
            path = out / kind / f"{name}.stl"
            m = write_stl(tris, path, name)
            log(f"  {kind:<9} {path.name:<18} {m:5d} 三角形  "
                f"体积 {mesh_volume(tris) * 1e6:8.2f} cm³  包围盒 {mesh_extent_mm(tris)} mm")
        if not args.no_check:
            n_edges, n_open = edge_report(tris)
            if n_open:
                bad += 1
                log(f"  [warn] {name}: {n_open}/{n_edges} 条边不是恰好两个三角形共用，网格不封闭")
            else:
                log(f"  [ok]   {name}: 封闭（{n_edges} 条边）  {desc}")
        log("")

    if bad:
        log(f"[warn] 有 {bad} 个零件网格不封闭，V-HACD / 碰撞可能异常")
        return 1
    log(f"[完成] {len(PARTS)} 个零件 × 2 份（visual / collision）= {2 * len(PARTS)} 个 STL")
    return 0


if __name__ == "__main__":
    sys.exit(main())

