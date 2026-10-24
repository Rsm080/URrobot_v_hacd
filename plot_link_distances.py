#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 piece_distances_all_samples.csv 画成 "每根手臂连杆的距离-时间" 六宫格。

数据来源
--------
读 link_piece_distance_matrix.py 的输出（行 = 凸块，列 = 各采样时刻）：

    按连杆在同一时刻对所有凸块取 min  ->  d_link(t)
    （和 link_distance_to_obstacle.csv 的算法一致，所以最小的那几个数能对上）

默认画 6 根手臂连杆（不含基座、不含夹爪）：

    shoulder_link   upper_arm_link   forearm_link
    wrist_1_link    wrist_2_link     wrist_3_link

基座（base_link）和末端（ee_link、robotiq_*、*finger*、*knuckle*、*pad*，以及电主轴的
spindle_mount_link / spindle_link / tool_link）默认被过滤掉；想画别的连杆就用 --links
显式点名（点名后不再过滤）。

每张图里
--------
    粗彩线  = 该连杆的 d_link(t)（其所有凸块取 min）
    细灰线  = 该连杆各凸块自己的 d_piece(t)，用来看 "是哪一块贴得最近"
    灰虚线  = --warn 警戒距离（默认 0.05 m = 5 cm）
    圆点    = 全程最小值的位置，旁边标注数值和时刻

用法
----
    python plot_link_distances.py                     # 读 distance_matrix_out/，存 PNG
    python plot_link_distances.py --show              # 画完弹窗（需要 $DISPLAY）
    python plot_link_distances.py --warn 0.1          # 10 cm 警戒线
    python plot_link_distances.py --no-pieces         # 只看连杆 min，不要每块的细线
    python plot_link_distances.py --no-shared-y       # 每张图各自缩放纵轴（看细节）
    python plot_link_distances.py --out /tmp/d.png --dpi 200
    python plot_link_distances.py --links shoulder_link,forearm_link
    python plot_link_distances.py --save-csv /tmp/link_distances_vs_t.csv

注意
----
* 列头 `t=0.033` 是**归一化相位**（一个运动周期 = 1），不是秒；
  换算关节角：q_k = home_k + amp*sin(2*pi*freq*t + k*phase_step)。
* CSV 里的距离只保留 4 位小数；要原始精度就直接读 distances.npz 的 piece_distances。
* 中文字体自动找 Noto Sans CJK / 文泉驿 / 微软雅黑…；一个都没有时图上自动改英文，
  也可以用 --font / --lang 指定。
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "distance_matrix_out" / "piece_distances_all_samples.csv"
DEFAULT_PNG = ROOT / "distance_matrix_out" / "link_distances_over_time.png"

# 默认不画的连杆：基座 + 末端（夹爪手指/指节/衬垫；电主轴法兰/机身/铣刀）
NON_ARM = re.compile(
    r"(base|obstacle|ee_link|tool|tcp|flange|gripper|robotiq|finger|knuckle|pad|hand"
    r"|spindle|collet|chuck|mill|cutter|bit)",
    re.IGNORECASE,
)

# 中文字体候选（按优先级）
CJK_FONTS = (
    "Noto Sans CJK SC", "Noto Sans CJK TC", "Noto Sans CJK JP",
    "Source Han Sans SC", "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
    "Microsoft YaHei", "SimHei", "PingFang SC", "Heiti SC",
    "Droid Sans Fallback", "AR PL UMing CN",
)

# 6 根连杆用的配色
PALETTE = ["#d1495b", "#3a7ca5", "#2e8b57", "#e08a1e", "#7b52ab", "#1f9e9e",
           "#8c564b", "#c2477f"]

TEXT = {
    "zh": {
        "title": "各连杆 → 障碍物 最小距离（t = 归一化相位，一个运动周期 = 1）",
        "xlabel": "归一化时间 t",
        "ylabel": "到障碍物的距离 [m]",
        "legend_link": "连杆（各凸块取 min）",
        "legend_pieces": "该连杆的各个凸块",
        "legend_warn": "警戒线 {v:.1f} cm",
        "footnote": "数据：{name}（行=凸块、列=时刻；本图按连杆对凸块取 min）",
    },
    "en": {
        "title": "Per-link minimum distance to obstacle (t = normalized phase, one cycle = 1)",
        "xlabel": "normalized time t",
        "ylabel": "distance to obstacle [m]",
        "legend_link": "link (min over its pieces)",
        "legend_pieces": "individual pieces",
        "legend_warn": "warning {v:.1f} cm",
        "footnote": "source: {name} (rows = pieces, cols = time; min over pieces per link)",
    },
}


def log(msg: str = "") -> None:
    """和 link_piece_distance_matrix.py 一样的打印风格（立即 flush）。"""
    print(msg, flush=True)


def parse_names(text: str) -> list[str]:
    """--links shoulder_link,forearm_link -> ['shoulder_link', 'forearm_link']"""
    return [n.strip() for n in str(text).split(",") if n.strip()]


# ---------------------------------------------------------------------------
# 读 CSV
# ---------------------------------------------------------------------------
def read_samples_csv(
        path: Path) -> tuple[np.ndarray, list[str], dict[str, list[tuple[str, np.ndarray]]]]:
    """-> (t, 连杆顺序, {连杆: [(凸块标签, 各时刻距离), ...]})"""
    with open(path, newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.reader(fh) if r]
    if len(rows) < 2:
        raise ValueError(f"{path} 里没有数据行")

    header = [h.strip() for h in rows[0]]
    if header[:2] != ["piece", "link"]:
        raise ValueError(f"{path} 表头不对：前两列应是 piece,link，实际是 {header[:2]}")
    try:
        t = np.array([float(h.split("=", 1)[1]) for h in header[2:]], dtype=np.float64)
    except (IndexError, ValueError) as exc:
        raise ValueError(
            f"{path} 的时刻列头应形如 t=0.033，实际是 {header[2:4]}") from exc
    if len(t) == 0:
        raise ValueError(f"{path} 里没有任何时刻列")

    order: list[str] = []
    groups: dict[str, list[tuple[str, np.ndarray]]] = {}
    for row in rows[1:]:
        if len(row) < 3:
            continue
        piece, link = row[0].strip(), row[1].strip()
        values = np.array([float(v) for v in row[2:2 + len(t)]], dtype=np.float64)
        if len(values) != len(t):
            raise ValueError(f"{path}: {piece} 这行只有 {len(values)} 个数，应有 {len(t)} 个")
        groups.setdefault(link, []).append((piece, values))
        if link not in order:
            order.append(link)

    if not order:
        raise ValueError(f"{path} 里没有解析到任何凸块")
    return t, order, groups


def select_links(order: list[str], groups: dict, wanted: list[str] | None) -> list[str]:
    """--links 点名优先；否则自动排除基座 / 夹爪，剩下的当手臂连杆。"""
    if wanted:
        picked = []
        for name in wanted:
            if name in groups:
                picked.append(name)
            else:
                log(f"[warn] --links 里的 {name} 不在 CSV 中，已忽略")
        if not picked:
            raise ValueError("--links 里的连杆一个都没找到；CSV 里可选: " + ", ".join(order))
        return picked

    picked = [ln for ln in order if not NON_ARM.search(ln)]
    if not picked:
        raise ValueError("没自动识别出手臂连杆，请用 --links 点名；CSV 里可选: "
                         + ", ".join(order))
    return picked


def link_series(groups: dict, links: list[str]) -> dict[str, tuple[np.ndarray, list]]:
    """每根连杆 -> (d_link(t) = 各凸块取 min, [(凸块标签, d_piece(t)), ...])"""
    out = {}
    for name in links:
        entries = groups[name]
        mat = np.vstack([values for _piece, values in entries])
        out[name] = (mat.min(axis=0), entries)
    return out


# ---------------------------------------------------------------------------
# 字体 / 语言 / 后端
# ---------------------------------------------------------------------------
def load_pyplot(show: bool):
    """按需选后端：只存图片时用 Agg，不需要 $DISPLAY。"""
    try:
        import matplotlib
    except ModuleNotFoundError as exc:                     # pragma: no cover
        raise SystemExit("[error] 需要 matplotlib：python -m pip install matplotlib") from exc
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def setup_font(plt, requested: str | None, lang: str) -> str:
    """挑字体并返回图上文字语言（'zh' / 'en'）。"""
    from matplotlib import font_manager

    available = {f.name for f in font_manager.fontManager.ttflist}
    font = requested if (requested and requested in available) else None
    if requested and font is None:
        log(f"[warn] 系统里没有字体 {requested!r}，改为自动挑选")
    if font is None:
        font = next((n for n in CJK_FONTS if n in available), None)

    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["axes.unicode_minus"] = False            # 负号别显示成方块
    if font is None:
        if lang == "zh":
            log("[warn] 没找到中文字体（装 fonts-noto-cjk 即可），图上改用英文标注")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
        return "en"

    plt.rcParams["font.sans-serif"] = [font, "DejaVu Sans"]
    log(f"[字体] {font}")
    return "zh" if lang in ("auto", "zh") else "en"


# ---------------------------------------------------------------------------
# 画图
# ---------------------------------------------------------------------------
def draw_panel(ax, t: np.ndarray, name: str, curve: np.ndarray, entries: list,
               color: str, T: dict, warn: float, show_pieces: bool,
               ylim: tuple[float, float] | None = None) -> None:
    if show_pieces and len(entries) > 1:                  # 该连杆每个凸块自己的曲线
        for k, (_piece, values) in enumerate(entries):
            ax.plot(t, values, color="0.62", lw=0.8, alpha=0.8, zorder=1,
                    label=T["legend_pieces"] if k == 0 else None)

    ax.plot(t, curve, color=color, lw=2.0, zorder=3, label=T["legend_link"])

    lo, hi = float(curve.min()), float(curve.max())
    if ylim is None:                                     # 每个面板各自缩放
        pad = max(0.01, 0.12 * (hi - lo))
        bottom = (min(lo, warn) - pad) if warn > 0.0 else (lo - pad)
        ax.set_ylim(max(-0.02, bottom), hi + pad)
    else:                                                # 6 个面板共用同一把尺子
        ax.set_ylim(*ylim)
    ax.set_xlim(float(t[0]), max(1.0, float(t[-1])))

    k = int(np.argmin(curve))                            # 全程最小值：圆点 + 标注
    ax.plot([t[k]], [curve[k]], "o", ms=5.5, color=color, zorder=4)

    # 标注往"空白多"的一侧放：按点在轴里的高度决定写上面还是下面
    y0, y1 = ax.get_ylim()
    frac = (curve[k] - y0) / ((y1 - y0) or 1.0)
    dy = -15 if frac > 0.55 else 13
    dx = -8 if t[k] > 0.85 else 8
    ax.annotate(f"min {curve[k]:.3f} m @ t={t[k]:.3f}",
                xy=(t[k], curve[k]), xytext=(dx, dy), textcoords="offset points",
                ha="right" if dx < 0 else "left",
                va="top" if dy < 0 else "bottom",
                fontsize=7.5, color="0.15",
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.78),
                arrowprops=dict(arrowstyle="->", lw=0.7, color="0.4"))

    if warn > 0.0:
        ax.axhline(warn, ls="--", lw=1.0, color="0.45", zorder=2,
                   label=T["legend_warn"].format(v=warn * 100.0))

    ax.set_title(name, fontsize=10.5, pad=6)
    ax.grid(True, ls=":", lw=0.6, color="0.75")
    ax.tick_params(labelsize=8.5)
    ax.legend(fontsize=7, loc="best", framealpha=0.85, borderpad=0.3)


def plot_curves(plt, t: np.ndarray, series: dict, links: list[str],
                warn: float, show_pieces: bool, T: dict, source: str,
                shared_y: bool = True):
    n = len(links)
    ncols = min(3, n)
    nrows = int(math.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.6 * ncols, 3.2 * nrows),
                             squeeze=False, sharex=True)

    ylim = None
    if shared_y:                                          # 统一纵轴，6 根杆才好横向比
        gmin = min(float(series[name][0].min()) for name in links)
        gmax = max(float(series[name][0].max()) for name in links)
        low = min(gmin, warn) if warn > 0.0 else gmin
        pad = max(0.02, 0.08 * (gmax - low))
        ylim = (max(-0.02, low - pad), gmax + pad)

    for k, name in enumerate(links):
        curve, entries = series[name]
        draw_panel(axes.flat[k], t, name, curve, entries,
                   PALETTE[k % len(PALETTE)], T, warn, show_pieces, ylim)

    for ax in axes.flat[n:]:                              # 多出来的格子留空
        ax.set_visible(False)

    # x 标签只给底行、y 标签只给左列（比 supxlabel 稳，脚注不会和它挤在一起）
    for ax in axes.flat[:n]:
        spec = ax.get_subplotspec()
        if spec.rowspan.start == axes.shape[0] - 1:
            ax.set_xlabel(T["xlabel"], fontsize=9.5)
        if spec.colspan.start == 0:
            ax.set_ylabel(T["ylabel"], fontsize=9.5)

    fig.suptitle(T["title"], fontsize=12.5)
    fig.text(0.995, 0.004, T["footnote"].format(name=source),
             ha="right", va="bottom", fontsize=7.5, color="0.35")
    fig.tight_layout(rect=(0.01, 0.03, 1.0, 0.95))
    return fig


# ---------------------------------------------------------------------------
# 控制台汇总 / 可选导出
# ---------------------------------------------------------------------------
def print_summary(t: np.ndarray, series: dict, links: list[str], warn: float) -> None:
    log("-" * 84)
    log(f"  {'link':<26}{'d(t=0)':>9}{'min [m]':>10}{'t_min':>8}  closest piece")
    rows = []
    for name in links:
        curve, entries = series[name]
        k = int(np.argmin(curve))
        piece = min(entries, key=lambda e: float(e[1][k]))[0]
        rows.append((name, float(curve[0]), float(curve[k]), float(t[k]), piece))
        log(f"  {name:<26}{curve[0]:>9.4f}{curve[k]:>10.4f}{t[k]:>8.3f}  {piece}")

    best = min(rows, key=lambda r: r[2])
    log(f"  -> 全程最近: {best[0]}  {best[2]:.4f} m @ t={best[3]:.3f}（{best[4]}）")
    if warn > 0.0:
        risky = [r[0] for r in rows if r[2] < warn]
        if risky:
            log(f"  [warn] {len(risky)} 根连杆进入 {warn * 100.0:.1f} cm 警戒区: "
                + ", ".join(risky))
    log("-" * 84)


def save_link_csv(path: Path, t: np.ndarray, series: dict, links: list[str]) -> None:
    """行 = 连杆、列 = 时刻（各凸块已取 min），方便再拿去画别的图。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["link"] + [f"t={float(x):.3f}" for x in t])
        for name in links:
            writer.writerow([name] + [f"{float(v):.4f}" for v in series[name][0]])


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="把 piece_distances_all_samples.csv 画成每根手臂连杆的 距离-时间 六宫格。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--csv", default=str(DEFAULT_CSV),
                    help="输入的 piece_distances_all_samples.csv")
    ap.add_argument("--out", default=str(DEFAULT_PNG), help="输出 PNG 路径")
    ap.add_argument("--links", type=parse_names, default=None,
                    help="要画的连杆（逗号分隔）；缺省自动取手臂连杆，排除基座和夹爪")
    ap.add_argument("--warn", type=float, default=0.05,
                    help="警戒距离 (m)，<=0 表示不画虚线")
    ap.add_argument("--pieces", action=argparse.BooleanOptionalAction, default=True,
                    help="是否叠加该连杆各凸块自己的曲线（细灰线）")
    ap.add_argument("--shared-y", action=argparse.BooleanOptionalAction, default=True,
                    help="6 张图共用同一纵轴范围（便于横向比较）；关掉则各自缩放看细节")
    ap.add_argument("--dpi", type=int, default=160, help="PNG 分辨率")
    ap.add_argument("--save-csv", default=None,
                    help="另存一份 \"行=连杆、列=时刻\" 的聚合 CSV")
    ap.add_argument("--show", action="store_true", help="画完弹窗显示（需要 $DISPLAY）")
    ap.add_argument("--font", default=None, help="指定字体名（缺省自动找中文字体）")
    ap.add_argument("--lang", choices=("auto", "zh", "en"), default="auto",
                    help="图上文字语言")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    csv_path = Path(args.csv).expanduser()
    if not csv_path.is_file():
        log(f"[error] 找不到 {csv_path}")
        log("        先跑一次 python link_piece_distance_matrix.py 生成它")
        return 2

    try:
        t, order, groups = read_samples_csv(csv_path)
        links = select_links(order, groups, args.links)
    except ValueError as exc:
        log(f"[error] {exc}")
        return 2

    series = link_series(groups, links)
    n_pieces = sum(len(v) for v in groups.values())
    log(f"[数据] {csv_path}")
    log(f"       {len(t)} 个采样点，{n_pieces} 个凸块，{len(order)} 个连杆")
    log(f"[连杆] 本次画 {len(links)} 根（不含基座 / 夹爪）: " + ", ".join(links))
    print_summary(t, series, links, float(args.warn))

    plt = load_pyplot(bool(args.show))
    lang = setup_font(plt, args.font, args.lang)
    fig = plot_curves(plt, t, series, links, float(args.warn),
                      bool(args.pieces), TEXT[lang], csv_path.name,
                      bool(args.shared_y))

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=int(args.dpi), bbox_inches="tight", pad_inches=0.2)
    log(f"[输出] PNG: {out}（{len(links)} 个子图，{args.dpi} dpi）")

    if args.save_csv:
        csv_out = Path(args.save_csv).expanduser()
        save_link_csv(csv_out, t, series, links)
        log(f"[输出] 聚合 CSV: {csv_out}（行=连杆、列=时刻）")

    if args.show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
