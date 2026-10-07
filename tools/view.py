#!/usr/bin/env python3
"""scan.csv (x_m,y_m,intensity) を表示するビューア。

使い方:
    python3 tools/view_scan.py scan.csv              # 表示
    python3 tools/view_scan.py scan.csv --watch      # ファイル更新を監視して自動再描画
    python3 tools/view_scan.py scan.csv --save a.png # 画像に保存 (画面は開かない)
    python3 tools/view_scan.py scan.csv --range 8    # 表示範囲を ±8 m に固定

依存: numpy, matplotlib  (pip install numpy matplotlib)
"""
import argparse
import math
import os
import sys

import numpy as np


def load(path):
    """CSV を読んで (x, y, intensity) の配列を返す。空・読み取り途中のファイルは空配列。"""
    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    except (OSError, ValueError):
        return np.empty(0), np.empty(0), np.empty(0)
    if data.size == 0 or data.shape[1] < 3:
        return np.empty(0), np.empty(0), np.empty(0)
    return data[:, 0], data[:, 1], data[:, 2]


def nice_step(limit):
    """距離リングの間隔を見やすい値 (0.5/1/2/5 m ...) から選ぶ。"""
    raw = limit / 4
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


def main():
    ap = argparse.ArgumentParser(description="2D LiDAR scan.csv viewer")
    ap.add_argument("csv", nargs="?", default="scan.csv")
    ap.add_argument("--watch", action="store_true", help="ファイルが更新されたら再読み込み")
    ap.add_argument("--save", metavar="PNG", help="画像として保存して終了")
    ap.add_argument("--range", type=float, metavar="M", help="表示範囲 ±M [m] (省略時は自動)")
    args = ap.parse_args()

    import matplotlib

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    x, y, inten = load(args.csv)
    if x.size == 0 and not args.watch:
        sys.exit(f"{args.csv}: 点が読み込めませんでした")

    if args.range:
        limit = args.range
    elif x.size:
        limit = max(1.0, math.ceil(float(np.hypot(x, y).max())))
    else:
        limit = 6.0

    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(x, y, c=inten, s=5, cmap="viridis", vmin=0, vmax=255)
    fig.colorbar(sc, ax=ax, label="intensity", shrink=0.8)

    # 距離リング
    step = nice_step(limit)
    r = step
    while r <= limit * 1.001:
        ax.add_patch(plt.Circle((0, 0), r, fill=False, ls="--", lw=0.6, color="gray", alpha=0.6))
        ax.text(r * 0.707, r * 0.707, f"{r:g} m", color="gray", fontsize=8)
        r += step
    ax.axhline(0, color="gray", lw=0.4)
    ax.axvline(0, color="gray", lw=0.4)
    ax.plot(0, 0, "r^", ms=9, label="sensor")  # センサ位置

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right")

    def update_title(n):
        ax.set_title(f"{os.path.basename(args.csv)}  ({n} points)")

    update_title(x.size)

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"saved {args.save}")
        return

    if args.watch:
        state = {"mtime": os.path.getmtime(args.csv) if os.path.exists(args.csv) else 0}

        def refresh(_):
            try:
                m = os.path.getmtime(args.csv)
            except OSError:
                return
            if m == state["mtime"]:
                return
            state["mtime"] = m
            nx, ny, ni = load(args.csv)
            if nx.size == 0:
                return
            sc.set_offsets(np.column_stack([nx, ny]))
            sc.set_array(ni)
            update_title(nx.size)
            fig.canvas.draw_idle()

        anim = FuncAnimation(fig, refresh, interval=300, cache_frame_data=False)  # noqa: F841

    plt.show()


if __name__ == "__main__":
    main()