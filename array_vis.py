#!/usr/bin/env python3
"""
Systolic array matmul visualizer  (output-stationary).

Computes C = A @ B for A:(M,K), B:(K,N) on an ARRAY_N x ARRAY_N grid of PEs,
rendering one frame per cycle. Operands are shown SYMBOLICALLY (a1,a2,... and
b1,b2,...), not as numeric values.

Indexing (matches the common lecture animation):
  A is displayed/labelled ROW-MAJOR:     A[i,t] -> a_{i*K + t + 1}
  B is displayed/labelled COLUMN-MAJOR:  B[t,j] -> b_{t + j*K + 1}
So for 3x3, A's first row is a1 a2 a3 and B's first column is b1 b2 b3.

Dataflow (output-stationary):
  PE(i,j) is stationary and accumulates C[i,j].
  'a' operands stream left->right along row i; 'b' operands stream top->bottom
  along column j. Operand pair (A[i,t], B[t,j]) meets at PE(i,j) on cycle
  t + i + j. Each PE does one MAC per cycle it has a valid pair.

The left/top panels show the FULL SKEW SCHEDULE as staggered staircases
(row i of A shifted right by i; column j of B shifted down by j), with the
current cycle's wavefront highlighted.

Requires ARRAY_N >= M and ARRAY_N >= N (tiling left as a TODO).

Usage:
  python array_vis.py --array-n 3 --M 3 --K 3 --N 3 --gif
  python array_vis.py --array-n 4 --M 2 --K 5 --N 3 --mp4 --fps 3
"""
import argparse, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BG      = "#7fd6b0"
PE_FILL = "#ffffff"
PE_DEAD = "#e8e8e8"
PE_EDGE = "#111111"
A_COL   = "#d13b2f"
B_COL   = "#1f3d99"
ACC_COL = "#5b2a86"
HL      = "#ff9500"
DIM     = "#9a9a9a"


def a_label(i, t, K):
    return f"a{i * K + t + 1}"


def b_label(t, j, K):
    return f"b{t + j * K + 1}"


def build_events(M, K, N, array_n):
    assert array_n >= M and array_n >= N, (
        f"array-n={array_n} too small for output {M}x{N}; tiling not implemented")
    last_cycle = (K - 1) + (M - 1) + (N - 1)
    events = [dict() for _ in range(last_cycle + 1)]
    for i in range(M):
        for j in range(N):
            for t in range(K):
                events[t + i + j][(i, j)] = t
    return events, last_cycle


def draw_matrix_panel(ax, x0, y0, mat_shape, kind, K):
    rows, cols = mat_shape
    s = 0.42
    ax.text(x0, y0 + 0.5, kind, color=(A_COL if kind == "A" else B_COL),
            fontsize=13, fontweight="bold")
    for r in range(rows):
        for c in range(cols):
            x = x0 + c * s
            y = y0 - r * s
            ax.add_patch(Rectangle((x, y - s), s, s, facecolor="white",
                                   edgecolor="#cccccc", lw=1))
            lab = a_label(r, c, K) if kind == "A" else b_label(r, c, K)
            col = A_COL if kind == "A" else B_COL
            ax.text(x + s/2, y - s/2, lab, ha="center", va="center",
                    color=col, fontsize=8)
    return y0 - rows * s


def draw_skew_panel(ax, kind, M, K, N, cur_cycle):
    cell = 1.0
    if kind == "A":
        for i in range(M):
            y = -i * cell - cell/2
            for t in range(K):
                slot = t + i
                x = -1.2 - slot * 0.62
                entered = slot < cur_cycle
                active = slot == cur_cycle
                col = DIM if entered else A_COL
                fw = "bold" if active else "normal"
                ax.text(x, y, a_label(i, t, K), ha="center", va="center",
                        color=col, fontsize=9, fontweight=fw)
                if active:
                    ax.add_patch(Rectangle((x - 0.28, y - 0.18), 0.56, 0.36,
                                 facecolor="none", edgecolor=HL, lw=2))
    else:
        for j in range(N):
            x = j * cell + cell/2
            for t in range(K):
                slot = t + j
                y = 1.2 + slot * 0.55
                entered = slot < cur_cycle
                active = slot == cur_cycle
                col = DIM if entered else B_COL
                fw = "bold" if active else "normal"
                ax.text(x, y, b_label(t, j, K), ha="center", va="center",
                        color=col, fontsize=9, fontweight=fw)
                if active:
                    ax.add_patch(Rectangle((x - 0.28, y - 0.18), 0.56, 0.36,
                                 facecolor="none", edgecolor=HL, lw=2))


def draw_frame(c, events, acc_terms, M, K, N, array_n, macs, last_cycle, outpath):
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
    ax.set_xlim(-K*0.62 - 3.5, array_n + 1.0)
    ax.set_ylim(-array_n - 1.8, K*0.55 + 3.0)
    ax.set_aspect("equal"); ax.axis("off")
    cell = 1.0

    def cell_xy(i, j):
        return (j * cell, -i * cell)

    active = events[c]

    for i in range(array_n):
        for j in range(array_n):
            x, y = cell_xy(i, j)
            live = (i < M and j < N)
            ax.add_patch(Rectangle((x, y - cell), cell, cell,
                         facecolor=PE_FILL if live else PE_DEAD,
                         edgecolor=PE_EDGE, lw=2))
            if not live:
                continue
            ax.text(x + 0.06, y - 0.10, f"P{i}{j}", ha="left", va="top",
                    color=DIM, fontsize=7)
            terms = acc_terms[(i, j)]
            shown = terms[-3:]
            for r, term in enumerate(shown):
                ax.text(x + cell/2, y - 0.32 - r*0.19, term, ha="center",
                        va="center", color=ACC_COL, fontsize=8)
            if len(terms) > 3:
                ax.text(x + cell/2, y - 0.24, "...", ha="center", va="center",
                        color=DIM, fontsize=7)
            if (i, j) in active:
                t = active[(i, j)]
                ax.add_patch(Rectangle((x, y - cell), cell, cell,
                             facecolor="none", edgecolor=HL, lw=3))
                ax.text(x + cell/2, y - cell + 0.15,
                        f"{a_label(i,t,K)}*{b_label(t,j,K)}", ha="center",
                        va="center", color="black", fontsize=8, fontweight="bold")

    draw_skew_panel(ax, "A", M, K, N, c)
    draw_skew_panel(ax, "B", M, K, N, c)

    top_y = K*0.55 + 2.4
    left_x = -K*0.62 - 3.3
    yA = draw_matrix_panel(ax, left_x, top_y, (M, K), "A", K)
    draw_matrix_panel(ax, left_x, yA - 0.7, (K, N), "B", K)

    ax.text(-1.9, -0.5, "A ->", color=A_COL, fontsize=11, fontweight="bold")
    ax.text(-0.4, 1.15 + K*0.55, "B v", color=B_COL, fontsize=11, fontweight="bold")

    done = (c == last_cycle)
    hud = (f"cycle: {c} / {last_cycle}    MACs: {macs} / {M*K*N}\n"
           f"array {array_n}x{array_n}   C = A({M}x{K})*B({K}x{N})")
    ax.text(left_x, -array_n - 0.6, hud, ha="left", va="top",
            color="#333333", fontsize=11, family="monospace")
    if done:
        ax.text(left_x, -array_n - 1.3, "RESULT COMPLETE", color="#0a7d3a",
                fontsize=13, fontweight="bold")

    fig.savefig(outpath, dpi=110, bbox_inches="tight", facecolor=BG)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--array-n", type=int, default=3)
    ap.add_argument("--M", type=int, default=3)
    ap.add_argument("--K", type=int, default=3)
    ap.add_argument("--N", type=int, default=3)
    ap.add_argument("--out", default="frames")
    ap.add_argument("--mp4", action="store_true")
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--fps", type=int, default=2)
    args = ap.parse_args()
    M, K, N = args.M, args.K, args.N

    try:
        events, last_cycle = build_events(M, K, N, args.array_n)
    except AssertionError as e:
        print(f"error: {e}", file=sys.stderr); sys.exit(1)

    os.makedirs(args.out, exist_ok=True)
    acc_terms = {(i, j): [] for i in range(M) for j in range(N)}
    macs = 0
    paths = []
    for c in range(last_cycle + 1):
        for (i, j), t in events[c].items():
            acc_terms[(i, j)].append(f"{a_label(i,t,K)}{b_label(t,j,K)}")
            macs += 1
        p = os.path.join(args.out, f"frame_{c:03d}.png")
        draw_frame(c, events, acc_terms, M, K, N, args.array_n, macs,
                   last_cycle, p)
        paths.append(p)

    for (i, j), terms in acc_terms.items():
        assert len(terms) == K, f"PE{i}{j} got {len(terms)} terms, expected {K}"
    print(f"wrote {len(paths)} frames to {args.out}/  (cycles 0..{last_cycle})")

    if args.mp4:
        import subprocess
        mp4 = os.path.join(args.out, "systolic.mp4")
        subprocess.run(["ffmpeg", "-y", "-framerate", str(args.fps),
                        "-i", os.path.join(args.out, "frame_%03d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4],
                       check=True, capture_output=True)
        print(f"wrote {mp4}")
    if args.gif:
        try:
            from PIL import Image
            imgs = [Image.open(p) for p in paths]
            gif = os.path.join(args.out, "systolic.gif")
            imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                         duration=int(1000/args.fps), loop=0)
            print(f"wrote {gif}")
        except ImportError:
            print("gif skipped: pip install pillow")


# --- TODO: tiling for ARRAY_N < M or N ---
# Loop output tiles of size <=ARRAY_N; accumulate over K per tile; enforce the
# 2*(N-1) drain gap between back-to-back tiles.

if __name__ == "__main__":
    main()