"""Draw docs/stanley_algorithm_{1_flow,2_steering,3_speed}.png from the current
config (path_tracking.yaml / vehicle.yaml values are written in by hand below;
re-run after changing them). Speed profile: the package's own speed_profile()
on a map's waypoints.csv.

    python3 docs/make_algorithm_figures.py [map_dir]
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

from krowd_stanley_control.route_geometry import path_curvatures, segment_lengths  # noqa: E402
from krowd_stanley_control.speed_profile import SpeedLimits, speed_profile  # noqa: E402

from matplotlib import font_manager  # noqa: E402

for _font in ("NotoSansCJK-Regular.ttc", "NotoSansCJK-Bold.ttc"):
    _path = Path("/usr/share/fonts/opentype/noto") / _font
    if _path.exists():
        font_manager.fontManager.addfont(str(_path))
# The .ttc registers under its first face's name (JP); it has the Hangul too.
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
OUT = Path(__file__).resolve().parent
MAP = Path(sys.argv[1] if len(sys.argv) > 1 else
           "/home/krowd/krowd2026_ws/src/krowd_slam/krowd_maps/20260927_211309")

INK = "#1f2933"
MUTED = "#52606d"
BLUE = "#2f6fb3"
BLUE_BG = "#e8f1fb"
GREEN = "#2e7d4f"
GREEN_BG = "#e7f5ec"
RED = "#b83a3a"
RED_BG = "#fbeaea"
AMBER = "#9a6700"
AMBER_BG = "#fff4d6"
GREY_BG = "#f2f4f6"


def box(ax, x, y, w, h, title, body="", fc=BLUE_BG, ec=BLUE, tsize=12, bsize=10):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.003,rounding_size=0.01",
                                fc=fc, ec=ec, lw=1.4))
    if body:
        ax.text(x + 0.015, y + h - 0.018, title, ha="left", va="top", fontsize=tsize,
                weight="bold", color=INK)
        ax.text(x + 0.015, y + h - 0.018 - 0.028 * tsize / 12, body, ha="left", va="top",
                fontsize=bsize, color=INK, linespacing=1.35)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=tsize,
                weight="bold", color=INK)


def arrow(ax, x0, y0, x1, y1, color=MUTED, text="", tx=0.0, ty=0.0, style="-|>", ls="-"):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=style, mutation_scale=14,
                                 color=color, lw=1.5, linestyle=ls))
    if text:
        ax.text((x0 + x1) / 2 + tx, (y0 + y1) / 2 + ty, text, fontsize=9.5, color=color,
                ha="center", va="center")


def canvas(w, h, title, subtitle):
    fig = plt.figure(figsize=(w, h), dpi=110)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    fig.patch.set_facecolor("white")
    ax.text(0.02, 0.975, title, fontsize=19, weight="bold", color=INK, va="top")
    ax.text(0.02, 0.938, subtitle, fontsize=11, color=MUTED, va="top")
    return fig, ax


# ---------------------------------------------------------------- 1. flow
def figure_flow():
    fig, ax = canvas(15, 13.5, "Stanley 경로 추종: 20 Hz 제어 주기",
                     "stanley_node가 50 ms마다 위에서 아래로 검사합니다. 빨간 칸에 걸리면 그 주기는 거기서 끝나고 "
                     "정지 명령(brake)을 냅니다.")
    # inputs
    box(ax, 0.02, 0.80, 0.30, 0.10, "입력 토픽",
        "/localization/vehicle_state  ESKF 위치·heading·속도·yaw rate\n"
        "/planning/trajectory  경로·곡률·목표속도 (route_publisher)\n"
        "/encoder  실제 조향각     /ekf/heading_verified", fc=GREY_BG, ec=MUTED, bsize=9.5)
    box(ax, 0.35, 0.80, 0.30, 0.10, "route_publisher_node",
        "맵의 waypoints.csv (RTK로 기록, 약 1 m 간격)\n"
        "→ 곡률(5점 평활) → 속도 프로파일 (3번 그림)\n"
        "→ /planning/trajectory 로 발행", fc=GREY_BG, ec=MUTED, bsize=9.5)
    box(ax, 0.68, 0.80, 0.30, 0.10, "출력 토픽",
        "/target_angle  조향 명령 (컬럼 쪽 rad)\n"
        "/target_speed  속도 명령 (km/h)\n"
        "/brake  정지 요청  →  ros_can_bridge → CAN", fc=GREY_BG, ec=MUTED, bsize=9.5)

    steps = [
        ("① 기본 조건", "목표 도달 래치 / 위치 없음·0.5 s 넘게 끊김 / 경로 없음·만료 / 좌표계 불일치",
         "STOP GOAL_LATCHED · STALE_VEHICLE_STATE · NO_TRAJECTORY ..."),
        ("② heading 검증 (ESKF)", "RTK 직진 course로 heading이 검증될 때까지:\n1 s 대기 → 7 km/h 직진 creep (최대 15 s / 15 m)",
         "CREEP HEADING_UNVERIFIED  →  실패 시 STOP HEADING_NOT_ACQUIRED"),
        ("③ 조향 피드백", "/encoder 가 없거나 0.25 s 넘게 끊기면 주행 불가",
         "STOP NO_STEERING_FEEDBACK / STALE_STEERING_FEEDBACK"),
        ("④ 경로 투영", "앞차축을 경로에 투영 → cte(좌우 오차), 남은 거리, 진행 위치 wp\n(직전 위치 근처만 탐색: 루프 끝·시작 혼동 방지)",
         ""),
        ("⑤ corridor ±3 m", "경로에서 3 m 넘게 벗어나면 복귀 모드",
         "RECOVERING: 6 m 앞 경로점으로 pure pursuit, 4 km/h\n1.5 m·30° 안으로 오면 복귀 완료, 20 s·8 m 넘으면 STOP"),
        ("⑥ 경로 끝", "남은 거리 1 m 이내 → 정지하고 래치\n(10 m 이상 벗어났다가 시작점으로 오면 해제)",
         "STOP GOAL_REACHED → GOAL_LATCHED"),
        ("⑦ 방향 확인", "경로 방향과 차 heading 차이 > 60°",
         "STOP HEADING_NOT_ALIGNED"),
    ]
    y = 0.700
    h = 0.074
    for i, (title, body, stop) in enumerate(steps):
        box(ax, 0.10, y, 0.50, h, title, body, bsize=9.3, tsize=11.5)
        if stop:
            is_rec = title.startswith("⑤") or title.startswith("②")
            box(ax, 0.66, y + 0.008, 0.32, h - 0.016, "", "", fc=AMBER_BG if is_rec else RED_BG,
                ec=AMBER if is_rec else RED)
            ax.text(0.675, y + h / 2, stop, fontsize=9.3, color=AMBER if is_rec else RED,
                    va="center", ha="left", linespacing=1.3)
            arrow(ax, 0.60, y + h / 2, 0.66, y + h / 2, color=AMBER if is_rec else RED)
        if i < len(steps) - 1:
            arrow(ax, 0.35, y, 0.35, y - 0.016, text="통과", tx=0.03)
        y -= h + 0.016
    y += h + 0.016
    # final: both computed every cycle that got this far
    arrow(ax, 0.30, y, 0.30, y - 0.028)
    arrow(ax, 0.50, y, 0.66, y - 0.028)
    box(ax, 0.10, y - 0.118, 0.42, 0.088, "⑧ 조향 계산 (2번 그림)",
        "Stanley + 곡률 feedforward + yaw rate 감쇠\n→ 좌우 한계 → 실제 바퀴 기준 lead 제한 → 속도 제한\n→ 조향표로 /target_angle",
        fc=GREEN_BG, ec=GREEN, bsize=9.5)
    box(ax, 0.55, y - 0.118, 0.43, 0.088, "⑨ 속도 계산 (3번 그림)",
        "경로 목표속도 (곡률·코너·감속 램프)\n→ 출발 램프 → speed trim (코너 R<15 m에서는 정지)\n→ /target_speed, brake=false",
        fc=GREEN_BG, ec=GREEN, bsize=9.5)
    ax.text(0.02, 0.018, "꺼져 있는 검사: ESKF–MOLA 맵 heading 비교 (map_heading_consistency_required: false, 2026-09-26). "
            "상태 문자열은 /stanley_node/status.", fontsize=9.5, color=MUTED)
    fig.savefig(OUT / "stanley_algorithm_1_flow.png")
    plt.close(fig)


# ---------------------------------------------------------------- 2. steering
def figure_steering():
    fig, ax = canvas(16, 10.5, "조향 계산: 무엇을 더하고, 어떻게 제한하나",
                     "각도는 모두 앞바퀴(road wheel) 기준. 양수 = 왼쪽. 값은 현재 path_tracking.yaml / vehicle.yaml.")
    # --- geometry sketch (left): a gentle left bend, vehicle right of it
    g = fig.add_axes([0.02, 0.10, 0.42, 0.74])
    g.set_aspect("equal")
    g.axis("off")
    R0 = 14.0
    th = [math.radians(a / 2) for a in range(-2 * 110, -2 * 55)]
    rx = [R0 * math.cos(t) for t in th]
    ry = [R0 * math.sin(t) + R0 for t in th]
    g.plot(rx, ry, color="#e4e7eb", lw=30, solid_capstyle="round", zorder=0)
    g.plot(rx, ry, color=INK, lw=2.2, zorder=1)
    g.text(rx[0] - 0.2, ry[0] + 0.9, "경로 (waypoints)", fontsize=10.5, color=INK, ha="center")
    L = 1.34
    yaw = math.radians(-12.0)
    F = (0.0, -1.6)
    Rr = (F[0] - L * math.cos(yaw), F[1] - L * math.sin(yaw))
    c, sn = math.cos(yaw), math.sin(yaw)
    corners = [(-0.55, -0.63), (1.91, -0.63), (1.91, 0.63), (-0.55, 0.63)]
    g.add_patch(Polygon([(Rr[0] + c * a - sn * b, Rr[1] + sn * a + c * b) for a, b in corners],
                        closed=True, fc=BLUE_BG, ec=BLUE, lw=1.6, zorder=2))
    g.plot(*Rr, "o", color=BLUE, ms=6, zorder=4)
    g.text(Rr[0] - 0.2, Rr[1] - 1.25, "뒤축 (base_link)", fontsize=9.5, color=BLUE, ha="center")
    g.plot(*F, "o", color=RED, ms=8, zorder=4)
    g.text(F[0] + 0.25, F[1] - 1.05, "앞축", fontsize=10.5, color=RED, weight="bold")
    P = (0.0, 0.0)  # projection of F onto the bend (its lowest point)
    g.plot(*P, "o", color=GREEN, ms=8, zorder=4)
    g.annotate("", xy=P, xytext=F, arrowprops=dict(arrowstyle="<->", color=RED, lw=1.8), zorder=5)
    g.text(0.15, -0.75, "e (cte)", color=RED, fontsize=11.5, weight="bold", ha="left", va="center")
    g.text(0.15, 0.35, "투영점", color=GREEN, fontsize=9.5)
    # route direction: chord 2 m behind to 2 m ahead of the projection
    a1, a2 = -math.pi / 2 - 2.0 / R0, -math.pi / 2 + 2.0 / R0
    p1 = (R0 * math.cos(a1), R0 * math.sin(a1) + R0)
    p2 = (R0 * math.cos(a2), R0 * math.sin(a2) + R0)
    g.plot(*zip(p1, p2), "s", color=GREEN, ms=5, zorder=4)
    g.annotate("", xy=(4.2, 0.0), xytext=P, arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2), zorder=5)
    g.text(2.2, 2.5, "경로 방향 = 투영점 앞뒤 2 m 두 점(■)을\n잇는 방향 (창 4 m)", color=GREEN,
           fontsize=9.5, va="bottom")
    g.annotate("", xy=(3.4, 0.08), xytext=(3.2, 2.4), arrowprops=dict(arrowstyle="-", color=GREEN, lw=0.8, ls=":"))
    tip = (F[0] + 4.0 * c, F[1] + 4.0 * sn)
    g.annotate("", xy=tip, xytext=F, arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2, ls="--"), zorder=5)
    g.text(tip[0] + 0.15, tip[1] - 0.2, "차 heading ψ", color=BLUE, fontsize=10, va="top")
    g.text(4.4, -1.0, "ψe", color=INK, fontsize=12, weight="bold", va="center")
    g.text(-7.0, 4.6, "이 그림: e < 0 (경로 오른쪽), ψe > 0\n→ 두 항 모두 왼쪽으로 조향", fontsize=10,
           color=INK, va="top")
    g.text(-7.0, 3.1, "κ = 경로 곡률 (1/반경, 왼쪽 +)\n경로 형상에서 계산 → 센서 잡음 없음", fontsize=9.5,
           color=MUTED, va="top")
    g.set_xlim(-7.5, 8.8)
    g.set_ylim(-3.2, 5.2)

    # --- formula (right)
    X = 0.46
    ax.text(X, 0.875, "δ (바퀴 조향각)  =  아래 네 항의 합", fontsize=15, weight="bold", color=INK, va="center")
    terms = [
        ("0.6 · ψe", "heading 항", "경로 방향과의 차이. 게인 0.6 (1.0이면 고속에서 흔들림)", BLUE_BG, BLUE),
        ("− atan( 0.3 · e / (v + 3.0 m/s) )", "cte 항", "좌우 오차를 줄임 (경로 왼쪽이면 오른쪽으로).\n저속에서 과민하지 않게 v에 10.8 km/h(3.0 m/s)를 더함", BLUE_BG, BLUE),
        ("+ 0.4 · atan( 1.34 m · κ )", "곡률 feedforward", "코너에 필요한 조향 중 heading 항이 못 채우는 40 %.\n"
         "없으면 그만큼을 cte로 벌려고 코너 바깥으로 밀림 (9/24: 1.9~2.5 m)", GREEN_BG, GREEN),
        ("+ g(v) · ( v·κ − r )", "yaw rate 감쇠", "경로가 요구하는 회전(v·κ)과 실제 회전(r)의 차이.\n"
         "g = 0.15 s, 15 km/h 위에서는 0.15 · 15/v (고속 떨림 방지)", GREEN_BG, GREEN),
    ]
    y = 0.845
    for formula, name, note, fc, ec in terms:
        box(ax, X, y - 0.085, 0.51, 0.08, "", "", fc=fc, ec=ec)
        ax.text(X + 0.012, y - 0.022, formula, fontsize=13, weight="bold", color=INK, va="center")
        ax.text(X + 0.37, y - 0.022, name, fontsize=10.5, color=ec, weight="bold", va="center")
        ax.text(X + 0.012, y - 0.058, note, fontsize=9, color=MUTED, va="center", linespacing=1.3)
        y -= 0.095
    # limit chain
    y -= 0.015
    ax.text(X, y, "그다음 제한 (순서대로)", fontsize=12.5, weight="bold", color=INK)
    chain = [
        ("좌우 한계", "오른쪽 −19.4° (R 3.81 m)\n왼쪽 +22.6° (R 3.21 m)"),
        ("lead 제한", "실제 바퀴(/encoder) 기준\n직선 ±5°, 코너 ±12°\n(R 50→15 m 사이 섞음)"),
        ("변화 속도", "명령 40°/s 이하\n(실제 모터 약 12°/s가 한계)"),
        ("조향표", "타이어각 → 컬럼 명령\n영점 −0.21 rad, ±143° 제한"),
    ]
    w = 0.113
    xx = X
    for i, (t, b) in enumerate(chain):
        box(ax, xx, y - 0.13, w, 0.1, t, b, fc=AMBER_BG, ec=AMBER, tsize=10.5, bsize=8.6)
        if i < len(chain) - 1:
            arrow(ax, xx + w + 0.002, y - 0.08, xx + w + 0.017, y - 0.08, color=AMBER)
        xx += w + 0.019
    ax.text(X, y - 0.165, "→ /target_angle  (20 Hz)", fontsize=12, weight="bold", color=GREEN)
    ax.text(X, 0.035, "e: 앞축 좌우 오차(왼쪽 +)   ψe: heading 오차   v: 속도   κ: 경로 곡률   r: 측정 yaw rate (/ekf/odometry)",
            fontsize=9.5, color=MUTED)
    fig.savefig(OUT / "stanley_algorithm_2_steering.png")
    plt.close(fig)


# ---------------------------------------------------------------- 3. speed
def figure_speed():
    fig, ax = canvas(16, 10, "속도 결정: 경로 프로파일 → 출발 램프 → speed trim",
                     f"아래 그래프는 현재 설정으로 {MAP.name} 경로를 계산한 결과 (package의 speed_profile() 그대로).")
    stages = [
        ("① 곡률 상한", "v ≤ √(0.5 m/s² · R)\nR < 30 m 코너는 최대 10 km/h\n하한 10 km/h, 직선 최대 20 km/h"),
        ("② 코너 앞뒤", "코너 3 m 전에 코너 속도 도달\n코너 끝 3 m 전부터 가속 허용"),
        ("③ 감속·가속 램프", "감속 0.5 m/s² (타력 주행 기준)\n가속 2.5 m/s²\n경로 끝에서는 0까지 (STOP_AT_END)"),
        ("④ 출발 램프", "출발 시 5 km/h부터\n1 m/s²로 올림"),
        ("⑤ speed trim", "실제 속도가 목표보다 낮으면\n최대 +5 km/h 더 요청 (0.3 /s 적분)\n경로 R < 15 m에서는 더하지 않음"),
    ]
    w = 0.182
    x = 0.02
    for i, (t, b) in enumerate(stages):
        fc, ec = (BLUE_BG, BLUE) if i < 3 else (GREEN_BG, GREEN)
        box(ax, x, 0.75, w - 0.012, 0.13, t, b, fc=fc, ec=ec, bsize=9.5)
        if i < len(stages) - 1:
            arrow(ax, x + w - 0.012, 0.815, x + w + 0.001, 0.815)
        x += w + 0.013
    ax.text(0.02, 0.715, "①~③ route_publisher_node (경로 발행 시 한 번)      ④~⑤ stanley_node (매 주기)      "
            "→ /target_speed", fontsize=10, color=MUTED)

    rows = list(csv.DictReader(open(MAP / "waypoints.csv")))
    pts = [(float(r["utm_x"]), float(r["utm_y"])) for r in rows]
    k = path_curvatures(pts, 5)
    lens = segment_lengths(pts)
    lim = SpeedLimits(max_speed_mps=20 / 3.6, max_lateral_accel_mps2=0.5, max_accel_mps2=2.5,
                      max_decel_mps2=0.5, min_speed_mps=10 / 3.6, curve_max_speed_mps=10 / 3.6,
                      curve_radius_m=30.0, curve_lead_m=3.0, curve_exit_lead_m=3.0)
    v = [s * 3.6 for s in speed_profile(lens, k, lim, stop_at_end=True)]
    s = [0.0]
    for length in lens:
        s.append(s[-1] + length)
    p = fig.add_axes([0.06, 0.08, 0.90, 0.54])
    for i in range(len(k) - 1):
        if abs(k[i]) > 1 / 30:
            p.axvspan(s[i], s[i + 1], color=AMBER_BG, lw=0)
    p.plot(s, v, color=BLUE, lw=2.2, label="경로 목표속도 (/planning/trajectory)")
    p.set_xlabel("경로 거리 (m)", fontsize=11)
    p.set_ylabel("km/h", fontsize=11)
    p.set_ylim(0, 23)
    p.set_xlim(0, s[-1])
    p.grid(alpha=0.3)
    p2 = p.twinx()
    radius = [min(1 / abs(c), 60) if abs(c) > 1e-6 else 60 for c in k]
    p2.plot(s, radius, color=MUTED, lw=1, alpha=0.7, label="곡률 반경 R (60 m에서 자름)")
    p2.set_ylabel("R (m)", fontsize=11, color=MUTED)
    p2.set_ylim(0, 120)
    h1, l1 = p.get_legend_handles_labels()
    h2, l2 = p2.get_legend_handles_labels()
    p.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=10)
    p.text(0.01, 0.95, "노란 구간 = R < 30 m 코너", transform=p.transAxes, fontsize=10, color=AMBER, va="top")
    fig.savefig(OUT / "stanley_algorithm_3_speed.png")
    plt.close(fig)


def combine():
    """The three figures stacked into one image, docs/stanley_algorithm.png."""

    from PIL import Image

    names = ("stanley_algorithm_1_flow.png", "stanley_algorithm_2_steering.png",
             "stanley_algorithm_3_speed.png")
    images = [Image.open(OUT / n).convert("RGB") for n in names]
    width = max(i.width for i in images)
    gap = 30
    sheet = Image.new("RGB", (width, sum(i.height for i in images) + gap * (len(images) - 1)), "white")
    y = 0
    for i, image in enumerate(images):
        sheet.paste(image, ((width - image.width) // 2, y))
        y += image.height
        if i < len(images) - 1:
            for x in range(40, width - 40):
                for dy in range(gap // 2 - 1, gap // 2 + 1):
                    sheet.putpixel((x, y + dy), (203, 210, 217))
            y += gap
    sheet.save(OUT / "stanley_algorithm.png")


if __name__ == "__main__":
    figure_flow()
    figure_steering()
    figure_speed()
    combine()
    print(OUT / "stanley_algorithm.png")
    print("\n".join(str(OUT / n) for n in ("stanley_algorithm_1_flow.png",
                                              "stanley_algorithm_2_steering.png",
                                              "stanley_algorithm_3_speed.png")))
