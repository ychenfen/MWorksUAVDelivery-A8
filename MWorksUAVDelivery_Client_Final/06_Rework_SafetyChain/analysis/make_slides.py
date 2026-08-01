"""生成安全链重跑结果的幻灯页（1920x1080），供拼接进演示视频。"""
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import FancyBboxPatch

plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "Heiti TC", "PingFang HK"]
plt.rcParams["axes.unicode_minus"] = False

FIG = Path("/Users/yuchenxu/Desktop/MWorksUAVDelivery/MWorksUAVDelivery_Client_Final/06_Rework_SafetyChain/figures")
OUT = Path("/private/tmp/claude-501/-Users-yuchenxu-Desktop-MWorksUAVDelivery/7503e47e-6052-465d-b6e9-b8133ccfdfda/scratchpad/slides")
OUT.mkdir(parents=True, exist_ok=True)

BG = "#0d1b2a"; FG = "#e8eef5"; ACC = "#4da6ff"; OK = "#4ade80"; BAD = "#f87171"; DIM = "#8fa3b8"
W, H = 19.20, 10.80


def canvas():
    fig = plt.figure(figsize=(W, H), dpi=100)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off"); ax.set_facecolor(BG)
    return fig, ax


def title(ax, t, sub=None):
    ax.text(0.06, 0.90, t, color=FG, fontsize=40, fontweight="bold", va="top")
    ax.plot([0.06, 0.30], [0.865, 0.865], color=ACC, lw=4, transform=ax.transAxes)
    if sub:
        ax.text(0.06, 0.835, sub, color=DIM, fontsize=20, va="top")


def save(fig, name):
    fig.savefig(OUT / name, facecolor=BG, dpi=100)
    plt.close(fig)
    print("  ", name)


# ---------- S1 章节标题 ----------
fig, ax = canvas()
ax.text(0.5, 0.60, "安全链验证", color=FG, fontsize=76, fontweight="bold", ha="center")
ax.text(0.5, 0.47, "MWORKS Sysplorer 重跑结果", color=ACC, fontsize=36, ha="center")
ax.plot([0.34, 0.66], [0.42, 0.42], color=ACC, lw=3)
ax.text(0.5, 0.34, "全部判据在实验执行之前确定，未因结果不利而调整", color=DIM, fontsize=22, ha="center")
save(fig, "s01.png")

# ---------- S2 机理结论 ----------
fig, ax = canvas()
title(ax, "姿态超限的成因定位", "六组仿真 · 累计 220 s 飞行")
rows = [("全部 15° 超限采样点", "落在参考轨迹不连续之后 0.06 ~ 0.08 s 内", OK),
        ("其余时段", "无一超限", OK),
        ("但参考不连续", "只是必要条件，不是充分条件", ACC)]
y = 0.66
for a, b, c in rows:
    ax.text(0.08, y, a, color=c, fontsize=30, fontweight="bold", va="center")
    ax.text(0.42, y, b, color=FG, fontsize=28, va="center")
    y -= 0.11
ax.text(0.08, 0.26, "同一场景的干净对照：t=20 s 与 t=40 s 单轴折角、t=30 s 双轴同时折角",
        color=DIM, fontsize=24, va="center")
ax.text(0.08, 0.18, "三者倾角指令完全等价（均饱和 9.500°、速率约 560 °/s、上升 40 ms）",
        color=DIM, fontsize=24, va="center")
ax.text(0.08, 0.10, "姿态峰却是 11.92° / 17.39° / 11.92°  ——  差别在指令之后",
        color=FG, fontsize=26, fontweight="bold", va="center")
save(fig, "s02.png")


def figslide(name, img, t, sub, note=None):
    fig, ax = canvas()
    title(ax, t, sub)
    im = mpimg.imread(str(FIG / img))
    h, w = im.shape[0], im.shape[1]
    box_w, box_h = 0.84, 0.60
    ar = w / h
    if ar > box_w * W / (box_h * H):
        dw = box_w; dh = box_w * W / ar / H
    else:
        dh = box_h; dw = box_h * H * ar / W
    x0 = 0.5 - dw / 2; y0 = 0.14 + (box_h - dh) / 2
    axi = fig.add_axes([x0, y0, dw, dh]); axi.imshow(im); axi.axis("off")
    if note:
        ax.text(0.5, 0.075, note, color=DIM, fontsize=21, ha="center")
    save(fig, name)


figslide("s03.png", "F1_参考不连续与超限对应.png", "参考不连续与超限时刻一一对应",
         "增强模型，三场景", "6/6 超限点均落在参考跳变后 1 s 内，其余 220 s 无一超限")
figslide("s04.png", "F2_单轴双轴对照.png", "单轴 vs 双轴不连续对照",
         "爬升增强模型 t=20 / 30 / 40 s", "指令侧完全等价，差别在内环饱和与混控分配（电机摆幅比 1.05 vs 16.64）")
figslide("s05.png", "F2_超限机理时序.png", "超限事件的指令与姿态时序",
         "参考跳变后 0.35 s", "指令 3~4 个采样点内打满限幅，姿态随后越过 15° 边界")
figslide("s06.png", "F3_指令变化率分离度.png", "限速阈值由数据分布确定",
         "正常飞行 vs 参考跳变段", "两者相差 20~40 倍；取 60 °/s，实测仅 0.06% 采样点被限住")
figslide("s07.png", "F4_误差分解.png", "参考不可行段对 RMSE 的贡献",
         "分段统计", "螺旋场景约一半 RMSE 来自 t=10 s 的 3 m 参考阶跃，非控制器能力所致")

# ---------- S8 重跑结果表 ----------
fig, ax = canvas()
title(ax, "安全链重跑结果", "MWORKS Sysplorer")
hdr = ["模型", "路径RMSE/m", "max|roll|/°", "max|pitch|/°", "15°超限累计时长/s"]
data = [["ClimbSafeERG", "0.236546", "9.839", "9.847", "0"],
        ["SpiralSafeERG", "0.467209", "9.300", "12.465", "0"],
        ["EightSafeERG", "0.147845", "12.503", "7.688", "0"],
        ["ClimbSafeFull", "0.236552", "9.840", "9.848", "0"],
        ["SpiralSafeFull", "0.467221", "9.299", "12.429", "0"],
        ["EightSafeFull", "0.147848", "12.470", "7.688", "0"]]
xs = [0.08, 0.34, 0.50, 0.64, 0.80]
y = 0.70
for i, hh in enumerate(hdr):
    ax.text(xs[i], y, hh, color=ACC, fontsize=24, fontweight="bold", va="center")
ax.plot([0.06, 0.94], [y - 0.035, y - 0.035], color=ACC, lw=2)
y -= 0.085
for r in data:
    for i, v in enumerate(r):
        c = OK if (i == 4) else FG
        fw = "bold" if i == 4 else "normal"
        ax.text(xs[i], y, v, color=c, fontsize=24, va="center", fontweight=fw)
    y -= 0.075
ax.text(0.08, 0.16, "六个模型的 15° 超限累计时长全部归零", color=OK, fontsize=28, fontweight="bold")
ax.text(0.08, 0.09, "对照：不接安全链 0.030 s   仅启用 CBF 0.060 s", color=DIM, fontsize=22)
save(fig, "s08.png")

# ---------- S9 判据达成 ----------
fig, ax = canvas()
title(ax, "预注册判据达成情况", "判据在实验前写定，不因结果调整")
items = [("判据一  等价性回归", "通过", "0.228016 vs 0.228024，差 8×10⁻⁶（0.0035%）", OK),
         ("判据二  ERG 消除超限", "通过 6/6", "六个模型超限累计时长全部为 0", OK),
         ("判据三  ERG 不损精度", "未达标 2/3", "爬升 +3.74% 超出 3% 门槛 0.74 个百分点", BAD),
         ("判据四  安全链消融", "完成", "结论与预期相反，见下页", ACC),
         ("判据五  编队精度", "控制律已验证", "Python 独立复现偏差 0.09% / 0.00%", OK),
         ("判据六  闭环鲁棒性", "尚未完成", "本报告不对鲁棒性作任何定量声明", DIM)]
y = 0.72
for a, b, c, col in items:
    ax.text(0.07, y, a, color=FG, fontsize=26, va="center")
    ax.text(0.34, y, b, color=col, fontsize=26, fontweight="bold", va="center")
    ax.text(0.50, y, c, color=DIM, fontsize=22, va="center")
    y -= 0.095
ax.text(0.07, 0.11, "判据三写定时以后来证实过期的基准计算为 +1.57%、看似达标；"
                    "基准修正不构成回头放宽判据的理由，按未达标记录。",
        color=BAD, fontsize=21, va="center")
save(fig, "s09.png")

# ---------- S10 消融发现 ----------
fig, ax = canvas()
title(ax, "消融实验：一个与预期相反的发现", "三模型倾角限幅同为 12°，可直接比较")
cols = [("仅启用 ERG", "0.236552", "0 s", OK), ("仅启用 CBF", "0.224530", "0.060 s", BAD),
        ("两者都开", "0.236552", "0 s", OK)]
x = 0.12
for nm, rm, vt, col in cols:
    ax.add_patch(FancyBboxPatch((x, 0.42), 0.22, 0.28, boxstyle="round,pad=0.012",
                                fc="#16273d", ec=col, lw=3, transform=ax.transAxes))
    ax.text(x + 0.11, 0.655, nm, color=FG, fontsize=26, ha="center", fontweight="bold")
    ax.text(x + 0.11, 0.575, f"RMSE {rm}", color=DIM, fontsize=22, ha="center")
    ax.text(x + 0.11, 0.485, f"超限 {vt}", color=col, fontsize=30, ha="center", fontweight="bold")
    x += 0.27
ax.text(0.07, 0.32, "消除超限的是限速（ERG），不是障碍函数（CBF）", color=OK, fontsize=32, fontweight="bold")
ax.text(0.07, 0.24, "ERG 开启后 CBF 累计修正量为 0 —— 从未触发", color=FG, fontsize=28)
ax.text(0.07, 0.15, "与机理分析一致：超限由指令阶跃激发，作用于指令幅值的障碍函数无法阻止响应超调，",
        color=DIM, fontsize=22)
ax.text(0.07, 0.09, "限制指令变化率才是对症手段。", color=DIM, fontsize=22)
save(fig, "s10.png")

figslide("s11.png", "F5_编队控制律验证.png", "多机编队控制律验证",
         "一致性编队 + 势场避撞", "Modelica 实测 0.552288 m / 4.097131 m，与 Python 独立复现偏差 0.09% / 0.00%")

# ---------- S12 自查与更正 ----------
fig, ax = canvas()
title(ax, "初赛版本的自查与更正", "以下问题由我们自己复算发现")
items = [("三类场景 RMSE 经安全链降低 53%/52%/55%", "由基线乘硬编码系数 0.47/0.48/0.45 得到"),
         ("阶跃超调降低 65.00%", "由基线超调乘 0.35 得到"),
         ("180 组蒙特卡洛 P95 均低于 15%", "算式中标称 RMSE 被约去，与控制器无关"),
         ("集成四大安全模块形成协同闭环", "四个模块从未被实例化，未进入控制回路"),
         ("三机编队最大队形误差 0.861 m", "实现中不含控制律，是延迟造成的几何偏差")]
y = 0.70
for a, b in items:
    ax.text(0.07, y, "✕", color=BAD, fontsize=26, va="center")
    ax.text(0.11, y, a, color=DIM, fontsize=24, va="center")
    ax.text(0.55, y, "→", color=ACC, fontsize=24, va="center")
    ax.text(0.60, y, b, color=FG, fontsize=23, va="center")
    y -= 0.095
ax.text(0.07, 0.16, "全部已在本次提交中更正，相关代码与数据已移出交付路径",
        color=OK, fontsize=26, fontweight="bold")
ax.text(0.07, 0.09, "报告中每一条量化结论均可由交付的仿真输出直接复算",
        color=DIM, fontsize=23)
save(fig, "s12.png")

print(f"\n共生成 {len(list(OUT.glob('*.png')))} 页 -> {OUT}")
