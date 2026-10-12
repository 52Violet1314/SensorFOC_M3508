#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/analyze_lut.py —— 分析标定出来的 2000 点表: 误差到底来自哪儿

为什么值得看:
    表里混着两种东西, 处理方式完全不同 ——
      1) 编码器/安装的**对齐偏差**   -> 直流分量
      2) 编码器 sin/cos 自身的非线性 -> 各次谐波(与电周期同步)
      3) 转子被**齿槽**拽住的机械滞后 -> 周期 = LCM(定子槽数, 转子极数)
         例: M3508 是 12 槽 14 极 -> 84 次/机械转 -> 84/7 = 12 次/电周期
    第 3 项是物理效应, 加大标定电压(磁场更硬)就会变小; 第 2 项怎么加电压都不动。
    所以"同一个电机用两档电压各标一次"就能把两者分开。

用法: python tools/analyze_lut.py [csv路径]      (默认 tools/enc_lut.csv)
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
N = 2000                    # 表长
POLE_PAIRS = 7              # M3508: 7 对极 -> 每个机械转 7 个电周期
SLOTS, POLES = 12, 14       # M3508: 12 槽 14 极 -> 齿槽周期 = LCM = 84 次/机械转


def load(path):
    vals = {}
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        a, _, b = line.partition(",")
        try:
            vals[int(a)] = float(b)
        except ValueError:
            pass
    if len(vals) < N:
        print("★只有 %d 行, 需要 %d 行" % (len(vals), N))
        return None
    return [vals[i] for i in range(N)]


def ascii_plot(v, width=100, height=9):
    """把表画成 ASCII(等宽字体下能直接看形状)"""
    lo, hi = min(v), max(v)
    span = (hi - lo) or 1.0
    rows = [[" "] * width for _ in range(height)]
    for x in range(width):
        i = int(x * N / width)
        j = int((v[i] - lo) / span * (height - 1))
        rows[height - 1 - j][x] = "#"
    print("  %+.4f ┐" % hi)
    for r in rows:
        print("         │" + "".join(r))
    print("  %+.4f ┘  ← 横轴 = 电角度 0→2π" % lo)


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else (ROOT / "tools" / "enc_lut.csv")
    if not path.exists():
        print("找不到 %s —— 先跑 tools/lut_capture.py 存一份" % path)
        return 1
    v = load(path)
    if v is None:
        return 1

    import math
    mean = sum(v) / N
    pp = max(v) - min(v)
    rms = math.sqrt(sum((x - mean) ** 2 for x in v) / N)
    print("表: %s" % path)
    print("  直流(对齐偏差) = %+.3f deg" % math.degrees(mean))
    print("  峰峰(总非线性) =  %.3f deg" % math.degrees(pp))
    print("  有效值         =  %.3f deg" % math.degrees(rms))

    # 谐波分析: 只关心"每个电周期几次", 换算到机械转要 ×POLE_PAIRS
    try:
        import numpy as np
    except ImportError:
        print("  (没有 numpy, 跳过谐波分解)")
        return 0

    a = np.asarray(v, dtype=float) - mean
    spec = np.fft.rfft(a)
    amp = 2.0 * np.abs(spec) / N
    order = np.argsort(amp)[::-1]

    print("\n主要谐波(按幅值):")
    print("  每电周期次数 | 每机械转次数 | 幅值(deg) | 占比")
    shown = 0
    for k in order:
        if k == 0 or amp[k] < math.radians(0.15):
            break
        print("      %4d     |    %5d      |   %6.3f  | %4.1f%%"
              % (k, k * POLE_PAIRS, math.degrees(amp[k]),
                 100.0 * amp[k] / (pp / 2.0)))
        shown += 1
        if shown >= 10:
            break

    # 齿槽周期(84/机械转 = 12/电周期)的能量占比: 判定"是不是齿槽在捣乱"
    cog = SLOTS * POLES // math.gcd(SLOTS, POLES)      # 84 次/机械转
    k_cog = cog // POLE_PAIRS                           # = 12 次/电周期
    e_cog = 0.0
    e_all = 0.0
    for k in range(1, N // 2):
        e_all += amp[k] ** 2
        if k % k_cog == 0:
            e_cog += amp[k] ** 2
    if e_all > 0:
        print("\n齿槽族(%d 次/机械转 = %d 次/电周期 及其整数倍)能量占比: %.1f%%"
              % (cog, k_cog, 100.0 * e_cog / e_all))
        print("  → 占比高 = 周期性部分主要是**转子滞后**(物理效应, 加大标定电压能压小)")
        print("  → 占比低 = 主要是**编码器自身非线性**(只能靠表修, 加电压没用)")

    print()
    ascii_plot(v)
    return 0


if __name__ == "__main__":
    sys.exit(main())
