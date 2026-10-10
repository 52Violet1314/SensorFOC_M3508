#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/verify_fastatan2.py —— 校验固件里那个快速 atan2 近似(SMO.c 的 SMO_Atan2Fast)

固件为了省中断时间, 用"两次半角精化 + 13 阶级数"替代 atan2f。
这个脚本用**和 C 代码逐行对应的系数**重新实现一遍, 和 math.atan2 逐点比对,
防止有人改了系数却没人验证(我第一版就是系数/精化次数写错, 误差 75°)。

用法: python tools/verify_fastatan2.py
"""
import math
import random
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read_coeffs():
    """从 SMO.c 里把 Horner 系数抠出来, 防止脚本和固件脱节"""
    text = (ROOT / "FOC/src/SMO.c").read_text(encoding="utf-8", errors="replace")
    m = re.search(r"return 4\.0f \* x \* \((.*?)\);", text, re.S)
    if not m:
        raise SystemExit("SMO.c 里没找到 SMO_AtanPos 的级数表达式")
    coeffs = [float(c) for c in re.findall(r"([0-9]+\.[0-9]+)f", m.group(1))]
    nref = text.count("x = x / (1.0f + sqrtf(1.0f + x * x));")
    return coeffs, nref


def atan_pos(x, coeffs, nref):
    for _ in range(nref):
        x = x / (1.0 + math.sqrt(1.0 + x * x))
    t = x * x
    acc = coeffs[-1]                      # 最内层常数
    for c in reversed(coeffs[:-1]):
        acc = c - t * acc
    return 4.0 * x * acc


def atan2_fast(y, x, coeffs, nref):
    ax, ay = abs(x), abs(y)
    if ax < 1e-12 and ay < 1e-12:
        return 0.0
    r = atan_pos(ay / ax, coeffs, nref) if ay <= ax else math.pi / 2 - atan_pos(ax / ay, coeffs, nref)
    a = r if x >= 0 else math.pi - r
    return -a if y < 0 else a


def main():
    coeffs, nref = read_coeffs()
    print("从 SMO.c 读到的系数: %s" % coeffs)
    print("半角精化次数: %d" % nref)
    print("表达式: 4·x·(c0 - t(c1 - t(c2 - ...)))  ← Horner")
    print()

    random.seed(20261)
    worst = 0.0
    worst_pt = None
    n = 200000
    for _ in range(n):
        y = random.uniform(-15.0, 15.0)
        x = random.uniform(-15.0, 15.0)
        if abs(x) < 1e-9 and abs(y) < 1e-9:
            continue
        d = abs(atan2_fast(y, x, coeffs, nref) - math.atan2(y, x))
        d = min(d, 2 * math.pi - d)
        if d > worst:
            worst, worst_pt = d, (x, y)

    print("%d 个随机点: 最大误差 %.9f rad (%.6f 度)" % (n, worst, math.degrees(worst)))
    if worst_pt:
        print("最差点: x=%.4f y=%.4f" % worst_pt)

    # 再扫一圈固定角度, 覆盖象限边界
    worst2 = 0.0
    for k in range(0, 3600):
        a = k * math.pi / 1800.0
        y, x = math.sin(a), math.cos(a)
        d = abs(atan2_fast(y, x, coeffs, nref) - math.atan2(y, x))
        d = min(d, 2 * math.pi - d)
        worst2 = max(worst2, d)
    print("单位圆 3600 点: 最大误差 %.9f rad (%.6f 度)" % (worst2, math.degrees(worst2)))

    print()
    if worst < 1e-4 and worst2 < 1e-4:
        print("判定: 可用(误差 < 0.006°, 远小于观测器本身的噪声与整定误差)")
    else:
        print("★ 判定: 精度不足, 检查系数个数与半角精化次数")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
