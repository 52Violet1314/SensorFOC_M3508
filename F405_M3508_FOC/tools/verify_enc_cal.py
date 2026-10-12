#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/verify_enc_cal.py —— 离线验证"2000 点电角度标定"的数学

为什么必须先在电脑上跑:
    标定要上电、要转电机、要占台架时间; 而数学错了(符号反了 / 分格索引对不上 /
    把"步进误差"当成"位置误差")在板子上只表现为"标完还是抖", 极难定位。
    上一版固件就是栽在这里: 它把每个格子的**步进误差**直接当成角度补偿量存进表,
    正确的量应该是这个误差的**积分**。这里用合成数据把整条链路跑一遍。

对照的真值模型:
    θ_raw = θ_true + err(θ_true)          (编码器自身的周期误差, 谐波叠加)
    标定时的参考 φ 就是 θ_true            (转子被电压矢量拖住)
    期望标出来的表 ≈ -err, 且补偿后误差应几乎归零。

用法: python tools/verify_enc_cal.py     (退出码非 0 = 标定数学有问题)
"""
import math

TWO_PI = 6.283185307179586
N = 2000            # 表长
FS = 10000.0        # 控制/采样频率 Hz
F_CAL = 2.0         # 标定电频率 Hz
CYCLES = 40         # 采集电周期数


def wrap(a):
    a = math.fmod(a, TWO_PI)
    if a < 0.0:
        a += TWO_PI
    return a


def wrap_pm_pi(a):
    while a >= math.pi:
        a -= TWO_PI
    while a < -math.pi:
        a += TWO_PI
    return a


def lut_index(theta):
    """照抄 Encoder_LutIndex()"""
    a = wrap(theta)
    idx = int((a / TWO_PI) * N)
    return N - 1 if idx >= N else idx


def lut_apply(theta_raw, theta_prev, lut):
    """照抄 Encoder_LutApply(): 表和标定同口径 —— 都按**粗角度**索引"""
    a = theta_raw + lut[lut_index(theta_raw)]
    if (a - theta_prev) > math.pi:
        a -= TWO_PI
    elif (a - theta_prev) < -math.pi:
        a += TWO_PI
    return a


def true_err(t):
    """编码器的真实周期误差(rad): 直流对齐误差 + 2/3/5 次谐波(sin/cos 不正交/幅值不等就是这个形状)"""
    return (0.030 + 0.050 * math.sin(2.0 * t) + 0.020 * math.sin(3.0 * t + 0.7)
            + 0.012 * math.sin(5.0 * t + 1.9))


def run_cal(lut, cycles=CYCLES, freq=F_CAL):
    """照抄固件里的标定累积: 每拍 acc[bin(raw)] += φ - raw"""
    acc = [0.0] * N
    cnt = [0] * N
    n = 0
    phi = 0.0
    steps = int(cycles * FS / freq)
    dphi = TWO_PI * freq / FS

    for _ in range(steps):
        t_true = wrap(phi)
        raw = wrap_pm_pi(t_true + true_err(t_true))

        e = wrap_pm_pi(phi - raw)
        # ★分格用**粗角度**(与固件 Encoder_CalSample 一致): 同一个粗角度必然对应同一个
        #   真实角度, 所以每格均值就是该格要补的量, 覆盖也必然是满的。
        idx = lut_index(raw)
        acc[idx] += e
        cnt[idx] += 1
        n += 1
        phi = wrap(phi + dphi)

    return acc, cnt, n


def finish(acc, cnt, n):
    """照抄 Encoder_CalFinish(): 平均 -> 补空洞 -> 5 点滑动平均"""
    lut = [0.0] * N
    have = [c > 0 for c in cnt]
    used = sum(1 for h in have if h)
    if n < N or used < (N * 9) // 10:
        return None, used

    for i in range(N):
        lut[i] = acc[i] / cnt[i] if cnt[i] > 0 else 0.0

    # 空洞补齐(循环表)
    for i in range(N):
        if have[i]:
            continue
        a, b, da, db = i, i, 0, 0
        while True:
            a = (a - 1) % N
            da += 1
            if have[a] or da >= N:
                break
        while True:
            b = (b + 1) % N
            db += 1
            if have[b] or db >= N:
                break
        if da >= N or db >= N:
            lut[i] = 0.0
            continue
        d = wrap_pm_pi(lut[b] - lut[a])
        lut[i] = lut[a] + d * da / (da + db)

    # 5 点环形滑动平均
    tmp = [0.0] * N
    for i in range(N):
        s = 0.0
        for k in (-2, -1, 0, 1, 2):
            s += lut[(i + k) % N]
        tmp[i] = s * 0.2
    return tmp, used


def residual(lut, freq=F_CAL, cycles=4):
    """补偿后还剩多大误差: 用另一段独立的运行轨迹评估(避免自我验证)"""
    phi = 0.0
    steps = int(cycles * FS / freq)
    dphi = TWO_PI * freq / FS
    worst = 0.0
    rms = 0.0
    prev = 0.0
    for k in range(steps):
        t_true = wrap(phi)
        raw = wrap_pm_pi(t_true + true_err(t_true))
        corr = lut_apply(raw, prev, lut)
        prev = corr
        e = wrap_pm_pi(corr - t_true)
        worst = max(worst, abs(e))
        rms += e * e
        phi = wrap(phi + dphi)
    return worst, math.sqrt(rms / steps)


def main():
    zero = [0.0] * N
    w0, r0 = residual(zero)
    print("标定前:             max|err| = %.4f rad (%.3f deg)   rms = %.4f rad"
          % (w0, math.degrees(w0), r0))

    # ---------- 第一遍 ----------
    acc, cnt, n = run_cal(zero)
    used = sum(1 for c in cnt if c > 0)
    lut1, _ = finish(acc, cnt, n)
    if lut1 is None:
        print("★第一遍标定就失败了(覆盖/样本不足), 检查参数")
        return 1
    w1, r1 = residual(lut1)
    print("第一遍: max|err| = %.4f rad (%.3f deg)   rms = %.4f rad   [样本 %d, 覆盖 %d/%d]"
          % (w1, math.degrees(w1), r1, n, used, N))

    # ---------- 第二遍(同口径, 只是再平均一次) ----------
    acc, cnt, n = run_cal(lut1)
    used = sum(1 for c in cnt if c > 0)
    lut2, _ = finish(acc, cnt, n)
    if lut2 is None:
        print("★第二遍失败")
        return 1
    w2, r2 = residual(lut2)
    print("第二遍: max|err| = %.4f rad (%.3f deg)   rms = %.4f rad   [覆盖 %d/%d]"
          % (w2, math.degrees(w2), r2, used, N))

    # ---------- 对照: 上一版固件的错误做法 ----------
    #   它把"每格平均步进"直接当补偿量(该量其实是位置误差的导数), 差一个积分。
    dphi = TWO_PI * F_CAL / FS
    step_err = [(acc[i] / cnt[i]) - dphi if cnt[i] > 0 else 0.0 for i in range(N)]
    scale = TWO_PI * CYCLES / sum(acc[i] for i in range(N) if cnt[i] > 0)
    bad = [step_err[i] * scale for i in range(N)]
    wb, rb = residual(bad)
    print("上一版做法(步进误差当补偿): max|err| = %.4f rad (%.3f deg)   rms = %.4f rad"
          % (wb, math.degrees(wb), rb))

    ok = (w1 < w0 * 0.35) and (w2 <= w1 * 1.02)
    print("\n结论: " + ("标定数学正确(把误差压掉 %.1f 倍)"
                        % (w0 / max(w1, 1e-9))
                        if ok else "★标定数学有问题, 别上电"))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
