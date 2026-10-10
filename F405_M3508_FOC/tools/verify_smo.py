#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/verify_smo.py —— 滑模观测器算法验证(主机侧, 纯 Python)

用"真值电机模型"生成 αβ 电流, 喂给与 FOC/src/SMO.c 完全等价的观测器递推,
检查角度跟踪误差随转速的变化, 并复现"符号写反(滑模项变成负反馈)"的后果。

★ 参数不是抄来的: 直接从 FOC/inc/FOC_Param.h 和 FOC/inc/SMO.h 里解析,
  所以改了固件参数这里会跟着变。

用法:
    python tools/verify_smo.py
    python tools/verify_smo.py --buggy      # 用旧代码的符号, 看它怎么坏
"""
import argparse
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read_defines(path, names):
    text = path.read_text(encoding="utf-8", errors="replace")
    out = {}
    for name in names:
        m = re.search(r"^#define\s+%s\s+([0-9eE\.\+\-fF]+)" % re.escape(name),
                      text, re.M)
        if m:
            out[name] = float(m.group(1).rstrip("fF"))
    return out


def load_params():
    p = read_defines(ROOT / "FOC/inc/FOC_Param.h",
                     ["FOC_RS", "FOC_LS", "FOC_POLE_PAIRS", "FOC_FLUX",
                      "OBS_EMF_MIN_V", "OL_VMAX_DEF"])
    a = read_defines(ROOT / "BSP/inc/App_Config.h", ["APP_CTRL_HZ"])
    s = read_defines(ROOT / "FOC/inc/SMO.h",
                     ["SMO_SLIDE_GAIN", "SMO_BOUNDARY", "SMO_EMF_LPF_ALPHA",
                      "SMO_SPD_LPF_ALPHA", "SMO_LAG_WDEF"])
    cfg = {
        "Rs": p["FOC_RS"], "Ls": p["FOC_LS"], "pp": int(p["FOC_POLE_PAIRS"]),
        "flux": p["FOC_FLUX"], "fs": a["APP_CTRL_HZ"],
        "emf_min": p["OBS_EMF_MIN_V"], "vmax": p.get("OL_VMAX_DEF", 6.5),
        "k": s["SMO_SLIDE_GAIN"], "delta": s["SMO_BOUNDARY"],
        "alpha": s["SMO_EMF_LPF_ALPHA"], "spd_alpha": s["SMO_SPD_LPF_ALPHA"],
        "lag_w": s["SMO_LAG_WDEF"],
    }
    cfg["Ts"] = 1.0 / cfg["fs"]
    cfg["F"] = math.exp(-cfg["Ts"] * cfg["Rs"] / cfg["Ls"])
    cfg["G"] = (1.0 - cfg["F"]) / cfg["Rs"]
    return cfg


def sat(x):
    return 1.0 if x > 1.0 else (-1.0 if x < -1.0 else x)


def noise(rng, amp):
    """±amp/2 的均匀噪声(用整数 LCG, 保证可复现)"""
    rng[0] = (rng[0] * 1103515245 + 12345) & 0xFFFFFFFF
    return ((rng[0] >> 16 & 0xFF) / 255.0 - 0.5) * amp


class Smo(object):
    """与 FOC/src/SMO.c 的 SMO_Run() 等价的递推"""

    def __init__(self, c, buggy=False):
        self.c = c
        self.buggy = buggy
        self.i_hat = [0.0, 0.0]
        self.e_lpf = [0.0, 0.0]
        self.theta_prev = 0.0
        self.omega = 0.0
        self.valid = False
        self.emf = 0.0

    def step(self, ia, ib, va, vb):
        c = self.c
        s0 = ia - self.i_hat[0]
        s1 = ib - self.i_hat[1]
        w0 = c["k"] * sat(s0 / c["delta"])
        w1 = c["k"] * sat(s1 / c["delta"])

        if not self.buggy:
            self.i_hat[0] = c["F"] * self.i_hat[0] + c["G"] * (va + w0)
            self.i_hat[1] = c["F"] * self.i_hat[1] + c["G"] * (vb + w1)
            e0, e1 = -w0, -w1
        else:
            # 旧代码: e = +k·sat(i−î) 却从电压里减掉 ⇒ 同相位负反馈
            self.i_hat[0] = c["F"] * self.i_hat[0] + c["G"] * (va - w0)
            self.i_hat[1] = c["F"] * self.i_hat[1] + c["G"] * (vb - w1)
            e0, e1 = w0, w1

        self.e_lpf[0] += c["alpha"] * (e0 - self.e_lpf[0])
        self.e_lpf[1] += c["alpha"] * (e1 - self.e_lpf[1])

        self.emf = math.hypot(self.e_lpf[0], self.e_lpf[1])
        # 与固件一致: θ = atan2(−e_alpha, e_beta)
        th = math.atan2(-self.e_lpf[0], self.e_lpf[1])

        if not self.valid:
            self.theta_prev = th
            self.valid = True
            self.omega = 0.0
        else:
            d = th - self.theta_prev
            if d > math.pi:
                d -= 2 * math.pi
            if d < -math.pi:
                d += 2 * math.pi
            self.theta_prev = th
            if self.emf >= self.c["emf_min"]:
                self.omega += c["spd_alpha"] * (d / c["Ts"] - self.omega)
        return th


def wrap(x):
    while x >= math.pi:
        x -= 2 * math.pi
    while x < -math.pi:
        x += 2 * math.pi
    return x


def run_point(c, rpm, buggy=False, seconds=0.60, settle=0.45, iq=5.0):
    """同步坐标系电流激励下的对照实验(与固件同一套递推)

    真值: 转子电角度 th, 电流矢量固定在"真实 dq 轴"上 (id=0, iq 给定),
          反电动势 e = we*flux*(-sin th, cos th)。
    观测器: 吃实测电流(加噪声) + 实际施加的电压(stator 电压, 含电感项),
            输出角度/速度估计。
    输出: 补偿前/后的角度残差, 以及速度估计 —— 补偿量用与固件相同的
          Lag = atan(|we|/(wf+wr)), wf = alpha*fs, wr = |we|+lag_w。
    """
    pp, flux, Rs, Ls, Ts = c["pp"], c["flux"], c["Rs"], c["Ls"], c["Ts"]
    we = rpm * pp * 2 * math.pi / 60.0
    smo = Smo(c, buggy)
    th = 0.0
    e_raw, e_cmp, omegas, emfs = [], [], [], []
    rng = [12345]
    wf = c["alpha"] * c["fs"]

    for i in range(int(seconds / Ts)):
        s, co = math.sin(th), math.cos(th)

        # ★αβ 电流按固件自己的 Park 口径反推: id=0, iq=+iq
        #   Park:  id = iα·cosθ + iβ·sinθ ,  iq = −iα·sinθ + iβ·cosθ
        #   逆变换: iα = −iq·sinθ , iβ = +iq·cosθ
        ia = -iq * s + noise(rng, 0.02)
        ib = iq * co + noise(rng, 0.02)

        # 对应的反电动势(与电流反相, 幅值 ωe·ψf) 与 stator 电压 v = e + R·i + L·di/dt
        va = (-we * flux * s) + Rs * (-iq * s)
        vb = (we * flux * co) + Rs * (iq * co)

        th_hat = smo.step(ia, ib, va, vb)

        if i * Ts > settle:
            # 误差统计用圆均值/圆标准差(见下面的 stat), 这里保持单点 wrap 即可
            err = wrap(th_hat - th)
            lag = math.atan(abs(smo.omega) / (wf + c["lag_w"]))
            if smo.omega < 0:
                lag = -lag
            e_raw.append(err)
            e_cmp.append(wrap(err + lag))
            omegas.append(smo.omega)
            emfs.append(smo.emf)

        th = (th + we * Ts) % (2 * math.pi)

    def stat(v):
        """圆均值/圆标准差: 偏置可能落在 ±180° 附近, 普通均值会被 wrap 骗"""
        s = sum(math.sin(x) for x in v) / len(v)
        c = sum(math.cos(x) for x in v) / len(v)
        r = math.hypot(s, c)
        return (math.degrees(math.atan2(s, c)),
                math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(r, 1e-12))))))

    m_raw, sd_raw = stat(e_raw)
    m_cmp, sd_cmp = stat(e_cmp)
    return {
        "rpm": rpm, "we": we,
        "err_raw_mean_deg": m_raw, "err_raw_std_deg": sd_raw,
        "err_cmp_mean_deg": m_cmp, "err_cmp_std_deg": sd_cmp,
        "rpm_est": sum(omegas) / len(omegas) / pp * 9.54929658551,
        "emf": sum(emfs) / len(emfs),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--buggy", action="store_true", help="用旧代码的符号")
    ap.add_argument("--rpms", default="300,600,1200,2000,3000,4000")
    a = ap.parse_args()

    c = load_params()
    print("参数(直接读自固件头文件):")
    print("  Rs=%.4fΩ Ls=%.1fuH pp=%d flux=%.4eWb fs=%dHz"
          % (c["Rs"], c["Ls"] * 1e6, c["pp"], c["flux"], c["fs"]))
    print("  F=%.4f G=%.4f A/V  k=%.1fV delta=%.1fA alpha=%.3f"
          % (c["F"], c["G"], c["k"], c["delta"], c["alpha"]))
    print("  |E|min=%.2fV  (12V 母线相电压幅值上限 %.2fV)"
          % (c["emf_min"], c["vmax"]))

    # ★离散稳定性自检: 换了控制频率/增益/边界层没跟着改, 观测器会直接发散。
    #   决定稳定的是无量纲量 G·k/δ, 线性区闭环极点是 F − G·k/δ, 必须落在单位圆内。
    pole = c["F"] - c["G"] * c["k"] / c["delta"]
    dmin = c["G"] * c["k"] / (1.0 + c["F"])
    print("  离散稳定性: G·k/δ = %.3f -> 闭环极点 %+.3f   δ下限 %.1fA(现用 %.1fA)  %s"
          % (c["G"] * c["k"] / c["delta"], pole, dmin, c["delta"],
             "OK" if abs(pole) < 1.0 and c["delta"] > dmin else "★发散!"))
    if abs(pole) >= 1.0 or c["delta"] <= dmin:
        print()
        print("★ 稳定性不成立, 后面的数字没有意义。按这个规则改:")
        print("   δ 必须与控制周期 Ts 成正比(因为 G=(1−F)/R 随 Ts 增大):")
        print("     40kHz(Ts=25us): δ=8A     20kHz(Ts=50us): δ=16A")
        raise SystemExit(1)
    print()
    if a.buggy:
        print("### 旧代码符号(滑模项当负反馈) ###")
    else:
        print("### 现在的实现(eps=i-i_hat, e_hat=-w, theta=atan2(-ea,eb)+Lag) ###")
    print("%7s | %-21s | %-21s | %10s %8s" %
          ("rpm", "补偿前 偏置/波动[deg]", "补偿后 偏置/波动[deg]",
           "rpm_est", "|E|[V]"))
    print("-" * 88)

    ok = True
    for rpm in [float(x) for x in a.rpms.split(",")]:
        r = run_point(c, rpm, a.buggy)
        print("%7.0f | %+8.2f +- %8.3f | %+8.2f +- %8.3f | %10.1f %8.3f" %
              (r["rpm"], r["err_raw_mean_deg"], r["err_raw_std_deg"],
               r["err_cmp_mean_deg"], r["err_cmp_std_deg"],
               r["rpm_est"], r["emf"]))
        # 判定只看"补偿后的波动"和速度估计(偏置是常数, 可以标定掉)
        if r["emf"] >= 3.0 * c["emf_min"]:
            if r["err_cmp_std_deg"] > 15.0:
                ok = False
            if abs(r["rpm_est"] - r["rpm"]) > 0.05 * r["rpm"]:
                ok = False
    print()
    if a.buggy:
        print("结论: 滑模项写成负反馈时, 电流观测器根本不收敛, 角度/速度全是错的。")
    else:
        print("结论: %s" % ("有效档位都能稳定跟踪, 速度估计误差 < 5%。" if ok else
                            "★有档位跟踪失败, 检查 k/delta/alpha 整定。"))
    print("说明: 偏置是常数(角度对齐), 现场用 off 命令标掉;")
    print("      剩余随转速漂的部分由 lag 命令(等效带宽)压; |E| 小的低速档本就不该用观测角。")


if __name__ == "__main__":
    main()
