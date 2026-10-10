#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/vf_sim.py —— 开环(V/F)启动能力估算: 回答"为什么堵转 / 为什么不转"

把 FOC/src/VF.c 的启动逻辑(对齐 / 斜坡 / 最低电频率 / 低频强制电压 / 积分角 / 失步保护)
原样搬过来, 配一个同步电机模型:

    L·diα/dt = vα − R·iα − eα          (αβ 电流真实动态, 不是准静态近似)
    L·diβ/dt = vβ − R·iβ − eβ
    eα = −ωe·ψf·sinθ,  eβ = +ωe·ψf·cosθ
    Te = 1.5·p·ψf·iq                   (iq 由真实 dq 轴取)
    J·dωm/dt = Te − Tload − Tcog·sin(p·θr) − B·ωm
    静止时 |Te−Tload| < T_static -> 被静摩擦锁住

用法:
    python tools/vf_sim.py                          # 固件默认参数
    python tools/vf_sim.py --vlo 4.0 --fres 8       # 指定参数
    python tools/vf_sim.py --sweep                  # 扫 vlo/fres
    python tools/vf_sim.py --inertia 1e-5           # 换惯量假设
"""
import argparse
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# ---- 电机/驱动参数(与固件一致) ----
PP = 7.0
FLUX = 1.0e-3
RS = 0.194
LS = 97e-6
FS = 20000.0
TS = 1.0 / FS

# ---- 机械参数(粗估, 只用于判断"够不够起动") ----
J_DEF = 1.0e-5     # 电机侧等效惯量 kg·m²(转子 + 减速箱+输出轴折算; M3508 无负载量级)
T_COG = 0.008      # 电机侧齿槽转矩幅值 N.m
T_STATIC = 0.015   # 电机侧静摩擦 N.m(减速箱油封+油脂)
B_VISC = 5.0e-6
T_LOAD = 0.0


def read_defines(path, names):
    text = path.read_text(encoding="utf-8", errors="replace")
    out = {}
    for name in names:
        m = re.search(r"^#define\s+%s\s+([0-9eE\.\+\-fF]+)" % re.escape(name), text, re.M)
        if m:
            out[name] = float(m.group(1).rstrip("fF"))
    return out


def load_fw_defaults():
    p = read_defines(ROOT / "FOC/inc/FOC_Param.h",
                     ["OL_RPM_DEF", "OL_RAMP_DEF", "OL_VBOOST_DEF", "OL_VSLOPE_DEF",
                      "OL_VMAX_DEF", "OL_FSTART_DEF", "OL_VALIGN_DEF",
                      "OL_VBOOST_LO_DEF", "OL_FRES_HZ", "OL_EMF_TIMEOUT_MS"])
    return {
        "rpm": p.get("OL_RPM_DEF", 1500.0),
        "ramp": p.get("OL_RAMP_DEF", 600.0),
        "vboost": p.get("OL_VBOOST_DEF", 1.0),
        "slope": p.get("OL_VSLOPE_DEF", 0.05),
        "vmax": p.get("OL_VMAX_DEF", 6.5),
        "fstart": p.get("OL_FSTART_DEF", 2.0),
        "valign": p.get("OL_VALIGN_DEF", 1.5),
        "vlo": p.get("OL_VBOOST_LO_DEF", 2.8),
        "fres": p.get("OL_FRES_HZ", 6.0),
        "timeout_ms": p.get("OL_EMF_TIMEOUT_MS", 2000.0),
    }


def run(vlo, fres, cfg, rpm_target=300.0, t_end=4.0, J=J_DEF, quiet=False):
    """返回 dict: ok/rpm_end/i_max/i_avg/t_spin/t_stall/log"""
    ramp, vboost, slope, vmax = cfg["ramp"], cfg["vboost"], cfg["slope"], cfg["vmax"]
    fstart, valign = cfg["fstart"], cfg["valign"]
    timeout_s = cfg["timeout_ms"] / 1000.0

    theta = 0.0
    rpm_cmd = 0.0
    th_r = 0.0
    wm = 0.0
    ia = ib = 0.0
    i_max = i_avg = 0.0
    t_spin = None
    t_stall = None
    align_left = 0.3
    log = []
    i_sum = 0.0
    n_cnt = 0

    n = int(t_end / TS)
    for k in range(n):
        t = k * TS

        diff = rpm_target - rpm_cmd
        step = ramp * TS
        rpm_cmd += max(-step, min(step, diff))

        if align_left > 0:
            align_left -= TS
            v = valign
            theta = 0.0
        else:
            fe = rpm_cmd / 60.0 * PP
            if abs(fe) < fstart:
                fe = fstart if fe >= 0 else -fstart
            we = abs(fe) * 2 * math.pi
            v = vboost + slope * abs(fe)
            if (we * FLUX < 0.25 or abs(fe) < fres) and vlo > v:
                v = vlo
            v = min(v, vmax)
            theta = (theta + 2 * math.pi * fe * TS) % (2 * math.pi)

        # αβ 电压指令
        va = v * math.cos(theta)
        vb = v * math.sin(theta)

        # 反电动势(真实角)
        we = abs(rpm_cmd) / 60.0 * PP * 2 * math.pi
        ea = -we * FLUX * math.sin(th_r)
        eb = we * FLUX * math.cos(th_r)

        # 电流动态(前向欧拉)
        ia += TS / LS * (va - RS * ia - ea)
        ib += TS / LS * (vb - RS * ib - eb)
        imag = math.hypot(ia, ib)
        i_max = max(i_max, imag)
        i_sum += imag
        n_cnt += 1

        # 转矩: iq = 电流在真实 q 轴的分量
        iq = -ia * math.sin(th_r) + ib * math.cos(th_r)
        Te = 1.5 * PP * FLUX * iq
        Tload = T_LOAD + T_COG * math.sin(PP * th_r)
        if abs(wm) < 1e-3 and abs(Te - Tload) < T_STATIC:
            acc = 0.0
        else:
            acc = (Te - Tload - B_VISC * wm) / J
        wm += acc * TS
        th_r = (th_r + wm * TS * PP) % (2 * math.pi)
        rpm_real = wm / (2 * math.pi) * 60.0

        if t_spin is None and rpm_real > 30.0:
            t_spin = t * 1000.0
        if t_stall is None and align_left <= 0 and t > timeout_s and we * FLUX < 0.25:
            t_stall = t * 1000.0

        if not quiet and (k % int(0.25 / TS) == 0):
            log.append((t * 1000.0, v, imag, rpm_cmd, rpm_real, we * FLUX))

    i_avg = i_sum / max(1, n_cnt)
    rpm_end = wm / (2 * math.pi) * 60.0
    return {"ok": rpm_end > 50.0 and t_stall is None, "rpm_end": rpm_end,
            "i_max": i_max, "i_avg": i_avg, "t_spin": t_spin, "t_stall": t_stall,
            "log": log}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vlo", type=float)
    ap.add_argument("--fres", type=float)
    ap.add_argument("--rpm", type=float, default=300.0)
    ap.add_argument("--inertia", type=float, default=J_DEF, help="电机侧惯量 kg·m^2")
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()

    cfg = load_fw_defaults()
    print("固件默认: vlo=%.2fV fres=%.1fHz vboost=%.2f slope=%.3f fstart=%.1f ramp=%.0f "
          "失步超时=%.0fms" % (cfg["vlo"], cfg["fres"], cfg["vboost"], cfg["slope"],
                              cfg["fstart"], cfg["ramp"], cfg["timeout_ms"]))
    print("机械假设(粗估): J=%.2g kg.m2 齿槽=%.3f N.m 静摩擦=%.3f N.m"
          % (a.inertia, T_COG, T_STATIC))
    print("              (J 是最大不确定项, 用 --inertia 换假设看趋势)\n")

    if a.sweep:
        print("%-7s %-8s %8s %10s %10s %9s  %s" %
              ("vlo[V]", "fres[Hz]", "峰值I[A]", "起动耗时", "末速rpm", "失步", "结论"))
        for vlo in (1.0, 2.0, 2.8, 3.5, 4.0, 4.5, 5.5):
            for fres in (2.0, 6.0, 9.0, 12.0):
                r = run(vlo, fres, cfg, a.rpm, J=a.inertia, quiet=True)
                print("%-7.1f %-8.1f %8.1f %10s %10.0f %9s  %s" %
                      (vlo, fres, r["i_max"],
                       ("%.0fms" % r["t_spin"]) if r["t_spin"] else "--",
                       r["rpm_end"],
                       ("%.0fms" % r["t_stall"]) if r["t_stall"] else "--",
                       "能启动" if r["ok"] else "★起不来"))
        return

    vlo = a.vlo if a.vlo is not None else cfg["vlo"]
    fres = a.fres if a.fres is not None else cfg["fres"]
    r = run(vlo, fres, cfg, a.rpm, J=a.inertia)

    print("仿真: vlo=%.2fV fres=%.1fHz 目标=%.0frpm(电机侧)\n" % (vlo, fres, a.rpm))
    print("%8s %8s %8s %9s %9s %8s" % ("t[ms]", "v[V]", "I[A]", "rpm_cmd", "rpm_real", "|E|[V]"))
    for row in r["log"][:16]:
        print("%8.0f %8.2f %8.1f %9.1f %9.1f %8.3f" % row)
    print()
    print("峰值电流 %.1fA, 平均 %.1fA" % (r["i_max"], r["i_avg"]))
    if r["t_stall"]:
        print("★ %.0fms 触发失步保护(2s 内 |E| 没建立)" % r["t_stall"])
    if r["ok"]:
        print("结论: 能牵入, %.0f ms 转起来, 末速 %.0frpm" % (r["t_spin"], r["rpm_end"]))
    else:
        print("结论: ★起不来(末速 %.0frpm)" % r["rpm_end"])
        print("      起动转矩 ≈ 1.5·p·ψf·(v−e)/R, 每 +1V 约多 %.2f N.m(电机侧)"
              % (1.5 * PP * FLUX / RS))
        print("      对策: 提 vlo / 提 fres / 降 ramp; 若电流已上 20A 仍不转 -> 机械卡住或惯量太大")


if __name__ == "__main__":
    main()


