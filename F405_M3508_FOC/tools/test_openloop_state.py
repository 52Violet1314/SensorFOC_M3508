#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/test_openloop_state.py —— 开环状态机回归测试(不需要板子)

把 VF.c + App_FOC.c 里那段"失步保护 + 使能"逻辑原样搬到 Python, 验证 4 件事:

  1. 第一次启动没转起来 -> 2000ms 后触发失步保护并封驱动   (保护有效)
  2. ★再次 vf 必须能重新起来, 不能一拍就被封死             (这就是"转都不转"那个 bug)
  3. 对齐 300ms 窗口内不许计时, 也不许误报失步
  4. 指令电频率低于"该转速下反电动势才可能出现"的门槛时不许计时
     (fe_min = EMF_TH/(2π·ψf)), 否则低速段会被误判成堵转

用法: python tools/test_openloop_state.py
"""
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TS = 25e-6          # 40kHz
DT_MS = 25e-3

PP = 7.0
FLUX = 1.0e-3


def read_def(path, name, default):
    text = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"^#define\s+%s\s+([0-9eE\.\+\-\*\(\)a-zA-Z_/fF ]+?)(?:\s*/\*|\s*$)"
                  % re.escape(name), text, re.M)
    if not m:
        return default
    expr = m.group(1).strip().rstrip("fF").strip()
    try:
        return float(eval(expr, {"__builtins__": {}}, {}))
    except Exception:
        return default


# ---- 与固件一致的常量(直接从源码读, 避免手抄漂移) ----
EMF_TH = read_def(ROOT / "FOC/inc/FOC_Param.h", "OBS_EMF_MIN_V", 0.3)
RUN_EMF = read_def(ROOT / "FOC/inc/VF.h", "VF_RUN_EMF_V", 0.25)
TIMEOUT_MS = read_def(ROOT / "FOC/inc/FOC_Param.h", "OL_EMF_TIMEOUT_MS", 2000.0)
ALIGN_MS = read_def(ROOT / "FOC/inc/FOC_Param.h", "OL_ALIGN_MS", 300.0)
FSTART = read_def(ROOT / "FOC/inc/FOC_Param.h", "OL_FSTART_DEF", 2.0)


class Drive(object):
    """VF.c + App_FOC.c 里失步保护/使能那段逻辑的等价实现"""

    def __init__(self):
        self.enabled = False
        self.stall = False
        self.align_done = False
        self.align_cnt = 0.0
        self.emf_low_t0 = None
        self.emf_low_ms = 0.0
        self.running = False
        self.tripped = 0          # 被保护封驱动的次数
        self.rpm_cmd = 0.0

    # --- VF_SetEnable ---
    def enable(self, on):
        if on:
            self.rpm_cmd = 0.0
            self.running = False
            self.stall = False
            self.emf_low_t0 = None
            self.emf_low_ms = 0.0
            self.align_cnt = ALIGN_MS
            self.align_done = False
        else:
            self.rpm_cmd = 0.0
            self.stall = False
        self.enabled = bool(on)

    # --- 一拍: align_done 推进 + 失步判定(VF_Tick 第 3/7 步) ---
    def tick(self, now_ms, emf_obs, v_out):
        if not self.enabled:
            return
        # 对齐推进
        if not self.align_done:
            if self.align_cnt > 0.0:
                self.align_cnt -= 1.0
            else:
                self.align_done = True
        if not self.align_done:
            return
        # 指令电频率(最低 fstart)
        fe = self.rpm_cmd / 60.0 * PP
        if abs(fe) < FSTART:
            fe = FSTART if fe >= 0 else -FSTART
        fe_min = RUN_EMF / (2.0 * math.pi * FLUX)
        if abs(fe) < fe_min:
            self.emf_low_t0 = None
            self.emf_low_ms = 0.0
        elif emf_obs >= RUN_EMF:
            self.running = True
            self.emf_low_t0 = None
            self.emf_low_ms = 0.0
        elif abs(v_out) > 0.5:
            if self.emf_low_t0 is None:
                self.emf_low_t0 = now_ms
            self.emf_low_ms = now_ms - self.emf_low_t0
            if self.emf_low_ms >= TIMEOUT_MS:
                self.stall = True
        else:
            self.emf_low_t0 = None

    # --- App_AdcStep 里的那一段 ---
    def isr(self, now_ms, emf_obs, v_out, rpm_cmd):
        self.rpm_cmd = rpm_cmd
        self.tick(now_ms, emf_obs, v_out)
        if self.stall:
            self.tripped += 1
            self.enable(False)     # 封驱动
            self.stall = False     # ★关键: 清锁, 允许再次 vf
            return True
        return False


def sim(drive, seconds, emf_of_cmd, v_out=2.8, rpm_target=300.0, ramp=200.0):
    """跑一段: 每个 tick 用 emf_of_cmd(rpm_cmd, t) 给出观测到的 |E|"""
    n = int(seconds / TS)
    rpm = 0.0
    last = None
    for k in range(n):
        t_ms = k * DT_MS
        step = ramp * DT_MS / 1000.0
        rpm += max(-step, min(step, rpm_target - rpm))
        if drive.isr(t_ms, emf_of_cmd(rpm, t_ms / 1000.0), v_out, rpm):
            last = t_ms
    return last


def main():
    print("常量: RUN_EMF=%.2fV TIMEOUT=%.0fms ALIGN=%.0fms FSTART=%.1fHz" %
          (RUN_EMF, TIMEOUT_MS, ALIGN_MS, FSTART))
    print("失步门限电频率 fe_min = %.2fHz (电机侧 %.1frpm)"
          % (RUN_EMF / (2 * math.pi * FLUX),
             RUN_EMF / (2 * math.pi * FLUX) / PP * 60))
    print("注意: RUN_EMF 必须 >= OBS_EMF_MIN_V(%.2fV), 否则永远判不出'已牵入'\n"
          % EMF_TH)
    if RUN_EMF < EMF_TH:
        print("★ 常量自检失败: VF_RUN_EMF_V(%.2f) < OBS_EMF_MIN_V(%.2f)"
              % (RUN_EMF, EMF_TH))
        raise SystemExit(1)

    fails = []

    # --- 用例1: 电机不转(观测始终没有 |E|) -> 应触发保护 ---
    # 注意: 目标转速要高于 fe_min 对应的转速(否则频率门限会一直抑制计时, 这是设计行为)
    d = Drive()
    d.enable(True)
    t_trip = sim(d, 3.0, lambda rpm, t: 0.02, ramp=600.0, rpm_target=1000.0)
    # 计时从"指令频率超过 fe_min"那一刻开始: t_ramp = fe_min/PP*60/ramp, 之后再过 TIMEOUT
    t_ramp_ms = (RUN_EMF / (2 * math.pi * FLUX)) / PP * 60.0 / 600.0 * 1000.0
    expect_ms = t_ramp_ms + TIMEOUT_MS
    ok1 = (t_trip is not None) and (abs(t_trip - expect_ms) <= 150.0)
    print("[%s] 用例1 卡住不转 -> %.0fms 触发失步保护(期望 %.0fms = 到达门限 %.0fms + 超时 %.0fms)" %
          ("PASS" if ok1 else "FAIL", t_trip if t_trip else -1, expect_ms, t_ramp_ms, TIMEOUT_MS))
    if not ok1:
        fails.append("用例1")

    # --- 用例2: 触发之后再次 vf, 必须能重新跑满, 而不是第 1 拍就被封 ---
    d.enable(True)                      # 用户再次敲 vf
    n_trip2 = 0
    first_trip_ms = None
    rpm = 0.0
    n = int(3.0 / TS)
    for k in range(n):
        t_ms = k * DT_MS
        rpm += 600.0 * DT_MS / 1000.0
        if rpm > 1000.0:
            rpm = 1000.0
        if d.isr(t_ms, 0.02, 2.8, rpm):
            n_trip2 += 1
            if first_trip_ms is None:
                first_trip_ms = t_ms
    ok2 = (n_trip2 == 1) and (first_trip_ms is not None) and (first_trip_ms >= TIMEOUT_MS - 50.0)
    print("[%s] 用例2 再次使能 -> 跑了 %.0fms 才触发保护, 共触发 %d 次(期望 1 次、约 %.0fms)" %
          ("PASS" if ok2 else "FAIL", first_trip_ms if first_trip_ms else -1, n_trip2, TIMEOUT_MS))
    if not ok2:
        fails.append("用例2")

    # --- 用例3: 对齐窗口内不许触发(哪怕 |E|=0) ---
    d3 = Drive()
    d3.enable(True)
    sim(d3, ALIGN_MS / 1000.0 * 0.8, lambda rpm, t: 0.0)
    ok3 = d3.tripped == 0
    print("[%s] 用例3 对齐窗口(%.0fms)内不误报, 触发次数=%d"
          % ("PASS" if ok3 else "FAIL", ALIGN_MS, d3.tripped))
    if not ok3:
        fails.append("用例3")

    # --- 用例4: 频率门限以下的低速段不许计时 ---
    d4 = Drive()
    d4.enable(True)
    d4.align_done = True
    fe_min = RUN_EMF / (2 * math.pi * FLUX)
    rpm_low = fe_min / PP * 60.0 * 0.5          # 频率只有门限的一半
    for k in range(int(5.0 / TS)):
        d4.isr(k * DT_MS, 0.0, 2.8, rpm_low)
        if d4.tripped:
            break
    ok4 = d4.tripped == 0
    print("[%s] 用例4 低频段(%.1frpm, 低于 fe_min)不计时, 触发次数=%d"
          % ("PASS" if ok4 else "FAIL", rpm_low, d4.tripped))
    if not ok4:
        fails.append("用例4")

    # --- 用例5: 正常牵入, 全程不许触发 ---
    d5 = Drive()
    d5.enable(True)
    def emf_ok(rpm, t):
        we = abs(rpm) / 60.0 * PP * 2 * math.pi
        return we * FLUX                     # 真实反电动势
    sim(d5, 3.0, emf_ok, ramp=600.0, rpm_target=1000.0)
    ok5 = d5.tripped == 0 and d5.running
    print("[%s] 用例5 正常牵入 -> 触发次数=%d running=%s"
          % ("PASS" if ok5 else "FAIL", d5.tripped, d5.running))
    if not ok5:
        fails.append("用例5")

    print()
    if fails:
        print("★ 失败用例: %s" % ", ".join(fails))
        raise SystemExit(1)
    print("全部通过: 保护有效、可重复使能、对齐与低频段不会误判。")


if __name__ == "__main__":
    main()
