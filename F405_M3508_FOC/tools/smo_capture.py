#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/smo_capture.py —— 采集滑模观测器打点数据(USB 虚拟串口)

固件在开环 V/F 模式下按 1kHz 输出 CSV 行:
    t_ms,theta_obs,theta_enc,err,emf,omega_obs,rpm_obs,rpm_enc,ialpha,ibeta,valpha,vbeta,vdc
以 '#' 开头的行是状态/摘要行, 单独存到 .log。

★ 只用标准库: Windows 下直接用 ctypes 开 COM 口, 不依赖 pyserial
  (上位机的 python 未必装了 pyserial, 而这里只需要一个很简单的读循环)。

用法示例:
    # 只采集(电机已经用 CAN/串口命令开起来了)
    python tools/smo_capture.py --port COM7 --seconds 10 --out data/smo_run1.csv

    # 采集前先发命令进入开环并开始打点(命令直接走同一个串口, 不用 CAN 盒)
    python tools/smo_capture.py --port COM7 --seconds 10 \
        --cmd "stream 1" --cmd "vf 800" --cmd "vf 1500" --cmd "vf 2500"

    # 自动扫速: 800 -> 1500 -> 2500, 每档 5s
    python tools/smo_capture.py --port COM7 --sweep 800,1500,2500 --dwell 5

结束时(或 Ctrl+C)会自动做一次分析, 也可单独分析已存文件:
    python tools/smo_capture.py --analyze data/smo_run1.csv
"""

import argparse
import csv
import ctypes
import math
import os
import sys
import time

CSV_HEADER = ["t_ms", "theta_obs", "theta_enc", "err", "emf", "omega_obs",
              "rpm_obs", "rpm_enc", "ialpha", "ibeta", "valpha", "vbeta", "vdc",
              "imag", "vcmd", "flags"]


# --------------------------------------------------------------------------
# Windows 串口(ctypes, 无第三方依赖)
# --------------------------------------------------------------------------
class SerialPort(object):
    def __init__(self, port, baud=115200):
        if os.name != "nt":
            raise RuntimeError("这个简易串口实现只支持 Windows; Linux 下请用 pyserial")
        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self.handle = self.kernel32.CreateFileW(
            "\\\\.\\%s" % port, 0xC0000000, 0, None, 3, 0, None)
        if self.handle == -1 or self.handle == 0xFFFFFFFF:
            raise OSError("打不开 %s (错误码 %d): 确认串口号 / 是否被别的程序占用"
                          % (port, ctypes.get_last_error()))
        # DCB: 波特率对 USB CDC 无意义, 但必须设一个合法值
        dcb = ctypes.create_string_buffer(28)
        ctypes.memmove(dcb, ctypes.byref(ctypes.c_uint32(28)), 4)
        ctypes.memmove(ctypes.byref(dcb, 4), ctypes.byref(ctypes.c_uint32(baud)), 4)
        ctypes.memmove(ctypes.byref(dcb, 8), ctypes.byref(ctypes.c_ubyte(1)), 1)  # fBinary
        if not self.kernel32.SetCommState(self.handle, dcb):
            raise OSError("SetCommState 失败")
        timeouts = ctypes.create_string_buffer(20)
        for i, v in enumerate((0, 0, 200, 2, 0)):   # ReadInterval/Total/RdMult/WrMult
            ctypes.memmove(ctypes.byref(timeouts, i * 4), ctypes.byref(ctypes.c_uint32(v)), 4)
        self.kernel32.SetCommTimeouts(self.handle, timeouts)
        self.kernel32.PurgeComm(self.handle, 0x0004 | 0x0008)   # RXABORT|TXABORT 清缓冲
        self.buf = ctypes.create_string_buffer(8192)
        self.read_n = ctypes.c_uint32(0)

    def write(self, text):
        n = ctypes.c_uint32(0)
        data = text.encode("ascii", "ignore")
        self.kernel32.WriteFile(self.handle, data, len(data),
                                ctypes.byref(n), None)

    def read(self):
        ok = self.kernel32.ReadFile(self.handle, self.buf, len(self.buf),
                                    ctypes.byref(self.read_n), None)
        if not ok or self.read_n.value == 0:
            return b""
        return self.buf.raw[:self.read_n.value]

    def close(self):
        if self.handle not in (-1, 0xFFFFFFFF):
            self.kernel32.CloseHandle(self.handle)
            self.handle = -1


# --------------------------------------------------------------------------
# 采集
# --------------------------------------------------------------------------
def capture(port, seconds, out_path, cmds, sweep, dwell, quiet=False):
    ser = SerialPort(port)
    rows = []
    comments = []
    buf = b""
    t0 = time.time()
    pending = list(cmds)
    sweep_list = [float(v) for v in sweep.split(",")] if sweep else []
    sweep_i = 0
    next_event = 0.0

    if sweep_list:
        pending.append("stream 1")
        pending.append("vf %.0f" % sweep_list[0])
        next_event = time.time() + dwell

    def emit(text):
        if not quiet:
            sys.stdout.write(text if text.endswith("\n") else text + "\n")
            sys.stdout.flush()

    for c in pending:
        ser.write(c + "\n")
        emit("# -> %s" % c)

    emit("# 采集 %s 共 %.1fs%s" % (port, seconds,
                                  ", 扫速 %s" % sweep_list if sweep_list else ""))

    try:
        while time.time() - t0 < seconds:
            now = time.time()
            if sweep_list and now >= next_event:
                sweep_i += 1
                if sweep_i < len(sweep_list):
                    cmd = "vf %.0f" % sweep_list[sweep_i]
                    ser.write(cmd + "\n")
                    emit("# -> %s (t=%.1fs)" % (cmd, now - t0))
                    next_event = now + dwell
                else:
                    next_event = float("inf")

            chunk = ser.read()
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode("ascii", "replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    comments.append(line)
                    emit(line)
                    continue
                parts = line.split(",")
                if len(parts) != len(CSV_HEADER):
                    continue
                try:
                    rows.append([float(x) for x in parts])
                except ValueError:
                    continue
    except KeyboardInterrupt:
        emit("# 用户中断, 停在这里")
    finally:
        ser.write("stop\n")
        time.sleep(0.1)
        ser.close()

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        f.write("# " + " ".join(comments[:40]) + "\n")
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)

    emit("# 已保存 %d 行 -> %s" % (len(rows), out_path))
    return out_path


# --------------------------------------------------------------------------
# 分析
# --------------------------------------------------------------------------
def wrap_pi(x):
    while x >= math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def load_csv_robust(path):
    """读 CSV, 返回 (mode, header, data)。

    ★不靠 "首字符是不是 #" 来过滤注释 —— 这里踩过坑(同一行在 CRLF/无 BOM 等
      不同读法下前缀判定会给出不一致的结果)。改成:
        直接在整份文本里找"含 t_ms 的那一行"当表头, 它之后的才是数据。
      这样注释行、空行、行尾差异全都不影响。"""
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        text = f.read()

    mode = None
    for ln in text.splitlines():
        if "drv=" in ln:
            mode = "openloop" if "drv=openloop" in ln else "encoder"

    lines = [ln.strip() for ln in text.splitlines()]
    hi = None
    for i, ln in enumerate(lines):
        if "t_ms" in ln:
            hi = i
            break
    if hi is None:
        return mode, None, []

    # 表头第一列在固件里写成 "#t_ms"(# 是行首标记, 粘在列名上了), 这里统一去掉
    header = [c.strip().lstrip("#").strip() for c in lines[hi].split(",")]
    data = []
    for ln in lines[hi + 1:]:
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        try:
            data.append([float(v) for v in parts])
        except ValueError:
            continue
    return mode, header, data


def analyze(path, plot=None, mode_force=None):
    """分析一次采集。

    两种模式(从文件头的 drv= 自动判断, 也可以用 --mode 强制):
      开环(无编码器): 只能看 |E| 是否建立、观测转速是否跟上、观测角是否在匀速转;
                      编码器三列(theta_enc/err/rpm_enc)无意义。
      编码器对照    : 额外给出观测角与编码器角的残差(这才是精度指标)。
    """
    mode, header, data = load_csv_robust(path)
    if mode_force:
        mode = mode_force
    if not header or not data:
        print("没读到数据行(文件: %s)" % path)
        return
    idx = {name: i for i, name in enumerate(header)}

    t = [r[idx["t_ms"]] for r in data] if "t_ms" in idx else [float(i) for i in range(len(data))]

    print("=" * 78)
    print("文件: %s   模式: %s" % (path, mode or "未知(没读到 # 头)"))
    dur = (t[-1] - t[0]) / 1000.0 if len(t) > 1 else 0.0
    print("样本: %d 行, 时长 %.2fs, 平均速率 %.0f Hz"
          % (len(data), dur, len(data) / dur if dur > 0 else 0))

    if mode == "openloop":
        analyze_openloop(data, idx)
    else:
        analyze_encoder(data, idx)
        if mode is None:
            print("   (提示: 若这是无编码器的开环采集, 上面那组数字没有意义, 用 --mode openloop)")

    if plot:
        try:
            make_plot(path, data, idx, plot)
            print("图已保存 -> %s" % plot)
        except ImportError:
            print("(没装 matplotlib, 跳过画图)")
    return data, idx


def analyze_openloop(data, idx):
    """无编码器: 判据是 |E| 建立 + 观测转速跟上 + 观测角匀速。

    堵转诊断: 有 imag(电流幅值)/vcmd(输出电压)/vdc(母线) 三列之后, 可以分清
      "电压给不出去"(母线/限幅) 还是 "电压给了但电流不够"(静摩擦/限幅) 还是
      "电流有、电压有、就是不转"(失步/相序/对齐)。
    """
    have_i = "imag" in idx
    have_v = "vcmd" in idx

    t = [r[idx["t_ms"]] / 1000.0 for r in data]
    emf = [r[idx["emf"]] for r in data]
    rpm_obs = [r[idx["rpm_obs"]] for r in data]
    theta_obs = [r[idx["theta_obs"]] for r in data]
    imag = [r[idx["imag"]] for r in data] if have_i else None
    vcmd = [r[idx["vcmd"]] for r in data] if have_v else None
    vdc = [r[idx["vdc"]] for r in data]

    n = len(data)
    good = [i for i in range(n) if emf[i] >= 0.30]
    print("-" * 78)
    print("|E| 有效(|E|>=0.30V)样本: %d / %d (%.0f%%)" % (len(good), n, 100.0 * len(good) / n))

    # ---- 堵转诊断(不管有没有 |E| 都先给出来) ----
    if imag and vcmd:
        i_max = max(imag)
        i_avg = sum(imag) / n
        v_max = max(vcmd)
        vdc_min = min(vdc)
        print("电压: 输出峰值 %.2fV, 母线最低 %.2fV  电流: 峰值 %.1fA, 平均 %.1fA"
              % (v_max, vdc_min, i_max, i_avg))
        if not good:
            print()
            print("★ 全程没有反电动势 —— 电机没转。按下面顺序定位:")
            if v_max < 0.5:
                print("  ① 输出电压几乎为 0: 检查 vf 命令是否真的进去了(敲 stat), "
                      "以及 TIM1 自检行 CR1/CCER")
            elif i_max < 1.0:
                print("  ① 有电压但没有电流(峰值 %.1fA): 驱动没输出/EN 没使能/"
                      "相线没接好 -> 先量三相是否有 PWM" % i_max)
            elif i_max >= 5.0:
                print("  ② 电压 %.2fV 下电流已经 %.1fA 还不转: 纯粹是起动转矩不够"
                      " -> 加大 vlo(3.5~4.5V)、fres 提到 8~10Hz、母线限流放到 5A"
                      % (v_max, i_max))
                print("     也可能是减速箱/负载卡住: 断电用手盘一下输出轴是否顺畅")
            else:
                print("  ② 电流 %.1fA、电压 %.2fV 都给了但不转: 可能是失步(相序/对齐)"
                      % (i_max, v_max))
                print("     试: fstart 提到 3~5Hz、ramp 降到 200、drv 1(若有编码器)")
            return

    if not good:
        print("★ 全程 |E| < 0.30V —— 观测器看不到反电动势, 说明电机根本没转起来:")
        print("  · 检查 stat 里的 STALL / vdc_fault 是否触发(固件会在超时后自动封驱动)")
        print("  · 调大 vlo/vboost(起步电压)、确认母线电压、确认 vf 命令真的进去了")
        return

    # |E| 与观测转速的对应关系: |E| ≈ ωe·ψf, 反推磁链, 顺带验证两条曲线是否自洽
    we = [abs(r[idx["omega_obs"]]) for r in data]
    ratios = [emf[i] / we[i] for i in good if we[i] > 50.0]
    if ratios:
        flux = sum(ratios) / len(ratios)
        print("反推磁链 |E|/|ωe| = %.3f mWb (代码里 FOC_FLUX = 1.000 mWb)" % (flux * 1000.0))

    # 观测转速 vs |E|: 应该有 |E| 时转速也接近指令
    print("-" * 78)
    if imag and vcmd:
        print("%-9s %7s %7s %7s %9s %9s %9s"
              % ("时间[s]", "|E|[V]", "I[A]", "Vout", "rpm_obs", "θ_obs", "θ速率"))
    else:
        print("%-9s %7s %9s %9s %9s" % ("时间[s]", "|E|[V]", "rpm_obs", "θ_obs", "θ速率"))
    step = max(1, len(data) // 12)
    for i in range(0, len(data), step):
        # 用相邻 100 点估角度速率(等效转速), 避免被 wrap 骗
        j = min(i + 100, len(data) - 1)
        dth = theta_obs[j] - theta_obs[i]
        while dth > math.pi:
            dth -= 2 * math.pi
        while dth < -math.pi:
            dth += 2 * math.pi
        dt = (t[j] - t[i]) or 1e-9
        rate = dth / dt / (2 * math.pi) * 60.0 / 7.0
        if imag and vcmd:
            print("%-9.2f %7.2f %7.1f %7.2f %9.1f %9.1f %9.1f"
                  % (t[i], emf[i], imag[i], vcmd[i], rpm_obs[i],
                     math.degrees(theta_obs[i] % (2 * math.pi)), rate))
        else:
            print("%-9.2f %7.2f %9.1f %9.1f %9.1f"
                  % (t[i], emf[i], rpm_obs[i],
                     math.degrees(theta_obs[i] % (2 * math.pi)), rate))

    # 结论
    tail = good[-max(1, len(good) // 10):]
    rpm_tail = [abs(rpm_obs[i]) for i in tail]
    emf_tail = [emf[i] for i in tail]
    rpm_avg = sum(rpm_tail) / len(rpm_tail)
    emf_avg = sum(emf_tail) / len(emf_tail)
    print("-" * 78)
    print("末段平均: |E| = %.2f V, rpm_obs = %.0f rpm" % (emf_avg, rpm_avg))
    if imag:
        i_tail = [imag[i] for i in tail]
        print("末段平均电流: %.1f A(峰值 %.1f A)" % (sum(i_tail) / len(i_tail), max(i_tail)))
    if rpm_avg > 100.0:
        print("判定: 电机确实转起来了(有反电动势、观测器给出了转速) —— 开环运行成立。")
        print("      下一步: 想要闭环无感, 就把驱动角从'积分角'平滑交给'观测角'。")
    else:
        print("判定: |E| 有了但观测转速很小 —— 可能只是抖动/极低速, 提高 vlo 再来。")


def analyze_encoder(data, idx):
    """有编码器: 给观测角 vs 编码器角的残差(精度指标)"""
    t = [r[idx["t_ms"]] for r in data]
    emf = [r[idx["emf"]] for r in data]
    err = [r[idx["err"]] for r in data]
    rpm_enc = [r[idx["rpm_enc"]] for r in data]
    rpm_obs = [r[idx["rpm_obs"]] for r in data]

    print("-" * 78)
    print("%-12s %7s %9s %9s %9s %9s" %
          ("实测转速档", "样本", "err均值", "err标准差", "|E|均值", "rpm观测误差"))
    bins = {}
    for i in range(len(data)):
        key = int(abs(rpm_enc[i]) // 250.0) * 250
        bins.setdefault(key, []).append(i)
    for key in sorted(bins):
        g = bins[key]
        if len(g) < 20:
            continue

        def mean(vals):
            return sum(vals) / len(vals)

        e = [err[i] for i in g]
        m = mean(e)
        sd = math.sqrt(max(0.0, mean([x * x for x in e]) - m * m))
        drpm = mean([rpm_obs[i] - rpm_enc[i] for i in g])
        print("%-12s %7d %+9.4f %9.4f %9.3f %+9.1f" %
              ("%d-%d" % (key, key + 250), len(g), m, sd,
               mean([emf[i] for i in g]), drpm))

    valid = [i for i in range(len(data)) if emf[i] >= 0.6]
    print("-" * 78)
    if valid:
        e = [err[i] for i in valid]
        m = sum(e) / len(e)
        sd = math.sqrt(max(0.0, sum(x * x for x in e) / len(e) - m * m))
        print("|E|>=0.6V 的样本 %d 个(%.0f%%): err 均值 %+.4f rad (%.2f°), 标准差 %.4f rad (%.2f°)"
              % (len(valid), 100.0 * len(valid) / len(data),
                 m, math.degrees(m), sd, math.degrees(sd)))
        print("判定: %s" % judge(m, sd))
    else:
        print("★ 没有任何样本的 |E| 达到 0.6V —— 观测器没进入有效区:")
        print("   可能原因: 转速太低(反电动势被噪声淹没) / 电压没加出去 /")
        print("             SMO 输入接错(滑模增益 H 或边界层 δ 没初始化)")


def judge(mean_err, std_err):
    d = math.degrees(mean_err)
    s = math.degrees(std_err)
    if s > 20.0:
        return "误差波动太大 —— 观测器没真正跟踪(检查 H/δ 与电流采样)"
    if abs(d) > 15.0 and s < 5.0:
        return ("误差基本恒定 %.1f° —— 是常数偏置(角度对齐/一拍延迟), "
                "用 USB 命令 off <rad> 补偿即可, 不算精度问题" % d)
    if abs(d) > 15.0:
        return "有系统性偏差且波动 %.1f° —— 先按常数偏置补偿, 再看残差" % s
    if s < 2.0:
        return "很好: 偏差 %.1f°、波动 %.1f°" % (d, s)
    return "可用: 偏差 %.1f°、波动 %.1f°" % (d, s)


def make_plot(path, data, idx, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r[idx["t_ms"]] / 1000.0 for r in data]
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].plot(t, [math.degrees(r[idx["theta_obs"]] % (2 * math.pi)) for r in data],
               label="observer", lw=1, color="tab:orange")
    if max(abs(r[idx["rpm_enc"]]) for r in data) > 1.0:      # 有编码器才画
        ax[0].plot(t, [math.degrees(r[idx["theta_enc"]] % (2 * math.pi)) for r in data],
                   label="encoder", lw=1)
    ax[0].set_ylabel("elec angle [deg]")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, [r[idx["emf"]] for r in data], lw=1, color="tab:green")
    ax[1].set_ylabel("|E| [V]")
    ax[2].plot(t, [r[idx["rpm_obs"]] for r in data], label="observer", lw=1)
    if max(abs(r[idx["rpm_enc"]]) for r in data) > 1.0:
        ax[2].plot(t, [r[idx["rpm_enc"]] for r in data], label="encoder", lw=1)
    ax[2].set_ylabel("rpm (motor side)")
    ax[2].set_xlabel("t [s]")
    ax[2].legend(loc="upper right")
    for a in ax:
        a.grid(True, alpha=0.3)
    fig.suptitle(os.path.basename(path))
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)


def main():
    ap = argparse.ArgumentParser(description="滑模观测器打点采集/分析")
    ap.add_argument("--port", help="串口号, 如 COM7")
    ap.add_argument("--seconds", type=float, default=10.0, help="采集时长 s")
    ap.add_argument("--out", default="data/smo_capture.csv", help="输出 CSV")
    ap.add_argument("--cmd", action="append", default=[],
                    help="采集前发送的文本命令, 可重复")
    ap.add_argument("--sweep", help="自动扫速, 如 800,1500,2500")
    ap.add_argument("--dwell", type=float, default=5.0, help="每档停留 s")
    ap.add_argument("--analyze", help="只分析已有 CSV")
    ap.add_argument("--mode", choices=["openloop", "encoder"],
                    help="分析模式(默认按文件头 drv= 自动判断)")
    ap.add_argument("--plot", help="顺便存一张 PNG")
    ap.add_argument("--list-ports", action="store_true", help="列出可用串口")
    a = ap.parse_args()

    if a.list_ports:
        import serial.tools.list_ports as lp  # 有 pyserial 才列
        for p in lp.comports():
            print(p.device, p.description)
        return

    if a.analyze:
        analyze(a.analyze, a.plot, a.mode)
        return

    if not a.port:
        ap.error("要么给 --port 采集, 要么给 --analyze 分析已有文件")

    path = capture(a.port, a.seconds, a.out, a.cmd, a.sweep, a.dwell)
    analyze(path, a.plot, a.mode)


if __name__ == "__main__":
    main()
