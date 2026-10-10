#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/ol_watch.py —— 只抓开环诊断行(# OL ...), 一条命令看清驱动在干什么

用法(不用串口助手, 也不用装 pyserial):
    python tools/ol_watch.py --port COM7                 # 只看, 持续打印
    python tools/ol_watch.py --port COM7 --cmd "prot 0" --cmd "vf 300"

它会:
  1. 打开串口, 把你给的 --cmd 依次发下去(每条间隔 0.4s);
  2. 持续打印所有以 '#' 开头的行(实时诊断 + STALL/故障 + TIM1 自检);
  3. Ctrl+C 退出, 并把整段存成 data/ol_<时间>.log。

★这一行是排查"电机不转"的关键, 它把问题分成三种:
    v_out≈0                 -> 电压没给出去(寄存器/EN/母线)
    v_out 有、I≈0            -> 驱动没输出或相线没接
    v_out 有、I 有、|E|=0     -> 电流给了但转子没跟上(转矩不够 / 失速 / 机械卡)
"""
import argparse
import ctypes
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class SerialPort(object):
    """ctypes 直开 Windows 串口(和 tools/smo_capture.py 里那套一样)"""

    def __init__(self, port, baud=115200):
        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self.handle = self.kernel32.CreateFileW(
            "\\\\.\\%s" % port, 0xC0000000, 0, None, 3, 0, None)
        if self.handle in (-1, 0xFFFFFFFF):
            raise OSError("打不开 %s (错误码 %d): 串口号对不对? 有没有被别的程序占用?"
                          % (port, ctypes.get_last_error()))
        dcb = ctypes.create_string_buffer(28)
        ctypes.memmove(dcb, ctypes.byref(ctypes.c_uint32(28)), 4)
        ctypes.memmove(ctypes.byref(dcb, 4), ctypes.byref(ctypes.c_uint32(baud)), 4)
        ctypes.memmove(ctypes.byref(dcb, 8), ctypes.byref(ctypes.c_ubyte(1)), 1)
        if not self.kernel32.SetCommState(self.handle, dcb):
            raise OSError("SetCommState 失败")
        tmo = ctypes.create_string_buffer(20)
        for i, v in enumerate((0, 0, 200, 2, 0)):
            ctypes.memmove(ctypes.byref(tmo, i * 4), ctypes.byref(ctypes.c_uint32(v)), 4)
        self.kernel32.SetCommTimeouts(self.handle, tmo)
        self.kernel32.PurgeComm(self.handle, 0x0004 | 0x0008)
        self.buf = ctypes.create_string_buffer(8192)
        self.n = ctypes.c_uint32(0)

    def write(self, text):
        n = ctypes.c_uint32(0)
        data = text.encode("ascii", "ignore")
        self.kernel32.WriteFile(self.handle, data, len(data), ctypes.byref(n), None)

    def read(self):
        ok = self.kernel32.ReadFile(self.handle, self.buf, len(self.buf),
                                    ctypes.byref(self.n), None)
        if not ok or self.n.value == 0:
            return b""
        return self.buf.raw[:self.n.value]

    def close(self):
        if self.handle not in (-1, 0xFFFFFFFF):
            self.kernel32.CloseHandle(self.handle)
            self.handle = -1


OL_RE = re.compile(r"v_out=([\d\.\-]+)V\s+I=([\d\.\-]+)A\s+\|E\|=([\d\.\-]+)V")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="串口号, 如 COM7")
    ap.add_argument("--cmd", action="append", default=[],
                    help="打开后要发的命令, 可重复, 例: --cmd \"prot 0\" --cmd \"vf 300\"")
    ap.add_argument("--seconds", type=float, default=15.0, help="抓多久(默认 15s)")
    a = ap.parse_args()

    out_dir = ROOT / "data"
    out_dir.mkdir(exist_ok=True)
    log_path = out_dir / ("ol_%s.log" % datetime.now().strftime("%Y%m%d_%H%M%S"))

    ser = SerialPort(a.port)
    print("已打开 %s, 日志 -> %s" % (a.port, log_path))
    print("(先等一下, 板子复位后头几行是开机自检)\\n")

    # 先清掉上电自检的残留, 再发命令
    t0 = time.time()
    while time.time() - t0 < 1.5:
        ser.read()

    cmds = a.cmd if a.cmd else ["stat"]
    for c in cmds:
        ser.write(c + "\n")
        print(">>> %s" % c)
        time.sleep(0.4)

    buf = b""
    lines = []
    samples = []
    t_end = time.time() + a.seconds
    try:
        while time.time() < t_end:
            chunk = ser.read()
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                s = raw.decode("ascii", "replace").strip()
                if s.startswith("#") and len(s) > 1:
                    print(s)
                    lines.append(s)
                    m = OL_RE.search(s)
                    if m:
                        samples.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    except KeyboardInterrupt:
        print("\n(手动中断)")
    finally:
        ser.write("stop\n")
        time.sleep(0.15)
        ser.close()

    log_path.write_text("\n".join(lines), encoding="utf-8")

    print("\n" + "=" * 66)
    if not lines:
        print("★ 一行都没收到。说明: 串口选错了 / 板子没在跑 / USB CDC 没枚举。")
        print("   先确认: 设备管理器里有你板子的 'USB 串行设备 (COMxx)'")
        return 1

    if not samples:
        print("收到 %d 行诊断, 但没有 'v_out/I/|E|' 那种实时行。" % len(lines))
        print("说明多半没进开环 —— 检查有没有发 'vf <rpm>' 成功(vf 会回一行 open-loop ON)")
        return 1

    v_max = max(s[0] for s in samples)
    i_max = max(s[1] for s in samples)
    e_max = max(s[2] for s in samples)
    v_avg = sum(s[0] for s in samples) / len(samples)
    i_avg = sum(s[1] for s in samples) / len(samples)

    print("抓到 %d 个采样点:" % len(samples))
    print("  输出电压 v_out: 峰值 %.2fV, 平均 %.2fV" % (v_max, v_avg))
    print("  电流     I    : 峰值 %.1fA, 平均 %.1fA" % (i_max, i_avg))
    print("  反电动势 |E|  : 峰值 %.3fV" % e_max)
    print("=" * 66)
    print("结论:")
    if v_max < 0.5:
        print("  v_out ≈ 0 -> **电压根本没给出去**。查: stat 的 TIM1 自检行(CR1/CCER)、")
        print("             EN_Gate 是否使能、母线 vdc 是否正常。")
    elif i_max < 0.5:
        print("  v_out 有(%.2fV)但 I ≈ 0 -> **驱动没输出电流**。查:" % v_max)
        print("             相线有没有接、EN_Gate、驱动芯片供电、TIM1 通道是否真在输出。")
    elif e_max < 0.10:
        print("  电压给了、电流也有(峰值 %.1fA), 但 |E| 始终为 0 -> " % i_max)
        print("  **转子没跟上旋转磁场**(电流全变成堵转发热)。两种可能:")
        print("    a) 起动转矩不够(真实相电阻比 0.194Ω 大很多?)-> 提高 vlo 到 4~5V 再试;")
        print("    b) 机械卡住/负载太重 -> 断电用手盘输出轴, 应该很轻快。")
        print("  ★注意: 这种情况电流会很大, 限流先设 2~3A, 别长时间堵着。")
    else:
        print("  |E| 已经建立到 %.3fV -> 电机确实在转(或至少抖起来了)。" % e_max)
        if e_max >= 0.35:
            print("  已经超过 'running' 门限, 这时可以 prot 1 打开保护, 再 stream 1 采数据。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
