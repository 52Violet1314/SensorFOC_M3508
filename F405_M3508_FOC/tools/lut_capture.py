#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/lut_capture.py —— 收编码器 2000 点校正表, 存档 + 生成固化的 Encoder_Lut.c

两种用法:

  1) 直接从串口收(板子已经在跑, 串口没被别的软件占着):
        python tools/lut_capture.py --port COM24
     会自动发一条 `lut` 命令, 收 2000 行 "idx,rad", 存 tools/enc_lut.csv,
     并顺手打印表的统计(直流分量=电角度对齐偏差, 峰峰值=编码器周期误差)。

  2) 已经有导出文件(用任意串口助手存下来的):
        python tools/lut_capture.py --in tools/enc_lut.csv

  生成固化表(覆盖 FOC/src/Encoder_Lut.c, 之后重新编译烧录就永久生效):
        python tools/lut_capture.py --in tools/enc_lut.csv --gen
        (也可以 --port COM24 --gen 一步到位)

  ★顺序很重要: 先在 RAM 里标定完(串口敲 cal, 等它转完), 再导出。
    板子一断电 RAM 里的表就没了 —— 固化进固件才不用每次重标。
"""
import argparse
import math
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
N = 2000
CSV_DEFAULT = ROOT / "tools" / "enc_lut.csv"
LUT_C = ROOT / "FOC" / "src" / "Encoder_Lut.c"
LINE_RE = re.compile(r"^\s*(\d{1,4})\s*,\s*(-?\d+(?:\.\d+)?)\s*$")


def capture_serial(port, baud=115200, timeout=60.0):
    import serial

    ser = serial.Serial(port, baud, timeout=0.2)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b"lut\r\n")
    ser.flush()

    rows = {}
    t0 = time.time()
    buf = b""
    while len(rows) < N and (time.time() - t0) < timeout:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if line.startswith("#"):
                print("   " + line)
                continue
            m = LINE_RE.match(line)
            if m:
                rows[int(m.group(1))] = float(m.group(2))
    ser.close()

    if len(rows) < N:
        print("★只收到 %d/%d 行 —— 上位机是不是没在收? 或者中途断了(lut end 没出现)" % (len(rows), N))
        return None
    return [rows[i] for i in range(N)]


def load_csv(path):
    rows = {}
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        m = LINE_RE.match(line)
        if m:
            rows[int(m.group(1))] = float(m.group(2))
    if len(rows) < N:
        print("★%s 里只有 %d 行有效数据(需要 %d 行)" % (path, len(rows), N))
        return None
    return [rows[i] for i in range(N)]


def report(lut):
    mean = sum(lut) / N
    mn = min(lut)
    mx = max(lut)
    rms = math.sqrt(sum((v - mean) ** 2 for v in lut) / N)
    print("表统计:")
    print("  直流(电角度对齐偏差) = %+.4f rad (%+.2f deg)  ← 表有效时这个值就是绝对对齐量"
          % (mean, math.degrees(mean)))
    print("  周期误差 峰峰 = %.4f rad (%.2f deg)   有效值 = %.4f rad (%.2f deg)"
          % (mx - mn, math.degrees(mx - mn), rms, math.degrees(rms)))
    print("  范围 = [%+.4f, %+.4f] rad" % (mn, mx))
    if (mx - mn) > 1.0:
        print("  ★峰峰超过 1rad(57deg): 表很可能是坏数据(转子打滑/没标定成), 别固化!")


def gen_c(lut):
    body = []
    for i in range(0, N, 4):
        body.append("    " + " ".join("%+.8ff," % lut[j] for j in range(i, min(i + 4, N))))
    text = (
        "/* 编码器电角度 2000 点校正表(固化) —— 由 tools/lut_capture.py 生成, 不要手改。\n"
        " * 表项 = 该角度格子上\"真实电角度 − 粗角度\"(rad)。全 0 = 不校正。\n"
        " */\n"
        "#include \"Encoder_Lut.h\"\n\n"
        "const float EncLut_Rom[ENCODER_LUT_SIZE] = {\n" + "\n".join(body) + "\n};\n"
    )
    LUT_C.write_text(text, encoding="utf-8")
    print("已写入 %s (%d 项)" % (LUT_C.relative_to(ROOT), N))
    print("重新编译烧录后永久生效: pwsh -File tools/build.ps1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="串口号, 例如 COM24")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--in", dest="infile", help="已有的导出文件(CSV: idx,rad)")
    ap.add_argument("--out", default=str(CSV_DEFAULT), help="CSV 存档路径")
    ap.add_argument("--gen", action="store_true", help="顺便生成 FOC/src/Encoder_Lut.c")
    args = ap.parse_args()

    if args.port:
        print("从 %s 收表(会自动发 lut 命令)..." % args.port)
        lut = capture_serial(args.port, args.baud)
    elif args.infile:
        lut = load_csv(args.infile)
    else:
        ap.error("要么 --port COMx, 要么 --in 文件")

    if lut is None:
        return 1

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="\n") as f:
        f.write("# idx,rad  (编码器电角度校正表, %d 点)\n" % N)
        for i, v in enumerate(lut):
            f.write("%d,%.8f\n" % (i, v))
    print("已存档 %s" % out)

    report(lut)
    if args.gen:
        gen_c(lut)
    else:
        print("要固化就加 --gen (会覆盖 FOC/src/Encoder_Lut.c)")
    return 0


if __name__ == "__main__":
    import sys as _s
    _s.exit(main())
