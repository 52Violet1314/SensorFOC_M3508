#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/audit_printf.py —— 检查全工程 printf/CDC_Printf 的「格式串 vs 实参」个数

为什么需要这个:
    固件里 printf 参数个数不匹配 **不会编译报错**(默认没有 -Wformat 检查),
    但运行时会从栈上读垃圾 -> vsnprintf 崩溃 -> 主循环卡死 -> **串口一行都不打印**。
    2026-10 就是踩了这个坑: 一处重复的 CDC_Printf 少了 3 个参数,
    表现为"烧进去之后什么都不打印", 很容易误判成硬件坏了。

用法: python tools/audit_printf.py        (退出码非 0 表示发现不匹配)
"""
import glob
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_GLOBS = ["BSP/src/*.c", "FOC/src/*.c", "Core/Src/*.c", "USB_DEVICE/App/*.c"]

# 匹配一个完整的函数调用(含跨行), 支持嵌套括号
CALL_RE = re.compile(r"\b(?:CDC_Printf|printf)\s*\(")
# C 的格式转换说明(排除 %%)
FMT_RE = re.compile(r"%(?!%)[-+ #0-9.*hljztL]*[diouxXeEfFgGaAcspn]")


def split_top(src):
    """按顶层逗号切分参数(忽略括号/方括号内的逗号)"""
    out, depth, cur = [], 0, ""
    for ch in src:
        if ch in "([{":
            depth += 1
        if ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur)
            cur = ""
        else:
            cur += ch
    out.append(cur)
    return out


def merge_strings(args):
    """把开头的连续字符串字面量合并成完整格式串, 返回 (格式串, 剩余实参个数)"""
    fmt, k = "", 0
    while k < len(args):
        a = args[k].strip()
        if not re.fullmatch(r'"(?:[^"\\]|\\.)*"', a):
            break
        fmt += a[1:-1]
        k += 1
    return fmt, len(args) - k


def main():
    bad = 0
    checked = 0
    for pattern in SRC_GLOBS:
        for f in glob.glob(str(ROOT / pattern)):
            src = Path(f).read_text(encoding="utf-8", errors="replace")
            for m in CALL_RE.finditer(src):
                i = m.end()
                depth, j = 1, i
                while j < len(src) and depth > 0:
                    if src[j] == "(":
                        depth += 1
                    elif src[j] == ")":
                        depth -= 1
                    j += 1
                args = split_top(src[i:j - 1])
                if not args:
                    continue
                fmt, nrest = merge_strings(args)
                if fmt == "":
                    continue          # 格式串不是字面量, 跳过
                n = len(FMT_RE.findall(fmt))
                checked += 1
                # ★只报"实参比格式符少"——那才是会读栈垃圾、HardFault 的情况。
                #   实参偏多通常是解析误报(实参里含三元表达式/宏展开带逗号), 不是错误。
                if nrest < n:
                    line = src[:m.start()].count("\n") + 1
                    rel = Path(f).relative_to(ROOT)
                    print("  ★ %s:%d  格式符 %d 个, 实参只有 %d 个" % (rel, line, n, nrest))
                    print("      |%s|" % fmt[:70])
                    bad += 1
                elif nrest > n:
                    line = src[:m.start()].count("\n") + 1
                    rel = Path(f).relative_to(ROOT)
                    print("  (提示) %s:%d 实参 %d > 格式符 %d, 多为解析误报, 请人眼看一眼"
                          % (rel, line, nrest, n))

    print("\n检查了 %d 个 printf 调用, 发现 %d 处不匹配" % (checked, bad))
    if bad:
        print("★ 这些位置在运行时会从栈上读垃圾, 可能导致 HardFault / 串口完全不输出。")
        return 1
    print("OK: 格式串与实参个数全部匹配。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
