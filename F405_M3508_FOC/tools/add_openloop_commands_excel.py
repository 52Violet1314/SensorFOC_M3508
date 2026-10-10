#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/add_openloop_commands_excel.py —— 往 CAN_Control_Commands.xlsx 里补
"开环测试"命令页(0x20~0x30)与"USB 文本命令"页, 并更新协议说明。

原始 CAN 指令表由 tools/create_can_commands_excel.py 生成, 本脚本只做增量补充,
可重复执行(已存在的同名页会先删除再写)。
"""
from pathlib import Path

from openpyxl import load_workbook
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter

ROOT = Path(__file__).resolve().parents[1]
XLSX = ROOT / "CAN_Control_Commands.xlsx"

HDR_FILL = PatternFill("solid", fgColor="1F4E78")
HDR_FONT = Font(bold=True, color="FFFFFF")
NEW_FILL = PatternFill("solid", fgColor="C55A11")


def i16(value):
    raw = int(round(value)) & 0xFFFF
    return [(raw >> 8) & 0xFF, raw & 0xFF]


def frame(sub, raw):
    b1, b2 = i16(raw)
    data = [sub, b1, b2, 0, 0, 0, 0, 0]
    return ["0x91", 8] + ["%02X" % b for b in data] + ["0x91 8 " + " ".join("%02X" % b for b in data)]


# (子命令, 含义, 单位, 缩放, 示例值列表, 备注)
COMMANDS = [
    (0x20, "开环 V/F 开关", "-", 1, [1, 0],
     "非0=进入开环(驱动角用编码器电角度+转矩角); 0=退出并封驱动"),
    (0x21, "开环目标转速", "rpm(电机侧)", 1, [800, 1500, 2500],
     "输出轴转速 = 该值/19; 内部斜坡到目标(0x26 设斜率)"),
    (0x22, "V/F 电压/频率斜率", "V/Hz", 1000, [50, 60, 80],
     "原始值 = V/Hz×1000; 12V 母线建议 50~60"),
    (0x23, "V/F 电压上限", "V(峰值相电压)", 1000, [6500, 6000, 5500],
     "原始值 = V×1000; 12V 母线 SVPWM 线性区 ≈6.9V"),
    (0x24, "转矩角(磁场超前角)", "rad", 10000, [1.5708, -1.5708],
     "帧内原始值 = rad×10000(π/2=15708); 最大转矩/安培; 转向不对就发负值"),
    (0x25, "观测角对照偏置", "rad", 10000, [0.4613, 0.0, -0.4613],
     "帧内原始值 = rad×10000; 只影响打点/显示的 err, 不动驱动; 默认 +0.4613"),
    (0x26, "指令转速斜坡", "rpm/s", 1, [300, 600, 1200],
     "越大起得越快、电流冲击越大"),
    (0x27, "观测器滞后补偿带宽", "rad/s", 1, [400, 800, 2000],
     "θ_true-θ_est ≈ atan(|we|/(α·fs+w)); 默认 400, 按实测残差微调"),
    (0x28, "驱动角来源", "-", 1, [0, 1],
     "0=开环积分角(不需要编码器, 默认) 1=编码器电角度+转矩角"),
    (0x29, "起步对齐电压", "V", 1000, [1500, 0],
     "原始值 = V×1000; 使能后先在固定角停 300ms 把转子拉到位; 0=不对齐"),
    (0x2A, "低频强制启动电压", "V", 1000, [2800, 4000, 4500],
     "★解决起步堵转: fe<fres 或还没牵入时电压不低于此值; 原始值 = V×1000"),
    (0x2B, "低频强制段上界频率", "Hz", 100, [600, 900, 1200],
     "原始值 = Hz×100; 默认 600=6Hz; 起步无力就提到 8~10Hz"),
    (0x30, "SMO 打点流开关", "-", 1, [1, 0],
     "1=开始 1kHz USB CSV 打点; 0=停止"),
]

USB_CMDS = [
    ("vf <rpm>", "进入开环并把目标转速设为 rpm(电机侧), 例: vf 1500"),
    ("vf", "只进入开环, 不改目标转速"),
    ("stop", "退出开环, 三相占空比封零"),
    ("cur <A>", "回到闭环电流模式, iq = A"),
    ("spd <rpm>", "回到闭环速度模式"),
    ("slope <V/Hz>", "改 V/F 斜率, 例: slope 0.055"),
    ("vboost <V>", "改低速补偿电压, 例: vboost 1.2"),
    ("vmax <V>", "改电压上限, 例: vmax 6.5"),
    ("ramp <rpm/s>", "改指令转速斜坡"),
    ("fstart <Hz>", "改开环积分角的起始电频率(默认 0.5Hz)"),
    ("valign <V>", "改起步对齐电压(默认 1.5V, 0=不对齐)"),
    ("vlo <V>", "★改低频强制启动电压(默认 2.8V): 起步堵转就加到 3.5~4.5"),
    ("fres <Hz>", "改低频强制段上界频率(默认 6Hz): 起步无力提到 8~10"),
    ("drv <0|1>", "驱动角来源: 0=开环积分角(无编码器, 默认) 1=编码器角"),
    ("angle <rad>", "改编码器模式下的转矩角, 例: angle 1.5708"),
    ("off <rad>", "改观测角对照偏置(只影响打点显示), 例: off 0.4613"),
    ("lag <rad/s>", "改滞后补偿等效带宽(默认 400), 用来压随转速漂的残差"),
    ("gain <V>", "改滑模增益 k(默认 50V)"),
    ("delta <A>", "改边界层 δ(默认 8A, 改小要重新核算稳定性)"),
    ("stream <0|1>", "打点流开关"),
    ("stat", "打印状态(含 TIM1 寄存器自检、观测器、V/F 参数、方向自检 smo_dir)"),
    ("help", "打印命令列表"),
]


def main():
    wb = load_workbook(XLSX)

    # ---- 1. 协议说明补充几行 ------------------------------------------------
    intro = wb.worksheets[0]
    extra = [
        ("开环测试", "数据字节0=20 进开环; 21 目标转速; 22 斜率(×1000); 23 电压上限(×1000)"),
        ("开环转矩角/偏置", "24 转矩角 mrad; 25 观测角对照偏置 mrad"),
        ("打点流", "数据字节0=30, 值非0=开始 1kHz CSV 打点(走 USB 虚拟串口)"),
        ("USB 文本命令", "同一根 USB 线上可直接敲 vf/stop/stream 等命令, 见 'USB文本命令' 页"),
    ]
    have = {str(r[0]) for r in intro.iter_rows(values_only=True)}
    for k, v in extra:
        if k not in have:
            intro.append([k, v])

    # ---- 2. 开环测试命令页 --------------------------------------------------
    title = "开环测试指令"
    if title in wb.sheetnames:
        del wb[title]
    ws = wb.create_sheet(title)
    ws.append(["组号", "子命令", "含义", "单位", "示例值", "协议原始值",
               "CAN ID", "DLC", "数据字节0", "数据字节1", "数据字节2",
               "数据字节3", "数据字节4", "数据字节5", "数据字节6", "数据字节7",
               "完整指令（单元格复制）", "备注"])
    for c in ws[1]:
        c.font = HDR_FONT
        c.fill = HDR_FILL
        c.alignment = Alignment(horizontal="center")

    n = 0
    for sub, name, unit, scale, values, note in COMMANDS:
        for v in values:
            n += 1
            raw = int(round(v * scale))
            ws.append([n, "0x%02X" % sub, name, unit, v, raw] + frame(sub, raw) + [note])

    ws.freeze_panes = "A2"
    ws.auto_filter.ref = ws.dimensions
    widths = {1: 6, 2: 8, 3: 22, 4: 16, 5: 12, 6: 12, 7: 8, 8: 6,
              17: 34, 18: 52}
    for col in range(1, ws.max_column + 1):
        ws.column_dimensions[get_column_letter(col)].width = widths.get(col, 9)

    # ---- 3. USB 文本命令页 --------------------------------------------------
    title2 = "USB文本命令"
    if title2 in wb.sheetnames:
        del wb[title2]
    ws2 = wb.create_sheet(title2)
    ws2.append(["命令", "说明"])
    for c in ws2[1]:
        c.font = HDR_FONT
        c.fill = HDR_FILL
    for cmd, desc in USB_CMDS:
        ws2.append([cmd, desc])
    ws2.column_dimensions["A"].width = 18
    ws2.column_dimensions["B"].width = 70

    wb.save(XLSX)
    print("updated:", XLSX)
    print("sheets:", wb.sheetnames)


if __name__ == "__main__":
    main()
