# ============================================================================
# tools/swd_test.ps1 —— 一键跑 SWD 连通性测试, 直接打印 OpenOCD 原始输出
#
#   pwsh -File tools/swd_test.ps1              # DAPLink/MCU-Link, 1MHz
#   pwsh -File tools/swd_test.ps1 -Speed 100   # 降到 100kHz
#   pwsh -File tools/swd_test.ps1 -Interface jlink
#
# 判读(看输出里有没有这两行):
#   连上了 ->  Info : SWD DPIDR 0x2ba01477
#              Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
#   没连上 ->  Error: Error connecting DP: cannot read IDR
# ============================================================================
param(
    [int]$Speed = 1000,
    [string]$Interface = 'cmsis-dap',
    [string]$OpenOcd = 'C:\msys64\mingw64\bin\openocd.exe',
    [int]$Seconds = 6
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$cfg = Join-Path $root 'tools\openocd_swd_test.cfg'
$out = Join-Path $env:TEMP 'swd_test_out.txt'
$err = Join-Path $env:TEMP 'swd_test_err.txt'

Write-Host "openocd : $OpenOcd"
Write-Host "接口    : $Interface   速度: ${Speed}kHz"
Write-Host "配置    : $cfg"
Write-Host ("-" * 70)

# ★顺序很重要: 必须先把 -f interface 和 -c "adapter speed" 传进去, 最后才 -f 目标配置
#   (目标配置里有 init/halt; 顺序反了 OpenOCD 会报 "does not support any transports")
$args = "-f interface/$Interface.cfg -c `"adapter speed $Speed`" -s `"C:\msys64\mingw64\share\openocd\scripts`" -f `"$cfg`""
$p = Start-Process -FilePath $OpenOcd -ArgumentList $args -PassThru -NoNewWindow `
        -RedirectStandardOutput $out -RedirectStandardError $err
Start-Sleep -Seconds $Seconds

if (-not $p.HasExited) {
    Write-Host "==> OpenOCD 仍在运行: SWD 已连上, GDB server 正在监听 50000" -ForegroundColor Green
    Get-NetTCPConnection -LocalPort 50000 -State Listen -ErrorAction SilentlyContinue |
        Select-Object LocalAddress, LocalPort, State | Format-Table -AutoSize | Out-String | Write-Host
    Stop-Process -Id $p.Id -Force
} else {
    Write-Host "==> OpenOCD 已退出 (exit=$($p.ExitCode)): SWD 没连上" -ForegroundColor Yellow
}

Write-Host "`n--- OpenOCD 输出 ---"
Get-Content $out -ErrorAction SilentlyContinue | Write-Host
Get-Content $err -ErrorAction SilentlyContinue | Write-Host

Write-Host ("-" * 70)
$all = ((Get-Content $out -ErrorAction SilentlyContinue) + (Get-Content $err -ErrorAction SilentlyContinue)) -join "`n"
if ($all -match 'SWD DPIDR') {
    Write-Host "结论: 探针 <-> 芯片 通了(读到了 DPIDR)。调试/GDB 问题在 VS Code 配置侧。" -ForegroundColor Green
} elseif ($all -match 'cannot read IDR') {
    Write-Host "结论: 探针本身 OK(CMSIS-DAP 已握手), 但芯片不应答 SWD。按顺序查:" -ForegroundColor Yellow
    Write-Host "  1) 目标板供电了吗? DAPLink 的 3V3 通常带不动整块板 -> 用板子自己的电源"
    Write-Host "  2) 共地了吗? 探针 GND 必须和板子 GND 接在一起"
    Write-Host "  3) SWDIO/SWCLK 有没有接反/虚接? 一般还要接 NRST 和 3V3(参考电平)"
    Write-Host "  4) 芯片是否被读保护(RDP)锁了? 用 STM32CubeProgrammer 看能不能解锁"
    Write-Host "  5) 换根线/换个探针(手头那个 J-Link)交叉验证: 换探针好了就是线/探针, 还不行就是板子"
} elseif ($all -match 'No device found|no device') {
    Write-Host "结论: 探针没被 OpenOCD 找到 —— 换 -Interface, 或装探针驱动。" -ForegroundColor Yellow
} else {
    Write-Host "结论: 看上面的输出自行判断。" -ForegroundColor Yellow
}
