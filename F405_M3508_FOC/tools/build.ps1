# ============================================================================
# tools/build.ps1 —— 增量编译 + 链接(直接调 GCC, 不经过 ninja)
#
# 为什么不用 cmake --build: 本机上 cmake/ninja 会卡在启动阶段(实测十几分钟
# 无输出、CPU 接近 0), 而 arm-none-eabi-gcc 单独跑只要 0.5s。这里直接复刻
# build/Debug/build.ninja 里的编译/链接命令: 缺哪个 .obj 就从对应源文件补编,
# 最后按 ninja 的顺序重新链接。
#
# 用法:
#   pwsh -File tools/build.ps1                 # 补齐缺失/改动的目标文件并链接
#   pwsh -File tools/build.ps1 -Force          # 全部重编(慢, 只在前者出问题时用)
#   pwsh -File tools/build.ps1 -Files FOC/src/SMO.c   # 只编指定文件(不链接)
# ============================================================================
param(
    [string[]]$Files = @(),
    [switch]$Force,
    [switch]$NoLink
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $root 'build/Debug'
$ninjaPath = Join-Path $buildDir 'build.ninja'
$ninja = Get-Content $ninjaPath -Raw

# ---- 1. 从 build.ninja 抽出 "目标 .obj -> 源文件" 全表 -----------------------
$map = @{}
foreach ($m in [regex]::Matches($ninja, '(?m)^build ([^\s:]+\.obj): \S+ (\S+)')) {
    $obj = $m.Groups[1].Value.Replace('$', '').Replace('/', '\').Replace('__\__', '..\..')
    $src = $m.Groups[2].Value.Replace('$', '').Replace('/', '\')
    $map[[System.IO.Path]::GetFullPath((Join-Path $buildDir $obj))] = $src
}
if ($map.Count -eq 0) { Write-Error "build.ninja 里没解析到任何目标文件"; exit 1 }

# ---- 2. 编译参数(include 列表与 build.ninja 一致, 写全了省得再解析) ---------
$incs = @(
    "$root/BSP/inc", "$root/FOC/inc", "$root/Task/inc", "$root/Core/Inc",
    "$root/USB_DEVICE/App", "$root/USB_DEVICE/Target",
    "$root/Drivers/STM32F4xx_HAL_Driver/Inc",
    "$root/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
    "$root/Middlewares/Third_Party/FreeRTOS/Source/include",
    "$root/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
    "$root/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
    "$root/Middlewares/ST/STM32_USB_Device_Library/Core/Inc",
    "$root/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc",
    "$root/Drivers/CMSIS/Device/ST/STM32F4xx/Include",
    "$root/Drivers/CMSIS/Include",
    "$root/Middlewares/ST/ARM/DSP/Inc"
) | ForEach-Object { "-I$_" }
$cflags = @(
    '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
    '-Wall', '-fdata-sections', '-ffunction-sections',
    # ★-Wformat=2: printf 格式符与实参个数不匹配必须报出来。
    #   2026-10 踩过坑: 一处 CDC_Printf 少了 3 个参数, 编译不报, 运行时 vsnprintf
    #   从栈上读垃圾 -> HardFault -> 串口一行都不打印, 被误判成"板子坏了"。
    '-Wformat=2', '-Wformat-security',
    '-O0', '-g3', '-std=gnu11', '-O2',
    '-DDEBUG', '-DSTM32F405xx', '-DUSE_HAL_DRIVER'
)

function Invoke-Compile([string]$src, [string]$obj) {
    $dir = Split-Path -Parent $obj
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }
    if ($src -like '*.s') {
        & arm-none-eabi-gcc @cflags @incs -x assembler-with-cpp -c $src -o $obj
    } else {
        & arm-none-eabi-gcc @cflags @incs -c $src -o $obj
    }
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $src" }
}

# ---- 3. 决定要编哪些 ---------------------------------------------------------
$targets = @()
if ($Files.Count -gt 0) {
    foreach ($f in $Files) {
        $full = [System.IO.Path]::GetFullPath((Join-Path $root $f))
        $hit = $map.GetEnumerator() | Where-Object { $_.Value -eq $full }
        if (-not $hit) { Write-Warning "build.ninja 里没有 $f, 跳过"; continue }
        foreach ($h in $hit) { $targets += [pscustomobject]@{ Obj = $h.Key; Src = $h.Value } }
    }
} else {
    foreach ($kv in $map.GetEnumerator()) {
        $need = $Force -or (-not (Test-Path $kv.Key))
        if ($need) { $targets += [pscustomobject]@{ Obj = $kv.Key; Src = $kv.Value } }
    }
}

Write-Host "== 需要编译 $($targets.Count) / $($map.Count) 个文件"
foreach ($t in $targets) {
    Write-Host "   CC $(Split-Path -Leaf $t.Src)"
    Invoke-Compile $t.Src $t.Obj
}

if ($NoLink) { Write-Host "== -NoLink: 不链接"; exit 0 }

# ---- 4. 链接 -----------------------------------------------------------------
$objs = $map.Keys | Where-Object { Test-Path $_ }
$ldflags = @(
    '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
    "-T$root/STM32F405XX_FLASH.ld",
    '--specs=nano.specs', '-u', '_printf_float',
    '-Wl,-Map=F405_M3508_FOC.map', '-Wl,--gc-sections',
    '-Wl,--print-memory-usage'
)

Write-Host "== LD ($($objs.Count) objs)"
Push-Location $buildDir
& arm-none-eabi-gcc @ldflags @objs -o F405_M3508_FOC.elf -lm
$rc = $LASTEXITCODE
if ($rc -eq 0) {
    & arm-none-eabi-objcopy -O binary F405_M3508_FOC.elf F405_M3508_FOC.bin
    & arm-none-eabi-size F405_M3508_FOC.elf
}
Pop-Location
exit $rc
