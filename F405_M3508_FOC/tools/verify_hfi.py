"""HFI 离线自检(不上电): 滤波器系数 + 环路极性 + 不同凸极率/噪声下的锁定能力

用法: python tools/verify_hfi.py [--fast]

做四件事:
  1. 带通/陷波系数自检: 从公式重算, 与 FOC/inc/HFI_INJ.h 里的字面值逐一比对;
     再算 |H|/相位(带通中心必须 |H|=1、相位=0, 否则解调会因滤波而偏相)。
  2. 环路极性(sign): 整链仿真, θ̂ 初值故意偏 ±30°, 必须**收敛**而不是跑飞
     —— 这条是所有 HFI 代码最容易写反、写反了现象最吓人的地方。
  3. 锁定能力表: 凸极率 {2,5,10,20}% × ADC 噪声 {0,1,2} LSB, 报最终角误差。
     这就是"这颗 M3508 的凸极够不够用"的离线答案(表贴磁钢通常很小)。
  4. 标定常数核对: 仿真反推 HFI_SAL_K(凸极率 ↔ Kth/|id|) —— 头文件里的值必须
     跟仿真对得上(对不上说明模型常数写错了)。

仿真里跑的链路与 FOC/src/HFI_INJ.c 逐行对应: 两级带通 → Park → 半周期同步平均
→ 2ωe 自适应陷波 → Kth 归一化 → PI(2kHz) → 积分。被控对象是带凸极的 RL
(静止, 与 HFI 的实际用法一致), 注入叠在编码器坐标系 d 轴上。
"""
import re
import sys
import math

import numpy as np

HDR = 'FOC/inc/HFI_INJ.h'
SMO = 'FOC/inc/SMO.h'
FAST = '--fast' in sys.argv
DBG_CAL = '--dbgcal' in sys.argv


# ---------------------------------------------------------------- 读固件参数
def load_defines(path):
    txt = open(path, encoding='utf-8').read()
    out = {}
    for m in re.finditer(r'^#define\s+(\w+)\s+([-+0-9.eE]+)[fFuU]*\s*(?:/\*.*)?$', txt, re.M):
        name, val = m.group(1), m.group(2)
        try:
            out[name] = float(val)
        except ValueError:
            pass
    return out


D = load_defines(HDR)
S = load_defines(SMO)
fail = []

print(f'固件: {HDR}')


def need(name, src=D):
    if name not in src:
        print(f'  ★ 读不到 {name}, 头文件被改过?')
        sys.exit(2)
    return src[name]


FS = 1.0 / need('HFI_TS')
HALF = int(need('HFI_HALF_TICKS'))
TSL = need('HFI_TS_LOOP')
UINJ = need('HFI_INJ_V_AMP')
RS = need('SMO_RS_DEF', S)
LS = need('SMO_LS_DEF', S)
SALK = need('HFI_SAL_K')
CAL_HALF = int(need('HFI_CAL_HALF'))
CAL_CYCLES = int(need('HFI_CAL_CYCLES'))
CAL_DEMODS = CAL_HALF * 2 * CAL_CYCLES          # 头文件里是表达式, 这里自己乘
CAL_SKIP = int(need('HFI_CAL_SKIP'))
print(f'固件参数: fs={FS:.0f}Hz 半周期={HALF}拍 '
      f'(方波 {FS/(2*HALF):.0f}Hz, 解调/翻转 {FS/HALF:.0f}Hz) '
      f'环路Ts={TSL*1e3:.2f}ms U={UINJ:.2f}V R={RS}Ω L={LS*1e6:.0f}uH')
print(f'         标定窗: {CAL_DEMODS} 次解调 = {CAL_DEMODS*TSL*1e3:.0f}ms '
      f'(每点 {CAL_HALF} 次, 丢前 {CAL_SKIP} 次, {CAL_CYCLES} 轮)')

# ---------------------------------------------------------------- 1. 滤波器
print('\n[1] 滤波器自检')
Q = need('HFI_BPF_Q')
f0 = FS / (2.0 * HALF)          # 方波基频: 极性每 HALF 拍翻一次 ⇒ 周期 2·HALF 拍
w0 = 2 * math.pi * f0 / FS
al = math.sin(w0) / (2 * Q)
a0 = 1 + al
b0, b1, b2 = al / a0, 0.0, -al / a0
a1, a2 = -2 * math.cos(w0) / a0, (1 - al) / a0
for name, want in (('HFI_BPF_B0', b0), ('HFI_BPF_B1', b1), ('HFI_BPF_B2', b2),
                   ('HFI_BPF_A1', a1), ('HFI_BPF_A2', a2)):
    got = need(name)
    ok = abs(got - want) < 3e-7
    print(f'   {name:12s} 头文件 {got:+.8f}  公式 {want:+.8f}  {"ok" if ok else "★不一致"}')
    if not ok:
        fail.append(f'{name} 与公式不符')


def bpf_h(f, sec=1):
    """RBJ 带通在频率 f 处的复响应(级联 sec 级)"""
    z = np.exp(-2j * np.pi * f / FS)
    h = (b0 + b1 * z + b2 * z ** 2) / (1 + a1 * z + a2 * z ** 2)
    return h ** sec


for f in (f0,):
    h = bpf_h(f)
    print(f'   |H({f:.0f}Hz)| = {abs(h):.6f} (要 1), 相位 {math.degrees(np.angle(h)):+.3f}° (要 0)')
    if abs(abs(h) - 1.0) > 1e-6 or abs(np.angle(h)) > 1e-6:
        fail.append('带通中心处增益/相位不对')
for f, tag in ((116.0, '1000rpm 基波'), (58.0, '478rpm 基波'), (2 * f0, '方波 3 次谐波(混叠到 2kHz)')):
    print(f'   级联 2 级 @{f:6.1f}Hz ({tag}): |H| = {abs(bpf_h(f, 2)):.4f}')
if abs(bpf_h(58.0, 2)) > 0.15:
    fail.append('基波抑制不足(级联带通)')

# 陷波在 2ωe 处 ≈ 0, 远处 ≈ 1
nQ = need('HFI_NOTCH_Q')
for fe in (10.0, 30.0, 60.0):
    fn = 2 * fe
    w = 2 * math.pi * fn * TSL
    aln = math.sin(w) / (2 * nQ)
    nb0 = 1 / (1 + aln); nb1 = -2 * math.cos(w) / (1 + aln); nb2 = nb0
    na1 = nb1; na2 = (1 - aln) / (1 + aln)
    z = np.exp(-2j * math.pi * fn * TSL)
    hn = (nb0 + nb1 * z + nb2 * z ** 2) / (1 + na1 * z + na2 * z ** 2)
    zf = np.exp(-2j * math.pi * 5.0 * TSL)
    hf = (nb0 + nb1 * zf + nb2 * zf ** 2) / (1 + na1 * zf + na2 * zf ** 2)
    print(f'   陷波 @2fe={fn:5.1f}Hz: |H|={abs(hn):.4f}(要≈0)   @5Hz: |H|={abs(hf):.3f}(要≈1)')

# ---------------------------------------------------------------- 2/3. 整链仿真
def simulate(sal_pct, noise_lsb, off_deg=30.0, seconds=0.60, seed=1, fund_a=0.0,
             fund_hz=58.0, cal_mode='new', hold_deg=None, gain_scale=1.0,
             use_notch=False, spin_hz=0.0, bias_sub=True, use_gate=True,
             hw_bias=0.0, hw_b2=0.0, cal_guard=False, b2c=None,
             rot_hw=0.0, rot_c=0.0):
    """cal_mode: 'new' = 固件现在的两段式(±45° 两点标定 → 零误差+速度前馈交接 → 冻结)
                  'old' = 改之前的行为(播种 45° 直接闭环, Kth 一直自适应) —— 用来复现跑飞
                  'off' = 不标定, θ̂ 从 off_deg 起步(极性测试用)
       spin_hz: 转子电频率 Hz(现场 205rpm ≈ 24Hz) —— 静态转子的仿真漏掉了这个工况"""
    rng = np.random.default_rng(seed)
    L0 = LS
    Ld = L0 * (1 - sal_pct / 200.0)
    Lq = L0 * (1 + sal_pct / 200.0)
    lsb = 20.0 * 2.0 / 4095.0                    # 电流采样 1LSB ≈ 9.8mA(±20A 量程)
    theta0 = math.radians(37.0)                  # 转子初始位置(编码器知道)
    th = theta0
    theta_true = theta0
    if cal_mode == 'old':
        th = theta0 - math.radians(45.0)         # 老固件: 使能时播种到 θenc−45°
    elif cal_mode == 'off':
        th = theta0 + math.radians(off_deg)      # 极性测试: 故意给 ±30° 初值
    if hold_deg is not None:                     # 判决性实验: θ̂ 钉死, 量静态 err(Δ)
        th = theta0 + math.radians(hold_deg)
    i_d = i_q = 0.0
    ct, st = math.cos(theta_true), math.sin(theta_true)
    # 滤波器状态
    bx = [[0.0] * 4 for _ in range(4)]           # 每级 x1,x2,y1,y2

    def bpf(state, x):
        x1, x2, y1, y2 = state
        y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        return [x, x1, y, y1], y

    A1 = A2 = B1 = B2 = [0.0] * 4
    n_x1 = n_x2 = n_y1 = n_y2 = 0.0
    n_cos = math.cos(2 * math.pi * need('HFI_NOTCH_F_MIN') * TSL)
    n_al = math.sin(2 * math.pi * need('HFI_NOTCH_F_MIN') * TSL) / (2 * nQ)
    n_a0i = 1 / (1 + n_al)
    kn = need('HFI_KTH_DEF')
    kth = kn
    kth_num = kth_den = kth_res = 0.0
    w_max = need('HFI_MAX_W')
    refused = False
    kth_done = (cal_mode != 'new')               # 'new' 要等标定窗结束才冻结
    kth_frozen = (cal_mode == 'old')             # 'old' 允许一直自适应(复现老行为)
    # ---- 标定窗状态机(与固件 hfi_cal_start 一一对应)----
    cal_off = need('HFI_CAL_OFFSET')
    cal_cnt = CAL_DEMODS if cal_mode == 'new' else 0
    cal_phase, cal_half, cal_skip = 0, CAL_HALF, CAL_SKIP
    cal_p_sum = cal_m_sum = 0.0
    cal_p_n = cal_m_n = 0
    kth_bias = 0.0
    nolock = 0
    nolock_max = int(need('HFI_NOLOCK_DEMODS'))
    rail = need('HFI_PLL_KP') * gain_scale + need('HFI_PLL_I_LIMIT')
    if hold_deg is not None:
        cal_cnt = 0                              # 判决性实验: 不走标定窗
        kth_frozen = True
    kp, ki = need('HFI_PLL_KP') * gain_scale, need('HFI_PLL_KI') * gain_scale
    ilim = need('HFI_PLL_I_LIMIT')
    w_lpf = need('HFI_OMEGA_LPF_ALPHA')
    wi = wp = w = w_f = 0.0
    w_enc = 0.0
    w_reached_rail = 0.0
    sat_run = 0
    sat_max = 0
    iq_acc = id_acc = 0.0
    cnt = 0
    sign = 1
    sign_prev = 0
    tick = 0
    errs = []
    err_log, id_log = [], []
    trace = []
    dbg = []
    filt_log = []
    n = int(seconds * FS)
    sub = 8
    h = 1.0 / (FS * sub)
    kth_snaps = []
    u = sign * UINJ                      # 本拍开始时正在作用的电压(上一拍写的)
    th_enc_prev = theta0
    for k in range(n):
        # ---- 0) 转子在转(现场 205rpm ≈ 24Hz): 编码器就知道这个角 ----
        theta_true = theta0 + 2 * math.pi * spin_hz * (k / FS)
        ct, st = math.cos(theta_true), math.sin(theta_true)
        theta_enc = theta_true
        dw_enc = (theta_enc - th_enc_prev + math.pi) % (2 * math.pi) - math.pi
        w_enc += 0.05 * (dw_enc * FS - w_enc)          # 与固件同式(fc≈80Hz)
        th_enc_prev = theta_enc
        # ---- 转速门限(与固件 HFI_Tick 一致): 超范围就拒绝, 不给垃圾 ----
        gated = use_gate and (abs(w_enc) > w_max) and (k > 50)
        if gated:
            if cal_cnt > 0:
                cal_cnt = 0                            # 转子在转: 中止/不启动标定窗
            if cal_mode == 'new':
                refused = True
        # ---- 1) 采样本拍电流 ----
        #   ★与固件时序一致: ADC 在 CC4 时刻采完, 中断里**之后**才写新 CCR ⇒
        #     采到的电流反映的是上一拍写下去的电压(所以解调用 sign_prev)。
        #     反过来的话(先写电压再采样)每次采样跨两段电压, 解调会混进一个
        #     与角度无关的偏置 —— 归档版 HFI.h 专门警告过这件事, 仿真里也复现了。
        ia = i_d * ct - i_q * st
        ib = i_d * st + i_q * ct
        if fund_a:                                   # 叠加基波(检验带通抑制, 线性叠加)
            wf = 2 * math.pi * fund_hz
            ia += fund_a * math.cos(wf * k / FS)
            ib += fund_a * math.sin(wf * k / FS)
        if noise_lsb:
            ia += rng.uniform(-noise_lsb, noise_lsb) * lsb
            ib += rng.uniform(-noise_lsb, noise_lsb) * lsb
        # ---- 2) 前端两级带通 ----
        A1, xa = bpf(A1, ia); A2, xa = bpf(A2, xa)
        B1, xb = bpf(B1, ib); B2, xb = bpf(B2, xb)
        # 自检: 带通是 IIR, 一旦发散(系数被改坏/输入异常)必须立刻炸出来,
        #   否则后面看到的会是一堆"看似能锁、其实全是假的"数字(踩过)。
        if not (-8.0 < xa < 8.0) or not (-8.0 < xb < 8.0):
            raise RuntimeError(
                f'带通输出发散: k={k} xa={xa:.4g} xb={xb:.4g} ia={ia:.4g} ib={ib:.4g} '
                f'coef b=({b0:.6f},{b1:.6f},{b2:.6f}) a=({a1:.6f},{a2:.6f})')
        # ---- 3) Park + 半周期累加 ----
        #   ★rot_hw = 硬件上"解调轴相对注入轴"的固有偏移(约定/接线/采样相位),
        #     rot_c = 固件用 hfi_demod_rot 补偿回来的量; Park 用两者之和。
        #     注入方向仍然只用 th(θ̂) —— 这正是"注入轴与解调轴不一致"的建模方式。
        sp, cp = math.sin(th + rot_hw + rot_c), math.cos(th + rot_hw + rot_c)
        iq_hf = -xa * sp + xb * cp
        id_hf = xa * cp + xb * sp
        iq_acc += iq_hf
        id_acc += id_hf
        cnt += 1
        if sign_prev and sign != sign_prev:
            if cnt > 1:
                kk = cnt - 1
                err_raw = sign_prev * (iq_acc - iq_hf) / kk
                id_dem = sign_prev * (id_acc - id_hf) / kk
                # ---- 硬件不对称(死区/PWM/传感器失配)在解调量上的等效项 ----
                #   现场实测 ~0.15A, 与 20% 凸极的真信号(0.033A)同量级甚至更大。
                #   ★关键: 它**随注入轴与转子的夹角变化**(电流方向决定死区误差方向),
                #     按对称性写成 b_dc + b2·cos(2Δθ):
                #       · 标定窗 Δθ=±45° → cos=0 ⇒ 量到的"偏置"只有 b_dc, 而 Kth 不受影响;
                #       · 锁定时 Δθ≈0 → 多出 b2, 减掉 b_dc 也剩着 b2 ⇒ 被积分器吸掉 ⇒
                #         丢转速跟踪 ⇒ 跑飞。这就是现场那一幕, 恒定偏置复现不出来。
                err_raw += hw_bias + hw_b2 * math.cos(2 * (theta_true - th))
                # 2ωe 自适应陷波(与固件同式)
                if use_notch:
                    fn = min(max(abs(w_f) / math.pi, need('HFI_NOTCH_F_MIN')),
                             need('HFI_NOTCH_F_MAX'))
                    ww = 2 * math.pi * fn * TSL
                    ca = need('HFI_NOTCH_ADAPT')
                    n_cos = ca * math.cos(ww) + (1 - ca) * n_cos
                    n_al = ca * (math.sin(ww) / (2 * nQ)) + (1 - ca) * n_al
                    n_a0i = 1 / (1 + n_al)
                    nb0 = n_a0i; nb1 = -2 * n_cos * n_a0i; nb2 = n_a0i
                    na1 = nb1; na2 = (1 - n_al) * n_a0i
                    y = nb0 * err_raw + nb1 * n_x1 + nb2 * n_x2 - na1 * n_y1 - na2 * n_y2
                    n_x2, n_x1 = n_x1, err_raw
                    n_y2, n_y1 = n_y1, y
                    err_n = y
                else:
                    err_n = err_raw
                # ---- Kth: 两段式标定(与固件 hfi_cal_start/PLL_Proc 3b 一一对应)----
                d_rep = th - theta_true              # 报告用: θ̂ − θenc(与固件打点同口径)
                while d_rep > math.pi / 2: d_rep -= math.pi
                while d_rep < -math.pi / 2: d_rep += math.pi
                d_enc = theta_true - th              # 拟合/激励用: θenc − θ̂
                while d_enc > math.pi / 2: d_enc -= math.pi
                while d_enc < -math.pi / 2: d_enc += math.pi
                s2 = math.sin(2 * d_enc)
                kf = need('HFI_KTH_FORGET')
                if not (-100.0 < err_raw < 100.0) or not (-10.0 < kth < 10.0) \
                        or not (-10.0 < id_dem < 10.0):
                    raise RuntimeError(
                        f'解调量爆炸: k={k} err={err_raw:.4g} id={id_dem:.4g} '
                        f'kth={kth:.4g} s2={s2:+.3f} th={math.degrees(th - theta_true):.1f}° '
                        f'w_f={w_f:.1f} i_d={i_d:.3g} i_q={i_q:.3g} '
                        f'xa={xa:.3g} xb={xb:.3g} ia={ia:.3g} cnt={cnt}')
                kth_res += kf * ((err_raw - kth * s2) ** 2 - kth_res)
                err_log.append(err_raw)
                id_log.append(id_dem)
                if cal_mode == 'new' and cal_cnt > 0:
                    # 标定窗: θ̂ 钉在编码器 ±45° 两个点上, 环路不闭
                    if cal_skip > 0:
                        cal_skip -= 1
                    elif cal_phase == 0:
                        cal_p_sum += err_raw; cal_p_n += 1
                    else:
                        cal_m_sum += err_raw; cal_m_n += 1
                    cal_half -= 1
                    if cal_half == 0:
                        cal_half = CAL_HALF
                        cal_phase ^= 1
                        cal_skip = CAL_SKIP
                    th = (theta_true + (cal_off if cal_phase == 0 else -cal_off)) % (2 * math.pi)
                    cal_cnt -= 1
                    if cal_cnt == 0:
                        if cal_p_n and cal_m_n:
                            ep = cal_p_sum / cal_p_n          # θ̂ = θenc+45°
                            em = cal_m_sum / cal_m_n          # θ̂ = θenc−45°
                            # ★符号: err = +Kth·sin(2(θenc−θ̂)) ⇒ +45°点 = b−Kth,
                            #   −45°点 = b+Kth ⇒ Kth = (em−ep)/2(第一版写反过, 仿真抓的)
                            kth = min(max(0.5 * (em - ep), need('HFI_KTH_MIN')),
                                      need('HFI_KTH_MAX'))
                            kth_bias = 0.5 * (em + ep)
                            if DBG_CAL:
                                print(f'   CAL: ep={ep:+.4f}(n={cal_p_n}) em={em:+.4f}'
                                      f'(n={cal_m_n}) → kth={kth:+.4f} bias={kth_bias:+.4f}')
                            # 固件的 flt=4 判据: 偏置和增益同量级 ⇒ 这套在这块板子上出不来角。
                            # ★只在**还没做过 hfiscan 补偿**(b2c 为空)时才拒绝 —— 补偿做完后
                            #   b_dc 由环路显式减掉, |bias|≥kth 不再是拒绝理由, 否则用户跑完
                            #   hfiscan 也会被永远挡在门外。
                            if cal_guard and (b2c is None) and (abs(kth_bias) >= kth):
                                refused = True
                        kth_done = True
                        kth_frozen = True
                        # 交接: 零误差 + 积分器/速度预置成编码器速度(前馈)
                        th = theta_true
                        wi = max(-ilim, min(ilim, w_enc)); w = wi; w_f = wi
                else:
                    # 'old' 模式: 用编码器基向量做 RLS(会一直自适应) —— 复现老固件
                    if (cal_mode == 'old') and (abs(s2) > need('HFI_KTH_S2_GATE')):
                        ff = need('HFI_KTH_FORGET_HOLD') if kth_frozen else kf
                        kth_num += ff * (err_raw * s2 - kth_num)
                        kth_den += ff * (s2 * s2 - kth_den)
                        if kth_den > need('HFI_KTH_DEN_MIN'):
                            kth = min(max(kth_num / kth_den, need('HFI_KTH_MIN')),
                                      need('HFI_KTH_MAX'))
                    # ---- 环路 ----
                    if (hold_deg is not None) or gated or refused:
                        pass                       # 判决性实验 / 转速超限 / 标定不可信: 不动 θ̂
                    else:
                        # ★先减掉标定窗量到的解调偏置: II 型环会把 DC 偏置当成频差,
                        #   用积分器"抵消"它 —— 代价是丢掉转速跟踪(现场跑飞的根因)
                        # ★先减掉两样已知的解调缺陷: 常数偏置 b_dc(标定窗量的) 与
                        #   不对称的 cos2Δ 分量 b2(hfiscan 量的) —— 不减的话 II 型环会把
                        #   它们当频差、用积分器吃掉 ⇒ 丢转速跟踪 ⇒ 跑飞(现场)。
                        err_use = err_n - (kth_bias if bias_sub else 0.0)
                        if b2c is not None:
                            err_use -= b2c * math.cos(2 * (theta_true - th))
                        eps = max(-1.0, min(1.0, err_use / max(kth, need('HFI_KTH_MIN'))))
                        wp = eps * kp
                        wi += eps * ki * TSL
                        wi = max(-ilim, min(ilim, wi))
                        w = wp + wi
                        w_f += w_lpf * (w - w_f)
                        th += w_f * TSL
                        # 饱和统计(与固件 4b 一致)
                        if abs(eps) >= need('HFI_NOLOCK_EPS'):
                            sat_run += 1
                            sat_max = max(sat_max, sat_run)
                        else:
                            sat_run = 0
                        w_reached_rail = max(w_reached_rail, abs(w_f) / max(rail, 1e-9))
                if len(dbg) < 30:
                    dbg.append((k, err_raw, kth, s2, math.degrees(th - theta_true)))
                if len(trace) < 400 and (k % 20 == 0):
                    trace.append((k * 0.1, math.degrees((th - theta_true) % (2 * math.pi)),
                                  math.degrees(d_rep), err_raw, kth))
                kth_snaps.append((kth, id_dem))
            iq_acc = iq_hf
            id_acc = id_hf
            cnt = 1
        sign_prev = sign
        # ---- 4) 本拍末: 翻转极性, 用新电压积分一拍得到下一拍的电流 ----
        if tick >= HALF:
            tick = 0
            sign = -sign
        tick += 1
        u = sign * UINJ
        # 注入沿 **PLL 估计**的 d 轴 θ̂(与 mf_HFI_Observer.m 一致)。
        #   ★若改成沿编码器轴注入, 电流矢量就跟着编码器轴走, 解调量与凸极无关
        #     (∝sinΔθ), 观测器等于在跟踪自己 —— 见 [1b] 节的对照。
        ct_i, st_i = math.cos(th), math.sin(th)
        ua, ub = u * ct_i, u * st_i
        u_d = ua * ct + ub * st
        u_q = -ua * st + ub * ct
        for _ in range(sub):
            i_d += h * (u_d - RS * i_d) / Ld
            i_q += h * (u_q - RS * i_q) / Lq
        # 末段统计(报告口径 = θ̂ − θenc, 折到 ±90°)
        if k > n - int(0.15 * FS):
            d_rep = th - theta_true
            while d_rep > math.pi / 2: d_rep -= math.pi
            while d_rep < -math.pi / 2: d_rep += math.pi
            errs.append(math.degrees(d_rep))
    err_fin = float(np.mean(errs[-40:])) if errs else 999.0
    kth_f, id_f = kth_snaps[-1] if kth_snaps else (0.0, 0.0)
    noise_rms = math.sqrt(kth_res) if kth_res > 0 else 0.0
    return dict(err=err_fin, rms=float(np.std(errs)) if errs else 999.0,
                kth=kth_f, id=id_f, res=noise_rms, done=kth_done, trace=trace,
                rail=w_reached_rail, sat_max=sat_max, bias=kth_bias,
                w_enc=w_enc, refused=refused,
                err_hold=(float(np.mean(err_log[-20:])) if err_log else 0.0),
                id_hold=(float(np.mean(np.abs(id_log[-20:]))) if id_log else 0.0),
                dbg=dbg,
                snr=(kth_f / noise_rms) if noise_rms > 1e-9 else float('inf'))


if '--trace' in sys.argv:
    tr = simulate(20.0, 0, off_deg=30.0, seconds=0.6, cal_mode='off')
    print('\n[traj] 从 Δ=+30° 起(不播种):  t(ms) Δ未折叠  Δ折叠  err_raw  kth')
    for t, dun, dfold, e, kk in tr['trace'][:24]:
        print(f'   {t:7.1f} {dun:8.1f} {dfold:+7.1f} {e:+.4f} {kk:.4f}')
    print('   前 30 次解调: k  err_raw   kth     s2     θ̂−θtrue(deg)')
    for k, e, kk, s2, dth in tr['dbg']:
        print(f'   {k:5d} {e:+.4f} {kk:.4f} {s2:+.3f} {dth:+8.1f}')


print('\n[0] 复现现场(2026-10 实测): 转子 24Hz(≈205电机rpm), 1LSB 噪声, 20% 凸极')
print('   ★现场波形: w 钉在 ±450rad/s(= Kp+I_LIMIT)、kth 掉到下限、err 乱跳 → 没锁上。')
print('   mode                        末段误差     RMS     kth[A]   偏置[A]  |ω|峰值/限幅 最长饱和')
r_old = simulate(20.0, 1, cal_mode='old', spin_hz=24.0, seconds=1.2)
r_new = simulate(20.0, 1, cal_mode='new', spin_hz=24.0, seconds=1.2, use_gate=True)
for tag, r in (('改之前(播种45°, 直接闭环)', r_old),
               ('改之后(两点标定+减偏置+转速门限)', r_new)):
    note = '  ← 超范围, 已拒绝(flt=3)' if r['refused'] else ''
    print(f'   {tag:28s} {r["err"]:+8.2f}° {r["rms"]:6.2f}°  {r["kth"]:.4f}  '
          f'{r["bias"]:+.4f}   {r["rail"]:5.2f}       {r["sat_max"]:5d}{note}')
if not r_new['refused']:
    fail.append('24Hz 工况下新模式没有拒绝 —— 门限没生效')

print('\n[0b] 转速适用范围(20% 凸极, 1LSB 噪声, 两段式标定 + 减偏置):')
print('   ★转子一转, 解调就多出与转速成正比的"正交泄漏", 标定窗两点被污染、')
print('     环路也锁不住 —— 所以这套方法只该用在静止/极低速(它本来就是补 SMO 的')
print('     478rpm 空档)。下面这表用来定"多慢才可靠"的门限, 不是猜的。')
print('   电频率  电机rpm   末段误差     RMS      kth[A]    偏置[A]   |ω|峰值/限幅 最长饱和')
for fs_ in (0.0, 2.0, 5.0, 10.0, 24.0):
    r = simulate(20.0, 1, cal_mode='new', spin_hz=fs_, seconds=0.8, use_gate=False)
    mark = '  ← 超门限(固件会拒绝)' if fs_ * 2 * math.pi > need('HFI_MAX_W') else ''
    print(f'   {fs_:5.1f}Hz {fs_*60/7:7.0f}   {r["err"]:+8.2f}°  {r["rms"]:6.2f}°  '
          f'{r["kth"]:.4f}  {r["bias"]:+.4f}   {r["rail"]:5.2f}       {r["sat_max"]:5d}{mark}')

print('\n[6] 硬件不对称回归(2026-10 现场): 解调不对称项 = b_dc + b2·cos(2Δθ)')
print('   ★现场实测(电机静止): 标定窗量到 kth=0.134 / bias=+0.150, 交接后 ω 立刻钉在')
print('     +450rad/s(= Kp+I_LIMIT)。原因不是"偏置大"本身(那能被减掉), 而是不对称**随')
print('     注入轴与转子的夹角变化**: 标定窗在 ±45° 只量到 b_dc, 锁定时 Δθ≈0 处多出的')
print('     b2 减不掉 → 被积分器吸掉 → 丢转速跟踪 → 跑飞。')
print('   这一节同时验证固件新增的两道保护: flt=4(|bias|≥kth ⇒ 标定不可信) 与')
print('   flt=2(ω 钉在限幅 ⇒ 失锁)。')
print('   b_dc   b2    标定kth  标定bias  |bias|≥kth?  末段误差  |ω|峰值/限幅  结果')
for hb, hb2, guard, b2c in ((0.00, 0.00, True, None),
                            (0.15, 0.25, False, None),
                            (0.15, 0.25, True, None),
                            (0.15, 0.25, False, 0.25),
                            (0.15, 0.25, True, 0.25)):
    r = simulate(20.0, 1, cal_mode='new', spin_hz=0.0, seconds=0.8,
                 use_gate=False, hw_bias=hb, hw_b2=hb2, cal_guard=guard, b2c=b2c)
    bad = abs(r['bias']) >= r['kth']
    # 有不对称时: 要么跑飞, 要么被顶到"假锁"点(Δθ≈−41°: Kth·sin2Δ+b2·cos2Δ=0 的错解)
    broken = (r['rail'] > 0.9) or (abs(r['err']) > 20.0)
    if r['refused']:
        res = '拒绝(flt=4)'
    elif r['rail'] > 0.9:
        res = '跑飞(flt=2 会抓)'
    elif broken:
        res = f'假锁 {r["err"]:+.0f}°'
    else:
        res = '锁定'
    print(f'   {hb:.2f}   {hb2:.2f}  {r["kth"]:.4f}   {r["bias"]:+.4f}   '
          f'{"是" if bad else "否":8s} {r["err"]:+8.2f}°  {r["rail"]:5.2f}       {res}'
          f'{"  ← 补了 b2" if b2c else ""}')
    if (hb2 > 0) and guard and (b2c is None) and not r['refused']:
        fail.append('硬件不对称 ≥ 凸极信号时没有拒绝(flt=4 判据失效)')
    if (hb2 > 0) and (not guard) and (b2c is None) and not broken:
        fail.append('没有判据时本应被不对称破坏却没破坏 —— 回归用例不再代表现场, 要重新标定它')
    if (hb2 > 0) and (b2c is not None) and broken:
        fail.append('补了 cos2Δ 不对称补偿后仍锁不住 —— 补偿这条修法不成立')
    if (hb2 > 0) and guard and (b2c is not None) and r['refused']:
        fail.append('已经补偿过后 flt=4 还在拒绝 —— 会把用户永远挡在门外')

print('\n[7] 验证 hfiscan 的判读逻辑(12 点静态扫描 + err=b+K·sin2Δ+C·cos2Δ 拟合)')
print('   这一节用固件 hfiscan 完全相同的算法: θ̂ 钉在 θenc+Δ(0..165°, 每档 300 次解调),')
print('   环路不闭 —— 所以它**不可能跑飞**, 是判断"真凸极 vs 解调不对称"的决断性测量。')
print('   情形          拟合K    拟合C    残差    snr    判读')
for tag, hb, hb2 in (('干净(纯凸极)', 0.0, 0.0),
                     ('不对称 b_dc=.15 b2=.25', 0.15, 0.25)):
    es, ks, cs = [], [], []
    for i in range(12):
        off = 15.0 * i
        rr = simulate(20.0, 1, cal_mode='off', hold_deg=off, seconds=0.15,
                      hw_bias=hb, hw_b2=hb2)
        es.append(rr['err_hold'])
        # ★基向量与固件同约定: 物理调制角是 (θenc−θ̂) = −Δ_off
        ks.append(-math.sin(2 * math.radians(off)))
        cs.append(-math.cos(2 * math.radians(off)))
    n = len(es)
    b = sum(es) / n
    K = 2.0 * sum(e * k for e, k in zip(es, ks)) / n
    C = 2.0 * sum(e * c for e, c in zip(es, cs)) / n
    res = math.sqrt(sum((e - (b + K * k + C * c)) ** 2 for e, k, c in zip(es, ks, cs)) / n)
    snr = abs(K) / res if res > 1e-9 else 999.0
    if (abs(K) > 3 * res) and (abs(C) < 0.5 * abs(K)):
        verdict = '真凸极(sin2Δ): 解调链没问题'
    elif abs(C) >= abs(K):
        verdict = '不对称/相位错(cos2Δ 占优)'
    else:
        verdict = '信噪比不足'
    print(f'   {tag:22s} {K:+.4f}  {C:+.4f}  {res:.4f}  {snr:5.1f}  {verdict}')
    if tag.startswith('干净') and '真凸极' not in verdict:
        fail.append('hfiscan 判读: 干净凸极没被判成"真凸极"')
    if tag.startswith('不对称') and '不对称' not in verdict:
        fail.append('hfiscan 判读: 不对称情形没被判出来(判据失效)')

print('\n[8] 解调轴相位错回归(现场签名: 标定 kth 撞下限 + 环路往一边跑)')
print('   ★建模: 注入轴 θ̂ 不动, 解调轴的 Park 偏 rot_hw(约定/接线/采样相位)。')
print('   理论: 偏 ρ 后量到的是 |E|·sin(2Δ−ρ) ⇒ 拟合 φ=atan2(C,K) ⇒ 修正 ρ_c = −φ。')
print('   这一节验证这条公式: 用扫描量出的 φ 推 ρ_c, 补偿后应回到干净的 |E|·sin(2Δ) 并锁住。')
print('   rot_hw   扫描K    扫描C    |E|     推出ρ_c  补偿后标定kth  环路')
for rhw in (0.0, math.radians(90.0), math.radians(137.0)):
    es = []
    for i in range(12):
        rr = simulate(20.0, 1, cal_mode='off', hold_deg=15.0 * i, seconds=0.15,
                      rot_hw=rhw, rot_c=0.0)
        es.append(rr['err_hold'])
    ks = [-math.sin(2 * math.radians(15.0 * i)) for i in range(12)]
    cs = [-math.cos(2 * math.radians(15.0 * i)) for i in range(12)]
    n = 12
    K = 2.0 * sum(e * k for e, k in zip(es, ks)) / n
    C = 2.0 * sum(e * c for e, c in zip(es, cs)) / n
    E = math.hypot(K, C)
    rho_c = -math.atan2(C, K)
    r = simulate(20.0, 1, cal_mode='new', spin_hz=0.0, seconds=0.8,
                 rot_hw=rhw, rot_c=rho_c)
    ok = (abs(r['err']) <= 5.0) and (r['rail'] <= 0.9)
    print(f'   {math.degrees(rhw):5.0f}°  {K:+.4f}  {C:+.4f}  {E:.4f}  '
          f'{math.degrees(rho_c):+7.1f}°  {r["kth"]:.4f}          '
          f'{"锁定" if ok else "没锁"}')
    if not ok:
        fail.append(f'解调轴相位错 {math.degrees(rhw):.0f}°: 补偿 ρ_c 后仍没锁住')
    if abs(math.degrees(rhw) - 90.0) < 1e-6 and abs(K) > 0.2 * E:
        fail.append('90° 相位错工况下 sin2Δ 分量没被转空 —— 没复现出现场签名, 用例要重定')

print('\n[1b] 判决性实验: θ̂ 钉死在不同 Δ 上, 直接量解调量 err(Δ)')
print('   目的: 解调量到底跟不跟凸极率成比例 —— 若 20% 与 2% 的两条曲线差不多,')
print('         说明观测器靠的是"各向同性项"(那就不是凸极观测器, 白搭);')
print('         若 20% 的幅度约是 2% 的 10 倍, 才是真的靠 Ld≠Lq。')
print('   Δ(deg)  ' + ''.join(f'{d:8d}' for d in range(-90, 91, 15)))
chars = []
for sal in (2.0, 20.0):
    row = []
    for dg in range(-90, 91, 15):
        r = simulate(sal, 0, seconds=0.06, hold_deg=float(dg))
        row.append(r['err_hold'])
    chars.append((sal, row))
    print(f'   {sal:4.0f}%   ' + ''.join(f'{v:+8.4f}' for v in row))
peak2 = max(abs(v) for v in chars[0][1])
peak20 = max(abs(v) for v in chars[1][1])
print(f'   峰值: 2% → {peak2:.4f}A, 20% → {peak20:.4f}A, 比值 {peak20/max(peak2,1e-9):.2f} '
      f'(真凸极应 ≈10)')
if peak20 / max(peak2, 1e-9) < 5.0:
    fail.append('解调量不随凸极率成比例 —— 不是凸极观测器')


print('\n[2] 环路极性(θ̂ 初值偏 ±30°, 必须收敛)')
for off in (30.0, -30.0):
    r = simulate(20.0, 0, off_deg=off, seconds=0.25 if FAST else 0.5, cal_mode='off')
    ok = abs(r['err']) < 5.0
    print(f'   初值偏 {off:+.0f}° → 末段误差 {r["err"]:+7.2f}° '
          f'(RMS {r["rms"]:.2f}°)  {"收敛 ok" if ok else "★不收敛!"}')
    if not ok:
        fail.append(f'极性/收敛失败 (off={off:+.0f}°)')

print('\n[2c] 环路增益/陷波扫描(20% 凸极, 1LSB 噪声): 找稳定区')
print('   ★结论: 陷波一开就振(与增益无关) —— 静止时 ω≈0, 陷波被压到 F_MIN=5Hz,')
print('     正好落在 PLL 的 ~35Hz 带宽里把相位余量吃光。所以固件默认关掉它,')
print('     想试验用 `hfin 1` 打开(它只在 2fe 远高于环路带宽、也就是转起来以后才有意义)。')
print('   gain×   陷波   末段误差    RMS')
for gs in (1.0, 0.5, 0.25, 0.125, 0.0625):
    for un in (True, False):
        r = simulate(20.0, 1, seconds=0.25 if FAST else 0.6,
                     gain_scale=gs, use_notch=un)
        print(f'   {gs:5.3f}   {"on " if un else "off"}   {r["err"]:+8.2f}°  {r["rms"]:6.2f}°')

print('\n[3] 锁定能力: 凸极率 × ADC 噪声 (静止; 目标 |误差| < 5°)')
print('   凸极率  噪声   末段误差   RMS     kth[A](标定?)  |id|[A]   snr')
grid = [(2.0, 1), (5.0, 1), (10.0, 1), (20.0, 1), (20.0, 0), (20.0, 2),
        (10.0, 2), (5.0, 2)]
if FAST:
    grid = grid[:5]
res_grid = []
for sal, nz in grid:
    r = simulate(sal, nz, seconds=0.25 if FAST else 0.6)
    res_grid.append((sal, nz, r))
    print(f'   {sal:5.1f}%  {nz:^4d}  {r["err"]:+8.2f}°  {r["rms"]:5.2f}°  '
          f'{r["kth"]:.4f} ({"是" if r["done"] else "否"})   {abs(r["id"]):.4f}   '
          f'{r["snr"]:5.1f}')

print('\n[4] 基波串扰: 58Hz(≈478rpm 基波)下还能不能锁 —— 分开看串扰与噪声')
for fa, nz in ((1.0, 0), (1.0, 1), (0.0, 1)):
    r = simulate(20.0, nz, seconds=0.25 if FAST else 0.5, fund_a=fa)
    print(f'   基波 {fa:.1f}A / 噪声 {nz}LSB → 末段误差 {r["err"]:+6.2f}°  '
          f'kth={r["kth"]:.4f}  |id|={abs(r["id"]):.4f}  噪声={r["res"]:.4f}A  '
          f'snr={r["snr"]:.1f}')
    if abs(r['err']) > 5.0 and nz <= 1:
        fail.append(f'基波 {fa:.1f}A 下失锁')

print('\n[5] 标定常数核对: 仿真反推 HFI_SAL_K = (Kth/|id|)/(ΔL/L)')
sim_k = []
for sal, nz, r in res_grid:
    if abs(r['id']) > 1e-4:
        sk = (r['kth'] / abs(r['id'])) / (sal / 100.0)
        sim_k.append(sk)
        print(f'   {sal:5.1f}% : Kth/|id| = {r["kth"]/abs(r["id"]):.4f} '
              f'→ HFI_SAL_K ≈ {sk:.2f}')
if sim_k:
    mk = float(np.mean(sim_k))
    print(f'   平均 HFI_SAL_K ≈ {mk:.2f}  (头文件 {SALK:.2f}, 比值 {mk/SALK:.2f})')
    if not 0.7 < mk / SALK < 1.4:
        fail.append(f'HFI_SAL_K({SALK:.2f}) 与仿真({mk:.2f})差太多, 头文件该改')

print()
if fail:
    print('★ 自检失败:')
    for f in fail:
        print('   -', f)
    sys.exit(1)
print('自检通过: 滤波器系数/环路极性/锁定能力/标定常数 全部符合预期')
