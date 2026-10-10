#ifndef FOC_PARAM_H
#define FOC_PARAM_H

/* ============================================================================
 * FOC_Param.h —— FOC 参数 与 通用变量 的统一入口
 *
 *   ① 参数: 只在本文件(或 App_Config.h)里改, 不要再散落到各个 .c
 *   ② 通用变量: 本文件只做 extern 声明, 定义留在"算它的那个 .c 里"
 *   ③ 板级/环路口径常量(ADC 标定、量程、PID、限幅、CAN 缩放)仍在
 *      App_Config.h, 本文件已把它包含进来
 *
 *   变量归属(谁定义, 谁负责更新):
 *     BSP/src/App_FOC.c  -> 电流/电压/编码器/转速 这些实时量
 *     FOC/src/Encoder.c  -> 编码器内部标定/解包状态(接口见 Encoder.h)
 *     你自己的观测器 .c   -> 观测角度/速度(在下面第 3 节加 extern)
 * ============================================================================ */

#include <stdint.h>
#include "App_Config.h"

/* ---------------------------------------------------------------------------
 * 1. 时间基(必须与 TIM1 ARR / APP_CTRL_HZ 一致)
 *    ★APP_CTRL_HZ 从 40kHz 改回 20kHz 的原因见 App_Config.h 顶部: 观测器把中断
 *      撑到 31.67us, 40kHz 预算只有 25us, 超预算导致采样丢拍、电机转不起来。
 * ------------------------------------------------------------------------- */
#define FOC_FS              ((float)APP_CTRL_HZ)   /* 电流环 = PWM = 采样频率 Hz */
#define FOC_TS              (1.0f / FOC_FS)        /* 控制周期 s (20kHz -> 50us) */

/* ---------------------------------------------------------------------------
 * 2. 电机参数
 *   A. 官方铭牌 —— 直接抄自 robomaster.com 产品页"技术参数"(M3508 P19 + C620)
 *      减速比取值另见 DJI 商城官方文案"减速箱减速比约为 19:1"
 *   B. 由铭牌推导 —— 官方没给电磁参数, 这里用空载点反推, 并注明不确定度
 *   C. 官方没给、必须实测的(相电阻/相电感)
 * ------------------------------------------------------------------------- */

/* ---------- A. 官方铭牌 ---------- */
#define M3508_VOLTAGE_NOMINAL   24.0f     /* V, 额定电压 */
#define M3508_SPEED_NOLOAD      482.0f    /* rpm, 空载转速(输出轴) */
#define M3508_TORQUE_MAX        3.0f      /* N·m, 持续最大扭矩(输出轴) */
#define M3508_SPEED_AT_MAXTRQ   469.0f    /* rpm, 3N·m 下的最大转速(输出轴) */
#define M3508_CURRENT_MAX       20.0f     /* A, C620 最大持续电流 */
#define M3508_GEAR_RATIO        19.0f     /* 减速箱减速比(约) */
#define M3508_MASS              365.0f    /* g, 电机 */
#define M3508_TEMP_MIN          0.0f      /* °C */
#define M3508_TEMP_MAX          50.0f     /* °C */
/* 官方未公布: 极对数、相电阻、相电感、堵转转矩/电流、转矩常数、反电动势常数 */

/* ---------- B. 由铭牌推导(FOC/观测器用) ---------- */
/* 极对数: 官方未列; 你板上 sin/cos 编码器实测"每机械圈 7 个电周期" -> 7 对极(14 极) */
#define FOC_POLE_PAIRS      7u

/* 永磁磁链 ψf[Wb], 峰值约定: e_phase_peak = ωe·ψf,  T = 1.5·p·ψf·iq
 *   空载点反推:  ψf = V_phase_peak / ωe
 *       V_phase_peak = 24/√3 = 13.86 V (SVPWM 线性区峰值, 空载时相电压基本顶到母线)
 *       ωe = (482/19)·2π/60·7 = 6711 rad/s  (空载 482rpm 输出 -> 电机 9158rpm)
 *       => ψf ≈ 2.07e-3 Wb  (上限)
 *   额定点自洽校核(用实测 R=0.194Ω / L=0.097mH):
 *       官方 3N·m @ 469rpm(输出) -> 电机 8911rpm, ωe=6531 rad/s
 *       vq = R·iq + ωe·ψf , vd = -ωe·Lq·iq , |v| 必须 ≤ 24/√3 = 13.86 V
 *       取 η=1(理想减速箱) 反解得 ψf ≈ 1.6~1.7e-3; 若取 η=0.8 则需要 ~14.5V, 已经超了
 *   ⇒ 两个口径差 ~20%, 且用实测 R 时额定点电压都不够用, 说明:
 *     ① 空载转速受摩擦/铁损影响, 反推偏大; ② 实测 R=0.194Ω 很可能含引线+接触电阻(0.2Ω量级!)。
 *   ★要准只有实测: 外部拖动电机, 量线电压峰值 V_ll 与电频率 f_e, ψf = V_ll/(√3·2π·f_e)
 *     暂时取两者中间值 1.9e-3(±15%), 只影响 Kt/转矩估算, 不影响观测器角度(atan2 不用 ψf) */
#define FOC_FLUX            0.0010f       /* Wb - 实测: 观测器 |E|/omega_e 两转速点(1.10e-3/0.96e-3)
                                              * 与"3N.m/10A + 齿轮效率0.7"推出的 1.06e-3 一致 -> 取 1.0e-3
                                              * ★原名 FOC_Flux, 与上面第 B 节的 1.9e-3 估算值混用过, 已统一成
                                              *   这个名字; 观测器只用它换算速度, 角度(atan2)不依赖磁链。 */
#define FOC_KT              (1.5f * (float)FOC_POLE_PAIRS * FOC_FLUX)  /* ≈0.0200 N·m/A(电机轴) */
#define FOC_KE              FOC_FLUX      /* V·s/rad(电角速度->相电压峰值) */
/* 输出轴等效转矩常数 = 19·FOC_KT·η(减速箱效率): 官方 3N·m 若对应约 10A
 *   -> 0.30 N·m/A; 而理想(η=1)时 3N·m 只需 7.3A -> 反推 η ≈ 0.7 (仅参考) */

/* ---------- C. 实测值(用户 LCR/电桥实测, 以此为准) ---------- */
#define FOC_RS              0.194f        /* 相电阻 Ω —— 实测(建议 4 线法复核: 2 线法在 0.2Ω 量级
                                            上引线+接触电阻占比很大) */
#define FOC_LS              0.000097f     /* 相电感 H —— 实测 0.097 mH */
#define FOC_LD              0.000097f     /* d 轴电感 H —— ★待多位置实测后再区分 Ld/Lq */
#define FOC_LQ              0.000097f     /* q 轴电感 H */

#define FOC_INERTIA         0.0f          /* 转动惯量 kg·m² —— ★官方未给, 待实测/估算 */

/* 12V 母线下的预期(电压受限): 空载转速 ≈ 482·12/24 = 241 rpm(输出轴);
 *   电流上限别超 M3508_CURRENT_MAX(20A), 现 APP_CURRENT_PID_LIMIT = 18A ✓ */

/* ---------------------------------------------------------------------------
 * 3. 观测器 / PLL 参数
 * ------------------------------------------------------------------------- */
/* PLL(锁相环, 跟踪观测角度并给出速度):
 *   fn 越高跟踪越快、但噪声越大; 类型 II 环稳态速度误差为 0,
 *   只有加速度下才有 ∝ a/ωn² 的滞后 —— 所以 fn 按"加速度"选, 不按转速选。
 *   电机侧 9000rpm 时电频率约 1050Hz, PLL 不必跟到那么高, 60Hz 足够。 */
#define OBS_PLL_FN_HZ       60.0f
#define OBS_PLL_ZETA        0.707f
#define OBS_PLL_WMAX        (2.0f * 3.14159265f * 1500.0f)  /* 速度限幅 rad/s(防跑飞) */

/* 反电动势有效性门限: |E| = √(E_α²+E_β²) 低于此值 -> SMO 输出 Valid=0, 角度不可信
 *   ★实测(2026-10): 电机静止时残差就有 0.375~0.43V, 用 0.3V 会让观测器拿噪声当角度追。
 *     建议 0.6~0.8V; 但下面这条注释是当年实测记录, 保留供对照, 改的时候一起改。 */
#define OBS_EMF_MIN_V       0.3f
#define OBS_EMF_MIN_SQ      (OBS_EMF_MIN_V * OBS_EMF_MIN_V)

/* 反电动势低通的相位滞后补偿 —— 不做的话偏置随转速漂, 标定不通用!
 *   E_alpha/E_beta 各过一个一阶低通(alpha=SMO_EMF_LPF_ALPHA, fs=APP_CTRL_HZ),
 *   一阶低通在频率 omega 处的相位滞后是 atan(omega/omega_c), 不是 omega*tau!
 *     omega_c = alpha*fs (现 alpha=0.14 @40kHz -> 5600 rad/s, 约 890Hz)
 *   小角近似 omega*tau (= omega/omega_c) 只在低转速成立, 高转速会"过补偿":
 *     1000rpm 电频率 117Hz: we/omega_c = 0.131, atan 与线性近似差 0.0008rad
 *     3000rpm 电频率 350Hz: we/omega_c = 0.395, 差 0.021rad
 *   实测(2026-10, alpha 还是 0.0785 时): 线性近似残差 -0.032rad@2000rpm -> -0.132rad@3000rpm,
 *   多漂 0.100rad, 其中 0.058 正是这个近似误差 -> 必须改用 atan。 */
#define OBS_EMF_WC          (0.14f * (float)APP_CTRL_HZ) /* = alpha*fs; 改 alpha/控制频率要重算 */

/* ---------------------------------------------------------------------------
 * 3b. 开环高速测试台(Open-Loop / V-F + SMO 打点)
 *
 *   ★没有编码器也能跑: 驱动角由固件自己积分(θ_drv = 2π∫fe dt), 这是真正的开环。
 *     观测器(SMO)只看 iαβ / vαβ 给出 θ̂/ω̂/|E|, 用作"电机到底转没转"的唯一反馈。
 *
 *   两种驱动角模式(VF_SetDriveMode, CAN 0x28 / USB 命令 drv):
 *     drv 0 = 开环积分角(默认, 无需编码器) ← 你要的
 *     drv 1 = 编码器电角度 + 转矩角(有编码器的台架上更稳; 也是原实现)
 *
 *   电压幅值按指令频率前馈:  V = V_BOOST + V_SLOPE·fe ,  fe = (电机侧 rpm/60)·极对数
 *   默认值口径: 12V 母线, 电机侧 rpm。
 *   在线可改: CAN 0x21(目标rpm) 0x22(斜率) 0x23(上限) 0x25(偏置) 0x26(斜坡)
 *             0x27(滞后带宽) 0x28(驱动角模式) 0x29(起步对齐电压)
 * ------------------------------------------------------------------------- */
#define OL_RPM_DEF          1500.0f   /* 默认目标转速(电机侧 rpm, 输出轴 ≈ /19) */
#define OL_RAMP_DEF         600.0f    /* 指令转速斜坡 rpm/s */
#define OL_VBOOST_DEF       1.0f      /* 低速补偿电压 V(峰值相电压) */
#define OL_VSLOPE_DEF       0.05f     /* 电压/频率斜率 V/Hz */
#define OL_VMAX_DEF         6.5f      /* 电压幅值上限 V(12V 母线 SVPWM 线性区 ≈6.9V) */
#define OL_TORQUE_ANGLE_DEF 1.57079632679f  /* 编码器模式下磁场超前电角度 rad(π/2) */
#define OL_RPM_MAX          6000.0f   /* 电机侧 rpm 软件限幅 */
/* 开环积分角的起始电频率 Hz。
 *   ★原来取 0.5Hz(电机侧 4.3rpm)太保守: M3508 带 19:1 行星减速箱,
 *     输出轴几 rpm 时静摩擦/齿槽转矩占主导, 牵不进去 -> 表现就是"堵转"。
 *     改 2Hz(电机侧 17rpm, 输出轴 0.9rpm)起步更硬。 */
#define OL_FSTART_DEF       2.0f
/* 起步对齐电压 V: 使能后先在固定角停 OL_ALIGN_MS, 让转子贴到已知位置 */
#define OL_VALIGN_DEF       1.5f
#define OL_ALIGN_MS         300u
/* ★低频强制启动电压 V: 电频率低于 OL_FRES_HZ 时, 电压不低于这个值。
 *   为什么需要: V/F 曲线在 fe→0 时 V→v_boost, 只有 1V, 电流 ≈1V/0.194Ω ≈ 5A,
 *   折算到输出轴转矩 ≈1N·m, 往往顶不住减速箱静摩擦 + 输出轴负载 -> 原地堵转。
 *   把低频段电压强行抬到 2.5~3.5V, 电流上到 12~18A, 才有足够的起动转矩。
 *   ★代价: 电流大、发热快, 所以只是"起步段"用, |E| 建立(牵入成功)后立刻退回 V/F 曲线。 */
#define OL_VBOOST_LO_DEF    2.8f
#define OL_FRES_HZ          6.0f      /* 低频强制段的上界电频率 Hz(电机侧 ≈51rpm) */
/* 失步/堵转判定时间 ms: 电压已加、|E| 却迟迟不达门槛 -> 封驱动。
 *   原来 800ms 对"低频强拖+机械惯量"太短, 起步还没起来就被断电(表现为一冲就停)。 */
#define OL_EMF_TIMEOUT_MS   2000u
/* 观测角对照偏置 rad(诊断): 有编码器时 观测角+Lag+偏置 应等于编码器角。
 *   ★数值标定(tools/verify_smo.py, 600~4000rpm): 20kHz + δ=16A 下补偿 Lag 之后
 *     还剩常数 −28.28°, 取反就是这里的 +0.4936; 标定后残差 ±0.8° 以内。
 *   ★改了控制频率或 δ 之后要重新跑这个脚本重新标定(40kHz 时是 +0.4613)。 */
#define OL_ANGLE_OFF_DEF    (+0.4936f)

/* ---------------------------------------------------------------------------
 * 4. 通用变量(实时量) —— 定义在 BSP/src/App_FOC.c
 * ------------------------------------------------------------------------- */
/* 电流: ADC 原始值 / 换算值(A) / Clarke / 滤波后 / dq */
extern float ADC_Current[3];      /* ADC 原始计数 */
extern float Current[3];          /* 三相电流 A (已减零点) */
extern float csa_vmid[3];         /* 上电标定的电流零点(ADC 计数) */
extern float ialpha;              /* α 轴电流 A (原始) */
extern float ibeta;               /* β 轴电流 A (原始) */
extern float ia_filt;             /* α 轴滤波后 A */
extern float ib_filt;             /* β 轴滤波后 A */
extern float cur_mag;             /* 电流矢量幅值 A */
extern float cur_mag_filt;        /* 幅值低通 */
extern float id;                  /* d 轴电流 A (Park, 用编码器角) */
extern float iq;                  /* q 轴电流 A */
extern float id_filt;             /* d 轴滤波后 A */
extern float iq_filt;             /* q 轴滤波后 A */

/* 电压/母线/温度 */
extern float ADC_Voltage[3];      /* 三相端电压 ADC 计数 */
extern float Voltage[3];          /* 三相端电压 V */
extern float ADC_Power;           /* 母线 ADC 计数 */
extern float Power;               /* 母线电压 V */
extern float ADC_Temp;            /* 温度 ADC 计数 */
extern float Temp;                /* 温度 V(原始, 未换算 °C) */

/* 编码器(sin/cos 原始值 + 解算结果; 详细接口见 Encoder.h) */
extern int   Encoder_Sin;
extern int   Encoder_Cos;
extern float Encoder_Angle;         /* 原始电角度 rad */
extern float Encoder_Elec_Angle;    /* 电角度 rad [0,2pi), 已含对齐偏置 */
extern float Encoder_Mech_Position; /* 上电后连续机械位置 rad */

/* 转速 */
extern float spd_meas_rpm;        /* 编码器差分 + 低通得到机械转速 rpm */

/* ---------------------------------------------------------------------------
 * 5. 加新变量的约定(避免到处 extern)
 *      - 谁算它, 就在谁的 .c 里定义(不加 static, 方便跨模块)
 *      - 在这里加一行 extern 声明, 并注明定义位置
 *      - 只在本文件内部用的量请加 static, 不要放这里
 * ------------------------------------------------------------------------- */

#endif
