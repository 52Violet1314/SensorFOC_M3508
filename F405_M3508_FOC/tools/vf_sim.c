/* tools/vf_sim.c —— 开环(V/F)启动逻辑验证: 把 FOC/src/VF.c 的算法原样搬过来跑
 *
 * 目的: 回答"为什么堵转/为什么不转"——把电压、电流、频率、能否牵入算出来,
 *       而不是靠猜。用同步电机模型(带齿槽转矩 + 负载转矩 + 静摩擦):
 *
 *   Te = 1.5·p·ψf·iq                 (iq = 电流矢量在 q 轴的分量)
 *   J·dωm/dt = Te − Tload − Tcog(θ) − B·ωm ,  静止时额外加静摩擦 Tstatic
 *
 * 编译: gcc -O2 -o build/sim/vf_sim tools/vf_sim.c -lm
 * 运行: build/sim/vf_sim            (默认参数)
 *       build/sim/vf_sim 4.0 8.0    (vlo=4.0V, fres=8Hz)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PP        7.0
#define FLUX      0.0010f
#define RS        0.194f
#define LS        0.000097f
#define FS        40000.0
#define TS        (1.0 / FS)

/* 机械参数(粗估, 用来判断"够不够起动", 不是精确值) */
#define J_MOTOR   2.0e-6      /* 电机侧转子惯量 kg·m²(含减速箱折算, 粗估) */
#define T_COG     0.010       /* 电机侧齿槽转矩幅值 N·m */
#define T_STATIC  0.020       /* 电机侧静摩擦 N·m(减速箱密封+油脂) */
#define B_VISC    2.0e-6      /* 粘性阻尼 N·m·s/rad */
#define T_LOAD    0.000       /* 输出轴负载折算到电机侧 N·m */

int main(int argc, char **argv)
{
    double vlo  = (argc > 1) ? atof(argv[1]) : 2.8;
    double fres = (argc > 2) ? atof(argv[2]) : 6.0;
    double vboost = 1.0, slope = 0.05, vmax = 6.5, fstart = 2.0, ramp = 200.0;
    double rpm_target = 300.0;

    double theta = 0.0;       /* 驱动角(积分角) */
    double rpm_cmd_i = 0.0;   /* 指令转速斜坡 */
    double th_r = 0.0;        /* 真实转子电角度 */
    double wm = 0.0;          /* 机械角速度 rad/s(电机侧) */
    double ia = 0.0, ib = 0.0;
    double t = 0.0;
    int    printed = 0;

    printf("开环启动仿真: vlo=%.2fV fres=%.1fHz vboost=%.1f slope=%.3f fstart=%.1f ramp=%.0f "
           "目标=%.0frpm(电机侧)\n", vlo, fres, vboost, slope, fstart, ramp, rpm_target);
    printf("机械假设: J=%.1fe-6 kg·m² 齿槽=%.3fN·m 静摩擦=%.3fN·m\n\n",
           J_MOTOR * 1e6, T_COG, T_STATIC);
    printf("%7s %8s %8s %8s %8s %8s %9s\n",
           "t[ms]", "v[V]", "I[A]", "rpm_cmd", "rpm_real", "|E|[V]", "状态");

    for (long k = 0; k < (long)(5.0 / TS); k++) {
        double fe, v, iq, id, Te, we, Tload, acc;
        double s, c, dth;
        int    boost_lo;

        t = k * TS;

        /* --- ① 指令转速斜坡 --- */
        {
            double diff = rpm_target - rpm_cmd_i;
            double step = ramp * TS;
            if (diff > step) rpm_cmd_i += step;
            else if (diff < -step) rpm_cmd_i -= step;
            else rpm_cmd_i = rpm_target;
        }

        /* --- ② 电频率(最低 fstart) --- */
        fe = rpm_cmd_i / 60.0 * PP;
        if (fabs(fe) < fstart) fe = (fe >= 0.0) ? fstart : -fstart;

        /* --- ③ 电压: 牵入前/低频段用 vlo, 之后回 V/F 曲线 --- */
        we = fabs(fe) * 2.0 * M_PI;
        v = vboost + slope * fabs(fe);
        boost_lo = (we * FLUX < 0.25) || (fabs(fe) < fres);   /* |E|<0.25V 视为未牵入 */
        if (boost_lo && vlo > v) v = vlo;
        if (v > vmax) v = vmax;

        /* --- ④ 积分角 --- */
        theta += 2.0 * M_PI * fe * TS;
        while (theta >= 2 * M_PI) theta -= 2 * M_PI;
        while (theta < 0) theta += 2 * M_PI;

        /* --- ⑤ 电机侧电流: 用稳态近似 i = (v − e)/Z, Z = R + jωeL --- */
        s = sin(th_r); c = cos(th_r);
        {
            double ea = -we * 0 - 0;   /* 下面按矢量算 */
            double ex = -we * FLUX * 0; /* 占位, 见下 */
            double va = v * cos(theta), vb = v * sin(theta);
            double eax = -we * FLUX * sin(th_r), ebx = we * FLUX * cos(th_r);
            double wL = we * LS;
            double den = RS * RS + wL * wL;
            /* i = (u − e)·(R − jωL)/|Z|², 取实部/虚部 */
            double ux = va - eax, uy = vb - ebx;
            ia = (ux * RS + uy * wL) / den;
            ib = (ux * (-wL) + uy * RS) / den;
            (void)ea; (void)ex;
        }

        /* --- ⑥ 电流矢量在真实 dq 轴上的分量(驱动角 − 真实角) --- */
        dth = theta - th_r;
        id =  ia * cos(th_r) + ib * sin(th_r);
        iq = -ia * sin(th_r) + ib * cos(th_r);
        (void)id; (void)s; (void)c;

        /* --- ⑦ 转矩与机械运动 --- */
        Te = 1.5 * PP * FLUX * iq;
        Tload = T_LOAD + T_COG * sin(PP * th_r);
        acc = (Te - Tload - B_VISC * wm) / J_MOTOR;
        if (fabs(wm) < 1e-3 && fabs(Te - Tload) < T_STATIC) acc = 0.0;  /* 静摩擦锁住 */
        wm += acc * TS;
        th_r += wm * TS * PP;
        while (th_r >= 2 * M_PI) th_r -= 2 * M_PI;
        while (th_r < 0) th_r += 2 * M_PI;

        /* --- 打印 --- */
        if (k % (long)(0.25 / TS) == 0 || (t < 0.05 && k % 400 == 0)) {
            double rpm_real = wm / (2 * M_PI) * 60.0;
            printf("%7.0f %8.2f %8.1f %8.1f %8.1f %8.3f %9s\n",
                   t * 1000.0, v, sqrt(ia * ia + ib * ib), rpm_cmd_i, rpm_real,
                   we * FLUX, boost_lo ? "boost_lo" : "curve");
            printed++;
            if (printed > 24) break;
        }
    }

    printf("\n结论: ");
    {
        double rpm_real = wm / (2 * M_PI) * 60.0;
        if (rpm_real > 50.0) printf("转起来了(%.0frpm)\n", rpm_real);
        else printf("没转起来(%.1frpm) -> 提高 vlo 或 fres\n", rpm_real);
    }
    return 0;
}
