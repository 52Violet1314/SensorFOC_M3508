/* tools/smo_sim.c —— 滑模观测器算法验证(主机侧仿真, 编译在 PC 上跑)
 *
 * 用"真值模型"生成 αβ 电流(带测量噪声), 喂给与固件完全相同的观测器代码路径,
 * 检查:
 *   ① 角度能不能跟上(误差均值/标准差 —— 均值是常数偏置, 标准差才是跟踪质量)
 *   ② 速度估计对不对
 *   ③ 转速升高后是否仍稳定(验证滑模增益 k 与边界层 δ 的整定)
 *   ④ ★故意把符号写反(负反馈)时, 误差会变成几百度的乱跳 —— 用来证明符号必须对
 *
 * 编译运行:
 *   gcc -O2 -o smo_sim tools/smo_sim.c -lm && ./smo_sim
 *   gcc -O2 -DBUGGY_SIGN -o smo_sim_bug tools/smo_sim.c -lm && ./smo_sim_bug
 */
#include <math.h>
#include <stdio.h>

/* ---- 与 FOC_Param.h / SMO.h 保持一致 ---- */
#define RS        0.194f
#define LS        0.000097f
#define TS        (1.0f / 20000.0f)
#define PP        7.0f
#define FLUX      0.0010f
#define K_GAIN    50.0f
#define DELTA     8.0f
#define ALPHA     0.14f
#define SPD_ALPHA 0.014f
#define EMF_MIN   0.3f
#define PI_F      3.14159265359f
#define TWO_PI_F  6.28318530718f

static float F, G;
static float i_hat[2];
static float e_lpf[2];
static float theta_prev, omega_e;
static int   theta_valid;

static float sat(float x) { return (x > 1.f) ? 1.f : ((x < -1.f) ? -1.f : x); }

static void smo_reset(void)
{
    F = expf(-TS * RS / LS);          /* 0.9512 */
    G = (1.0f - F) / RS;              /* 0.2514 A/V */
    i_hat[0] = i_hat[1] = 0.0f;
    e_lpf[0] = e_lpf[1] = 0.0f;
    theta_prev = omega_e = 0.0f;
    theta_valid = 0;
}

/* 与 FOC/src/SMO.c 的 SMO_Run() 等价 */
static float smo_step(float ia, float ib, float va, float vb, float *emf_out)
{
    float s0 = ia - i_hat[0];
    float s1 = ib - i_hat[1];
    float w0, w1;
    float th, dth;

#ifndef BUGGY_SIGN
    /* 正确: w = k·sat(i − î) 加在电压上; 滑模面上 w = −e ⇒ e_hat = −w
     * 角度用 θ = atan2(−e_alpha, e_beta)(见 tools/verify_smo.py 的验证结论) */
    w0 = K_GAIN * sat(s0 / DELTA);
    w1 = K_GAIN * sat(s1 / DELTA);
    i_hat[0] = F * i_hat[0] + G * (va + w0);
    i_hat[1] = F * i_hat[1] + G * (vb + w1);
    e_lpf[0] += ALPHA * (-w0 - e_lpf[0]);
    e_lpf[1] += ALPHA * (-w1 - e_lpf[1]);
#else
    /* 旧代码的写法: e = +k·sat(i − î) 却当反电动势从电压里减掉
     *   ⇒ î(k+1) = (1−Ts·R/L)·î + Ts/L·u − Ts·H/L·sat(i − î)
     *   滑模项是同相位的负反馈, î 被推到 i+δ 而不是收敛到 i, 滑模面不存在,
     *   e 被 e_lpf 低通后又减回去 ⇒ 系统退化成"零输入"低通, e_lpf → 0 */
    w0 = K_GAIN * sat(s0 / DELTA);
    w1 = K_GAIN * sat(s1 / DELTA);
    i_hat[0] = F * i_hat[0] + G * (va - w0);
    i_hat[1] = F * i_hat[1] + G * (vb - w1);
    e_lpf[0] += ALPHA * (w0 - e_lpf[0]);
    e_lpf[1] += ALPHA * (w1 - e_lpf[1]);
#endif

    *emf_out = sqrtf(e_lpf[0] * e_lpf[0] + e_lpf[1] * e_lpf[1]);
    th = atan2f(-e_lpf[0], e_lpf[1]);   /* 与固件一致 */

    if (!theta_valid) { theta_prev = th; theta_valid = 1; omega_e = 0.0f; }
    else {
        dth = th - theta_prev;
        if (dth >  PI_F) dth -= TWO_PI_F;
        if (dth < -PI_F) dth += TWO_PI_F;
        theta_prev = th;
        if (*emf_out >= EMF_MIN) omega_e += SPD_ALPHA * (dth / TS - omega_e);
    }
    return th;
}

/* 电机模型: L·di/dt = u − R·i − e,  e = we·ψf·(−sinθ, cosθ)
 * 电压按"上一拍指令、本拍电流"的时序(zero order hold), 与固件里的对齐方式一致 */
int main(void)
{
    static const float rpm_list[] = {300.0f, 600.0f, 1200.0f, 2000.0f, 3000.0f, 4000.0f};
    unsigned seed = 12345u;

#ifdef BUGGY_SIGN
    printf("### 复现旧代码的符号错误(滑模项当负反馈用)\n");
#else
    printf("### 正确符号(ε = i − î, e_hat = −w)\n");
#endif
    printf("%8s %11s %13s %12s %12s %10s\n",
           "rpm", "we[rad/s]", "err_mean[deg]", "err_std[deg]", "rpm_est", "|E|[V]");
    printf("--------------------------------------------------------------------------\n");

    for (unsigned k = 0; k < sizeof(rpm_list) / sizeof(rpm_list[0]); k++) {
        float rpm = rpm_list[k];
        float we = rpm * PP * TWO_PI_F / 60.0f;
        float th = 0.0f;                  /* 真实电角度(模型侧) */
        float va = 0.0f, vb = 0.0f;       /* 本拍施加的电压 */
        double se = 0.0, se2 = 0.0, semf = 0.0;
        long n = 0;
        float emf = 0.0f;
        const int settle = (int)(0.05f / TS);
        const int total  = (int)(0.30f / TS);

        smo_reset();
        for (int i = 0; i < total; i++) {
            /* 电流按固件 Park 口径反推(id=0, iq=+5A): iα=−iq·sinθ, iβ=+iq·cosθ
             * 对应反电动势 e=(−E sinθ, +E cosθ); stator 电压 v = e + R·i */
            float s = sinf(th), c = cosf(th);
            float ia_m = -5.0f * s;
            float ib_m =  5.0f * c;
            float th_hat, d;

            va = (-we * FLUX * s) + RS * ia_m;
            vb = ( we * FLUX * c) + RS * ib_m;

            seed = seed * 1103515245u + 12345u;
            ia_m += ((float)((seed >> 16) & 0xFFu) / 255.0f - 0.5f) * 0.02f;
            seed = seed * 1103515245u + 12345u;
            ib_m += ((float)((seed >> 16) & 0xFFu) / 255.0f - 0.5f) * 0.02f;

            /* 观测器 */
            th_hat = smo_step(ia_m, ib_m, va, vb, &emf);

            th += we * TS;
            if (th > TWO_PI_F) th -= TWO_PI_F;

            if (i > settle) {
                d = th_hat - th;
                while (d >=  PI_F) d -= TWO_PI_F;
                while (d <  -PI_F) d += TWO_PI_F;
                se += d;
                se2 += (double)d * d;
                semf += emf;
                n++;
            }
        }
        {
            double m = se / n;
            double var = se2 / n - m * m;
            printf("%8.0f %11.1f %13.3f %12.3f %12.1f %10.3f\n",
                   rpm, we, m * 180.0 / PI_F,
                   sqrt(var > 0 ? var : 0) * 180.0 / PI_F,
                   omega_e / PP * 9.54929658551f, semf / n);
        }
    }
    return 0;
}

