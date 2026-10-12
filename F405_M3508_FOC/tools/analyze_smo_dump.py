"""离线复核 SMO 对照打点: data/smo_dump_2026.csv
列(corr,raw,lut,mech,spd,out,iq,vdc,obs,err) 见 BSP/src/App_FOC.c:1302 / :695

用法: python tools/analyze_smo_dump.py [csv]
"""
import sys
import numpy as np
import pandas as pd

DEG = 180.0 / np.pi
TWO_PI = 2.0 * np.pi
NAME = ['corr', 'raw', 'lut', 'mech', 'spd', 'out', 'iq', 'vdc', 'obs', 'err']
PATH = sys.argv[1] if len(sys.argv) > 1 else 'data/smo_dump_2026.csv'


def wrap_pm(x):
    return (x + np.pi) % TWO_PI - np.pi


df = pd.read_csv(PATH, comment='#')
c = {n: df[n].to_numpy(float) for n in NAME}
n = len(df)
print(f'样本 {n} 行   文件 {PATH}')

# ---------------- 0. 恒等式审计(同时验证抄录无误) ----------------
e_lut = wrap_pm(c['corr'] - c['raw'] - c['lut'])
e_err = wrap_pm(c['obs'] - c['corr'] - c['err'])
print(f"[审计] corr=wrap(raw+lut) 违反 {int((abs(e_lut)>3e-4).sum())} 行, 最大残差 {abs(e_lut).max():.2e}")
print(f"[审计] err =wrap(obs-corr) 违反 {int((abs(e_err)>3e-4).sum())} 行, 最大残差 {abs(e_err).max():.2e}")

# ---------------- 1. 时间基 ----------------
dmech = np.diff(c['mech'])
spd = c['spd'][:-1]
dt = dmech / (spd * TWO_PI / 60.0) * 1e3          # ms
print(f"\n[时间基] 帧间隔(由 mech/编码器转速反推): 中位 {np.median(dt):.3f} ms "
      f"均值 {dt.mean():.3f} min {dt.min():.3f} max {dt.max():.3f}")
hist, edges = np.histogram(dt, bins=np.arange(0, 3.6, 0.1))
for h, lo in zip(hist, edges[:-1]):
    if h:
        print(f"   {lo:4.1f}~{lo+0.1:.1f} ms : {'#'*min(h,60)} {h}")
tick = dt / 0.1                                    # 以 100us 中断为单位的拍数
print(f"   拍数(按 100us): 中位 {np.median(tick):.1f}, 非整数散布 ±{(tick-np.round(tick)).std():.2f} 拍")
jump = np.where(dmech > 1.6 * np.median(dmech))[0]
print(f"   疑似丢帧 {len(jump)} 处; 最长间隔 {dt.max():.2f} ms")

# ---------------- 2. 角度统计 ----------------
def stat(tag, x):
    x = wrap_pm(x)
    print(f'   {tag:>4}: 均值 {x.mean()*DEG:+7.2f}°  标准差 {x.std()*DEG:6.2f}°'
          f'  峰峰 {np.ptp(x)*DEG:6.2f}°  RMS {np.sqrt((x**2).mean())*DEG:6.2f}°')

r = wrap_pm(c['err'] + c['lut'])        # = SMO角(未扣常数) - 编码器粗角
print('\n[电角度误差]')
stat('lut', c['lut'])
stat('err', c['err'])
stat('r', r)
print(f'   corr(err,lut) = {np.corrcoef(c["err"], c["lut"])[0,1]:+.3f}'
      f'   corr(err,spd) = {np.corrcoef(c["err"], c["spd"])[0,1]:+.3f}'
      f'   corr(err,iq) = {np.corrcoef(c["err"], c["iq"])[0,1]:+.3f}')

# err 里有多少是"表的形状": err = a*lut + 残差
lut0 = c['lut'] - c['lut'].mean()
a = float(np.dot(c['err'], lut0) / np.dot(lut0, lut0))
resid = c['err'] - a * lut0
print(f'   err ≈ {a:+.3f}·(lut−均值) + 残差: 残差 RMS = {resid.std()*DEG:.2f}°'
      f'  (err 总 RMS {c["err"].std()*DEG:.2f}°)')

# ---------------- 3. 按电角度折叠的谐波(复)对比 ----------------
def harm(theta, y, kmax=14):
    cols = [np.ones_like(theta)]
    for k in range(1, kmax + 1):
        cols += [np.cos(k * theta), np.sin(k * theta)]
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    r2 = 1.0 - ((y - pred) ** 2).sum() / ((y - y.mean()) ** 2).sum()
    return coef, r2


def amps(coef, kmax=14):
    return np.hypot(coef[1::2], coef[2::2])


th = c['corr']
kmax = 14
co_l, r2_l = harm(th, c['lut'], kmax)
co_e, r2_e = harm(th, c['err'], kmax)
co_r, r2_r = harm(th, r, kmax)
al, ae, ar = amps(co_l, kmax), amps(co_e, kmax), amps(co_r, kmax)
print(f'\n[按电角度折叠的最小二乘谐波, 单位度]  ({n} 点 / 约 '
      f'{abs(np.ptp(c["mech"]))*7/TWO_PI:.1f} 个电周期)')
print('   阶次 ' + ''.join(f'{k:6d}' for k in range(1, kmax + 1)))
print('   lut  ' + ''.join(f'{v*DEG:6.2f}' for v in al) + f'   交流RMS {np.sqrt((al**2).sum()/2)*DEG:5.2f}°')
print('   err  ' + ''.join(f'{v*DEG:6.2f}' for v in ae) + f'   交流RMS {np.sqrt((ae**2).sum()/2)*DEG:5.2f}°')
print('   r    ' + ''.join(f'{v*DEG:6.2f}' for v in ar) + f'   交流RMS {np.sqrt((ar**2).sum()/2)*DEG:5.2f}°')
print(f'   拟合优度 R²: lut {r2_l:.3f}   err {r2_e:.3f}   r {r2_r:.3f}')

print('\n[每阶: err 与 lut 的相位关系]  (θ_k = err 相位 − lut 相位; 0°=同相, 180°=反相)')
for k in range(1, kmax + 1):
    if max(al[k-1], ae[k-1]) * DEG < 0.5:
        continue
    cl = co_l[1 + 2*(k-1)] - 1j * co_l[2 + 2*(k-1)]      # 复幅值
    ce = co_e[1 + 2*(k-1)] - 1j * co_e[2 + 2*(k-1)]
    dphi = np.angle(ce) - np.angle(cl)
    print(f'   k={k:2d}: |lut| {al[k-1]*DEG:5.2f}°  |err| {ae[k-1]*DEG:5.2f}°  '
          f'比值 {ae[k-1]/al[k-1] if al[k-1]>0 else float("nan"):5.2f}  相差 {np.degrees(dphi):+7.1f}°')

# ---------------- 4. err 里与角度无关(动态)的部分 ----------------
def fit_resid(theta, y, coef, kmax=14):
    cols = [np.ones_like(theta)]
    for k in range(1, kmax + 1):
        cols += [np.cos(k * theta), np.sin(k * theta)]
    return y - np.column_stack(cols) @ coef


res_e = fit_resid(th, c['err'], co_e, kmax)
print(f'\n[汇总] err 总 RMS {c["err"].std()*DEG:.2f}°'
      f' = 角度确定性部分 {np.sqrt((ae**2).sum()/2)*DEG:.2f}° (R²={r2_e:.2f})'
      f' ⊕ 剩余(动态/随机) {res_e.std()*DEG:.2f}°')

# ---------------- 5. 剩余量里有没有"机械周期"成分(电角度表修不了的那种) ----------------
# 电角度查表只能表示"每电周期整数阶"的误差; 编码器偏心/自身磁钢误差是按机械角周期的,
# 在电角度域里是 1/7、12/7 这种非整数阶, 表结构上就补不掉。
thm = c['mech'] * 1.0 / 7.0 * 0 + c['mech']            # 机械角(rad, 累计)
mech_rev = thm / TWO_PI
print(f'\n[机械角域检验] 本段转过 {np.ptp(mech_rev):.2f} 个机械转; '
      f'对剩余量做机械阶次 m 的拟合, 报最大几项:')
rows = []
for m in range(1, 85):
    A = np.column_stack([np.cos(m * thm), np.sin(m * thm)])
    coef, *_ = np.linalg.lstsq(A, res_e, rcond=None)
    rows.append((m, float(np.hypot(*coef))))
rows.sort(key=lambda t: -t[1])
print('   机械阶 m: ' + ', '.join(f'{m}({a*DEG:.2f}°)' for m, a in rows[:6]))
print(f'   参考: 剩余量总 RMS {res_e.std()*DEG:.2f}°  '
      f'(单阶拟合幅值 {rows[0][1]*DEG:.2f}° 就是最大机械周期成分)')
