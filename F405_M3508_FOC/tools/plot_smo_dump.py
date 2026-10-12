"""把 SMO 对照打点画成图: 上=按电角度折叠的曲线, 下=谐波幅值对比
输出 data/smo_dump_2026.png
"""
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

DEG = 180.0 / np.pi
TWO_PI = 2.0 * np.pi
FONT = 'C:/Windows/Fonts/msyh.ttc'
W, H = 1180, 780
BG = (250, 250, 252)

df = pd.read_csv('data/smo_dump_2026.csv', comment='#')
corr = df['corr'].to_numpy(float)
lut = df['lut'].to_numpy(float)
err = df['err'].to_numpy(float)
r = (err + lut + np.pi) % TWO_PI - np.pi

img = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(img)
f20 = ImageFont.truetype(FONT, 20)
f17 = ImageFont.truetype(FONT, 17)
f15 = ImageFont.truetype(FONT, 15)

# ---------- 面板 A: 折叠曲线 ----------
ax0, ay0, ax1, ay1 = 90, 90, W - 40, 400
NB = 72
edges = np.linspace(0, TWO_PI, NB + 1)
idx = np.clip(((corr % TWO_PI) / TWO_PI * NB).astype(int), 0, NB - 1)
xs = (edges[:-1] + edges[1:]) / 2 / TWO_PI * 360
curve = {}
for name, y in (('lut', lut), ('r', r), ('err', err)):
    m = np.array([y[idx == i].mean() if (idx == i).any() else np.nan for i in range(NB)])
    curve[name] = m
allv = np.concatenate([curve['lut'], curve['r'], curve['err']])
v0, v1 = np.nanmin(allv) * DEG, np.nanmax(allv) * DEG
pad = 0.12 * (v1 - v0 + 1e-6)
v0, v1 = v0 - pad, v1 + pad


def X(deg):
    return ax0 + (deg / 360.0) * (ax1 - ax0)


def Y(deg):
    return ay1 - (deg - v0) / (v1 - v0) * (ay1 - ay0)


d.rectangle([ax0, ay0, ax1, ay1], outline=(160, 160, 170))
for g in range(0, 361, 60):
    d.line([X(g), ay0, X(g), ay1], fill=(228, 228, 234))
    d.text((X(g) - 14, ay1 + 8), f'{g}°', font=f15, fill=(90, 90, 100))
vstep = 5
gv = int(np.ceil(v0 / vstep)) * vstep
while gv <= v1:
    d.line([ax0, Y(gv), ax1, Y(gv)], fill=(236, 236, 242))
    d.text((14, Y(gv) - 9), f'{gv:+d}°', font=f15, fill=(90, 90, 100))
    gv += vstep
for name, col in (('r', (230, 140, 30)), ('lut', (40, 110, 220)), ('err', (110, 110, 120))):
    pts = [(X(x), Y(v)) for x, v in zip(xs, curve[name] * DEG) if np.isfinite(v)]
    d.line(pts, fill=col, width=3 if name != 'err' else 2)
d.text((ax0, 52), '按电角度折叠(72 格平均, 200 个采样点)', font=f20, fill=(40, 40, 50))
d.text((ax0 + 470, 58), '— r = SMO 量到的编码器真实误差', font=f15, fill=(230, 140, 30))
d.text((ax0 + 470, 78), '— lut = 查表补的量(认为的误差)', font=f15, fill=(40, 110, 220))
d.text((ax0 + 470, 98), '— err = obs − corr(打点第 10 列)', font=f15, fill=(110, 110, 120))
d.text((ax0, ay1 + 30), '第 2/4 阶(编码器 sin/cos 非线性) r 与 lut 基本重合 → 表是对的;'
                        ' 第 12 阶(齿槽族)两者反相 → 表补反了', font=f15, fill=(60, 60, 70))

# ---------- 面板 B: 谐波柱状 ----------
bx0, by0, bx1, by1 = 90, 500, W - 40, H - 60
KMAX = 14


def harm(theta, y, kmax=KMAX):
    cols = [np.ones_like(theta)]
    for k in range(1, kmax + 1):
        cols += [np.cos(k * theta), np.sin(k * theta)]
    coef, *_ = np.linalg.lstsq(np.column_stack(cols), y, rcond=None)
    return np.hypot(coef[1::2], coef[2::2]) * DEG


A = {'lut': harm(corr, lut), 'r': harm(corr, r), 'err': harm(corr, err)}
amax = max(a.max() for a in A.values()) * 1.15
d.rectangle([bx0, by0, bx1, by1], outline=(160, 160, 170))
sl = (bx1 - bx0) / KMAX
for i, k in enumerate(range(1, KMAX + 1)):
    x = bx0 + i * sl
    for j, (name, col) in enumerate((('lut', (40, 110, 220)), ('r', (230, 140, 30)), ('err', (150, 90, 190)))):
        h = A[name][i] / amax * (by1 - by0)
        x0 = x + 6 + j * (sl - 12) / 3
        d.rectangle([x0, by1 - h, x0 + (sl - 12) / 3 - 3, by1], fill=col)
    d.text((x + sl / 2 - 12, by1 + 8), f'{k}', font=f15, fill=(80, 80, 90))
d.text((bx0, 466), '各阶幅值对比(度, 按电角度做最小二乘拟合)', font=f20, fill=(40, 40, 50))
d.text((bx0 + 520, 472), '— lut   — r   — err', font=f15, fill=(70, 70, 80))
for gv in np.arange(0, amax, 1.0):
    d.line([bx0, by1 - gv / amax * (by1 - by0), bx1, by1 - gv / amax * (by1 - by0)], fill=(238, 238, 244))
    d.text((46, by1 - gv / amax * (by1 - by0) - 9), f'{gv:.0f}°', font=f15, fill=(90, 90, 100))
d.text((bx0 + 130, by1 + 30), '阶次 k (每电周期)', font=f15, fill=(80, 80, 90))

img.save('data/smo_dump_2026.png')
print('saved data/smo_dump_2026.png')
print('折叠曲线范围: r %+.2f~%+.2f°, lut %+.2f~%+.2f°, err %+.2f~%+.2f°'
      % (np.nanmin(curve['r']) * DEG, np.nanmax(curve['r']) * DEG,
         np.nanmin(curve['lut']) * DEG, np.nanmax(curve['lut']) * DEG,
         np.nanmin(curve['err']) * DEG, np.nanmax(curve['err']) * DEG))
