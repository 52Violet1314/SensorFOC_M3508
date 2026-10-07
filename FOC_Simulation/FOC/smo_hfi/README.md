# SMO + PLL + HFI 无位置传感器 FOC 仿真模型

全新独立模型（不改动原有 `../SVPWM.slx`），由脚本可重复生成、可一键跑通。

## 1. 运行方法

```matlab
cd D:/MATLABDoc/FOC/smo_hfi
smo_hfi_main          % 冒烟测试 -> 生成模型 -> 仿真 -> 出图出数据
```

或从命令行：

```bat
matlab -batch "cd('D:/MATLABDoc/FOC/smo_hfi'); smo_hfi_main"
```

产物：`SMO_HFI_FOC.slx`（模型）、`smo_hfi_results.png`（8 张曲线）、
`smo_hfi_results.mat`（原始数据）、`run_log.txt`（运行日志）。

## 2. 文件

| 文件 | 作用 |
|---|---|
| `pmsm_params.m` | 电机/驱动器参数、仿真设置 |
| `mf_Ref_Profile.m` | 转速给定 + 负载转矩曲线 |
| `mf_FOC_Controller.m` | 转速环 + 电流环（估计 dq 坐标系）+ 解耦 + 电压限幅 |
| `mf_PMSM_Plant.m` | 离散 dq 电机模型（含凸极）+ 平均电压逆变器 |
| `mf_SMO_PLL.m` | 滑模电流观测器 + 反电动势 PLL（含滤波相位补偿） |
| `mf_HFI_Observer.m` | 脉振高频注入 + 带通 + 同步解调 + HFI PLL |
| `mf_Observer_Fusion.m` | 两观测器融合（NCO 连续输出 + EMF 加权 + 180° 消歧） |
| `build_smo_hfi_model.m` | 用 Stateflow API 把上面 6 个文件填进 MATLAB Function 块并连线 |
| `run_smo_hfi_sim.m` | 仿真 + 指标 + 出图 |
| `smoke_test.m` / `smo_hfi_main.m` | 工具链自检 / 总入口 |
| `test_hfi_locked.m` / `test_plant_hf.m` / `test_hfi_demod_sign.m` | 离线单元测试（锁定转子、载波解调符号/增益） |
| `diag_smo_hfi.m` | 结果时间表打印 |
| `tw_norm.m` | To Workspace 数据读取 |

`.m` 算法文件同时是模型里 MATLAB Function 块的源码，改文本后重跑
`build_smo_hfi_model` 即可同步进模型。

## 3. 模型结构

```
Ref_Profile --> FOC_Controller --u_pi--> HFI_Observer --u--> [UnitDelay] --> PMSM_Plant
                    ^                        |                                  |
                    |                   theta_hfi,w_hfi,eps                ia,ib,ic,iab,theta
        Delay_theta,w (Ts)                   v                                  |
                    |                 Observer_Fusion <---- SMO_PLL <-----------+
                    +-------------------------+ <-- E_smo
```

- 采样 `Ts = 20 us`，定步长离散，仿真 `0.5 s`。
- 一个采样周期的电压延迟（`Delay_ua/ub`）既表示真实计算延时，也用于打断
  控制器—HFI—融合的代数环；`Delay_theta/Delay_w` 同理作用于角度反馈。
- 逆变器按平均电压建模（未做 PWM 开关），因为重点是观测器而不是调制。

## 4. 算法要点

1. **滑模观测器 + PLL**：alpha-beta 电流观测器 + `sign` 滑模项，低通（200 Hz）
   提取反电动势 → PLL 得角度与转速。低通带来的 `atan(we/wc)` 相位滞后在输出
   端做了补偿，否则高速会固定偏一个角度。
2. **HFI（d 轴脉振载波）**：沿估计 d 轴注入 `Vh*cos(wh*t)`（`Vh=0.5 V, fh=1 kHz`），
   在估计 dq 坐标系取 q 轴载波电流：
   `i_qh = Vh*(1/Ld-1/Lq)/(2*wh) * sin(2*dtheta) * sin(wh*t)`，
   与 `sin(wh*t)` 相乘并在**恰好一个载波周期**上滑动平均，得到
   `eps = Kth*sin(2*dtheta)`，`Kth = Vh*(1/Ld-1/Lq)/(4*wh)`。
   基波电流在估计坐标系里是直流量，被这一平均自然抵消；再加两级
   RBJ 带通（中心正好在载波上）抑制基波，带通在中心频率相位为零，
   所以解调不会偏相、也不会变号。
3. **融合**：NCO（角度跟踪环）输出连续角度；权重由**反电动势幅值**决定
   （`E1=cfg(12)*psi`、`E2=cfg(13)*psi`，本参数下 2.27 V→4.24 V，约
   37→70 rad/s 机械转速），比用转速加权稳得多。上报转速是"观测器转速 +
   30 Hz 低通"，**不含** NCO 修正量——否则角度误差会被当成转速灌进转速环。
4. **180° 消歧**：脉振注入只能定到 `theta mod pi`。反电动势观测器可用后，
   把载波估计折到同一极性分支（融合内部完成）。

## 5. 本次仿真结果（`run_log.txt` 原始数字）

| 工况 | 指标 |
|---|---|
| 0–0.05 s 零速（纯 HFI） | 角度误差 max/rms = 0.000° / 0.000° |
| 0.30–0.40 s 高速 | SMO 误差 max 4.43°，rms 2.95°；融合误差 max 7.18°，rms 4.07° |
| 0.44–0.50 s 负载阶跃 | 融合误差 max 0.70° |
| 转速环保持段 0.36–0.50 s | 误差 rms 1.98 rad/s（90 rad/s 的 2.2%），含负载阶跃瞬态 |
| 0.20 s 斜坡中 | 跟踪误差 −0.39 rad/s |
| 末尾 | 给定 90.00，实际 90.45，融合反馈 90.69 rad/s |

曲线见 `smo_hfi_results.png`。

## 6. 已知限制（重要）

- **凸极是假设**：原 `SVPWM.slx` 的电机（R=0.194Ω、L=0.097mH、ψ=0.01515Wb、
  pp=4）看起来是表贴式，Ld=Lq 时 HFI 无信号。本模型设 `Ld=97uH, Lq=145uH`
  （凸极比 1.49）才有 HFI 可观测性；换成真机参数时请按实测 Ld/Lq 修改。
- **SMO 用单一电感** `L=(Ld+Lq)/2`，忽略凸极，带载下存在几度固有偏差
  （本仿真稳态偏差 2.4°、纹波 rms 1.7°）。
- **HFI 只在低速有效**：转速升高后，一个滑动平均窗内转子已转过可观角度，
  解调失真（本仿真高速段 HFI 误差达 90°~180°）。融合权重在高速段为 1，
  完全屏蔽 HFI，所以不影响控制；但**不能**在高速段用 HFI 输出。
- **载波未做陷波**：电流环没有在 fh 处挖陷波，载波电流会被电流环部分抵消
  （本参数约压掉三成），只影响 HFI 环路增益，不影响正确性。
- **极性辨识/反转**：零速启动时正确锁定，是因为初始角度误差≈0。真正的
  任意初始位置启动还需极性辨识（脉冲注入或反电动势符号）；低速反转穿越
  零点也未做专门处理。
- **速度环带宽敏感**：`Ki_w` 从 5 提到 15 会导致失稳（速度环去追 HFI PLL
  的暂态），因此经整定后固定在 `Kp_w=0.5, Ki_w=5`。这是真实工程里
  "观测器带宽必须高于转速环带宽" 的体现。
- 逆变器为平均电压模型，不含死区、开关纹波、电流采样噪声与延时细节。

## 7. 下一步可做

1) 把被控对象换成 Simscape Electrical 的 PMSM + 两电平逆变器（复用
   `../SVPWM.slx` 里的库块与 SVPWM 算法），控制器与观测器部分可原样搬过去。
2) 电流环加 fh 陷波，提高 HFI 环路增益。
3) 加脉冲极性辨识 + 低速反转工况。
4) 用实测 Ld/Lq/ψ 重新标定，并加电流采样噪声做鲁棒性验证。
