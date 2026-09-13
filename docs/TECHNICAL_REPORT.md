# 赛车自动驾驶规划与控制技术总结

## 1. 项目范围

本项目在 Assetto Corsa 中搭建了一套从赛道信息、赛车线规划、速度规划到横向与纵向 MPC 控制的完整闭环，并在上海、浙赛和纽北三条赛道上完成验证。

![Project overview](../assets/figures/hero_overview.png)

系统的核心链路是：

```text
Assetto Corsa
  -> shared memory
  -> VehicleState
  -> Frenet 投影与参考线
  -> 横向 LMPC + 纵向 MPC
  -> 虚拟 X360 转向/油门/刹车
  -> Assetto Corsa
```

控制周期为 50 Hz。横向 LMPC 使用 `20 x 20 ms` 预测窗口，最终纵向 MPC 使用 `50 x 50 ms` 预测窗口。

![Closed-loop architecture](../assets/figures/closed_loop_architecture.png)

## 2. 闭环接口与实时信号

### 2.1 AC 共享内存

程序读取三个 AC 标准共享内存页：

| Page | 内容 |
| --- | --- |
| `acpmf_physics` | 车速、发动机、踏板、转向、轮胎、载荷、滑移、速度、角速度和加速度 |
| `acpmf_graphics` | 世界坐标、归一化赛道位置、圈数、出界状态、维修区和路面抓地力 |
| `acpmf_static` | 车型、赛道、布局和车辆静态参数 |

共享内存只有在 `packetId` 持续变化时才被视为有效，避免 AC 退出后残留的旧数据被当成实时状态。

### 2.2 `VehicleState`

控制器使用的核心状态包括：

| 类别 | 信号 |
| --- | --- |
| 时间与状态 | `timestamp`, `packet_id`, `status`, `completed_laps` |
| 运动状态 | `speed_kmh`, `position`, `heading_rad`, `yaw_rate_rad_s` |
| 速度与加速度 | 世界速度、车体局部速度、三轴加速度 |
| 执行器状态 | `gas`, `brake`, `steer`, `gear`, `rpm` |
| 轮胎状态 | 四轮角速度、垂直载荷、滑移、出界数量和路面抓地力 |
| 赛道信息 | `normalized_position`, `track`, `track_configuration` |

### 2.3 控制输入

控制器通过虚拟 Xbox 360 手柄输出：

- 左摇杆 X：转向。
- 左扳机：刹车。
- 右扳机：油门。

输入层同时支持 vJoy 和键盘 PWM，但最终赛道验证使用 `vgamepad`。虚拟手柄接口的优点是可以直接复用 AC 的车载输入链路，不需要修改 AC 本体。

## 3. 路径与速度规划

![Control stack](../assets/figures/control_stack.png)

### 3.1 赛道几何

项目从 AC 的 `fast_lane.ai` 读取参考线，并提取：

- 中心线或快速参考线。
- 左右赛道宽度。
- 离散路径长度与累计弧长。
- 航向角和曲率。
- 起点到闭环的投影关系。

路径会重采样到约 `2 m` 间距，并使用三次样条保持闭环几何连续。

### 3.2 赛车线优化

规划器先按曲率识别弯道，把每个弯道划分为连续窗口。之后使用 L-BFGS-B 优化路径横向偏移：

$$
J =
t_{\text{lap}}
+ w_\kappa \sum_i \kappa_i^2 \Delta s_i
+ w_c \sum_i \max(0, c_{\text{target}} - c_i)^2 \Delta s_i
+ w_s \sum_i \|\Delta^2 o_i\|^2 .
$$

其中：

- $t_{\text{lap}}$ 是静态积分得到的圈速代理。
- $\kappa_i$ 是路径曲率。
- $c_i$ 是参考线到左右边界的距离。
- $o_i$ 是横向偏移。
- 曲率、边界和偏移平滑分别由 $w_\kappa$、$w_c$、$w_s$ 加权。

每圈结束后，规划器根据横向误差、航向误差、转向活动和出界情况决定是否扩大或回退弯道偏移。若新几何的估计圈速变差超过阈值，则拒绝本次更新。

### 3.3 曲率限速

基础曲率限速为：

$$
v_\kappa = \min\left(
v_{\max},
\max\left(
v_{\min},
\sqrt{\frac{a_{\text{lat,plan}}}{|\kappa|}}
\right)
\right).
$$

随后执行：

1. 周期平滑，并取平滑值与原始值中的较小值。
2. 前向加速约束。
3. 后向制动约束。
4. 可选摩擦椭圆约束。

后向制动约束形式为：

$$
v_i^2 \le v_{i+1}^2 + 2a_{\text{brake}}\Delta s_i.
$$

当摩擦椭圆启用时，纵向和横向能力按：

$$
\left(\frac{a_x}{a_{x,\max}}\right)^2+
\left(\frac{a_y}{a_{y,\max}}\right)^2 \le 1
$$

修正。

### 3.4 分速度带增益

每个赛道使用 9 个速度带的倍率表。最终配置为：

| Track | Speed scale by band |
| --- | --- |
| Shanghai | `1.00, 1.16, 1.28, 1.375, 1.375, 1.375, 1.31, 1.33, 1.33` |
| Zhejiang | `1.16, 1.23, 1.24, 1.28, 1.29, 1.29, 1.31, 1.33, 1.33` |
| Nordschleife | `1.00, 1.16, 1.28, 1.375, 1.375, 1.375, 1.31, 1.33, 1.33` |

倍率后的速度仍受总速度上限和曲率上限约束：

$$
v_{\text{scaled}} \le
\sqrt{\frac{a_{\text{lat,scaled}}}{|\kappa|}}\cdot 3.6.
$$

### 3.5 高速制动提前量

高速下控制器额外增加规划响应时间：

$$
\tau_{\text{effective}} =
\tau_{\text{response}}
+ \max(0, v-200)g_{\text{hs}}.
$$

上海 V64 和纽北安全版使用：

$$
g_{\text{hs}}=0.004\ \text{s/(km/h)}.
$$

制动允许速度按前向距离和制动能力限制：

$$
v_{\text{allowed}}^2 \le
v_{\text{ahead}}^2+2a_{\text{brake}}d_{\text{effective}}.
$$

## 4. 横向 LMPC

![MPC equations, page 1](../assets/figures/mpc_equations_page-1.png)

![MPC equations, page 2](../assets/figures/mpc_equations_page-2.png)

### 4.1 Frenet 状态

车辆被投影到参考路径的 Frenet 坐标系，状态定义为：

$$
x =
\begin{bmatrix}
e_y & e_\psi & v_x & v_y & r
\end{bmatrix}^{T},
\qquad
u =
\begin{bmatrix}
\delta & a_x
\end{bmatrix}^{T}.
$$

其中 $e_y$ 是横向误差，$e_\psi$ 是航向误差，$v_x$ 是车体纵向速度，$v_y$ 是横向速度，$r$ 是横摆角速度。

### 4.2 连续模型

基础动态自行车模型为：

$$
\dot e_y = v_x e_\psi + v_y,
$$

$$
\dot e_\psi = r,
$$

$$
\dot v_x = a_x,
$$

$$
\dot v_y =
a_{33}(v_x)v_y+a_{34}(v_x)r+b_{31}(v_x)\delta,
$$

$$
\dot r =
a_{43}(v_x)v_y+a_{44}(v_x)r+b_{41}(v_x)\delta.
$$

代码允许使用辨识得到的速度调度映射覆盖标准轮胎刚度系数：

$$
a_{33}=-\frac{c_{\text{lat}}(v_x)}{v_x},
$$

$$
a_{34}=\frac{c_{\text{lat,yaw}}(v_x)}{v_x}-v_x,
$$

$$
a_{43}=\frac{c_{\text{yaw,vel}}(v_x)}{v_x},
$$

$$
a_{44}=\frac{c_{\text{yaw,damp}}(v_x)}{v_x}.
$$

输入增益 $b_{31}$ 和 $b_{41}$ 同样按速度插值。高横向载荷时，横摆响应还会乘一个 load schedule：

```text
0.0 g -> 1.00
0.2 g -> 0.90
0.4 g -> 0.75
0.6 g -> 0.60
0.8 g -> 0.50
1.2 g -> 0.45
```

这用于表示轮胎接近附着极限时横摆响应下降。

### 4.3 转向执行器模型

当启用转向状态时，状态扩展为：

$$
\dot \delta =
\frac{\delta_{\text{cmd}}-\delta}{\tau_{\text{steer}}}.
$$

最终配置中的执行器时间常数为：

```text
steering_actuator_tau_s = 0.14
steering_filter_tau_s = 0.10
steering_lead_s = 0.03
steering_rate_limit = 2.0 rad/s
steering_jerk_limit = 8.0 rad/s^3
```

控制输出还会经过滤波、相位超前、执行器滞后修正、转向速率限制和转向 jerk 限制。

### 4.4 离散化与参考

连续模型通过前向欧拉离散：

$$
A_d=I+A_c\Delta t,
\qquad
B_d=B_c\Delta t.
$$

并限制离散系统谱半径，避免高速速度桶下的数值发散。

参考序列中：

$$
v_{y,\text{ref}}=0,
\qquad
r_{\text{ref}}=v_x\kappa.
$$

路径点按 `reference_preview_s` 和预测速度推进。最终上海配置使用 `0.16 s` 预瞄。

### 4.5 Cost function

横向 MPC 的二次代价函数为：

$$
J =
\sum_{k=0}^{N-1}
\left[
(x_k-x_{\text{ref},k})^{T}Q_k(x_k-x_{\text{ref},k})
+u_k^{T}R_ku_k
+(\Delta u_k)^{T}R_{d,k}\Delta u_k
\right]
+J_{\text{terminal}}.
$$

终端权重使用 `terminal_weight_scale = 4.0`。

状态权重：

$$
Q_k=\operatorname{diag}
\left(
q_y m_y(v),
q_\psi m_\psi(v),
q_{v_x},
q_{v_y},
q_r m_r(v)
\right).
$$

最终上海/浙赛配置使用：

```text
q_lateral = 30
q_heading = 30
q_speed = 2
q_lateral_velocity = 2
q_yaw_rate = 5
r_steer = 18
r_accel = 0.4
rd_steer = 520
rd_accel = 8
```

速度和载荷相关权重会分别缩放横向、航向、横摆、转向和转向速率项。

### 4.6 约束

主要约束包括：

- 转向位置上下限。
- 转向速率限制。
- 转向 jerk 限制。
- 横向加速度和纵向加速度上下限。
- 转向执行器状态限制。

控制问题转换为 condensed QP，并使用 OSQP 在线求解。若 QP 失败，控制器保留上一帧转向命令，而不是输出不可信解。

## 5. 纵向 MPC

### 5.1 模型

纵向 MPC 使用增量加速度模型：

$$
a_{k+1}=(1-\alpha)a_k+\alpha u_k,
\qquad
\alpha=\frac{\Delta t}{\tau_{\text{response}}},
$$

$$
v_{k+1}=v_k+\Delta t\left(a_{k+1}-a_{\text{coast}}(v_k)\right).
$$

其中 $a_{\text{coast}}$ 是从实测滑行数据得到的速度相关阻力加速度。

### 5.2 Cost function

$$
J =
\sum_{k=0}^{N-1}
\left[
q_v(v_k-v_{\text{ref},k})^2
+q_a(a_k-a_{\text{ref},k})^2
+r_a\Delta a_k^2
+r_j\Delta^2 a_k^2
\right].
$$

最终配置中的关键参数：

```text
horizon = 50
dt = 0.05 s
response_tau_s = 0.08847 s
q_speed = 12
q_accel = 3
r_accel = 1.2
r_jerk = 70
max_accel = 5.0 m/s^2
max_brake = 19.6 m/s^2
max_jerk = 7.6 m/s^3
max_brake_jerk = 39.9 m/s^3
```

加速度与 jerk 的代价会随横向载荷和速度增大，使 MPC 在弯中更加平滑。

### 5.3 约束

纵向控制输入满足：

$$
-a_{\text{brake,max}} \le a_k \le a_{\text{accel,max}},
$$

$$
-j_{\text{brake,max}}\Delta t
\le \Delta a_k \le
j_{\text{accel,max}}\Delta t.
$$

在赛道规划层，每个点的允许减速度还根据速度带、路面抓地力和车辆当前横向载荷限制。

### 5.4 踏板映射

MPC 输出的是目标加速度，不是直接踏板百分比。油门和刹车由实测能力映射得到：

$$
T =
\operatorname{clamp}
\left(
\frac{a_{\text{propulsion}}}
{a_{\text{accel,capability}}(v)},
0,1
\right),
$$

$$
B =
\operatorname{interp}
\left(
v, a_{\text{decel,request}}
\right).
$$

刹车映射使用速度点和踏板点构成二维表，并在速度方向和踏板方向之间线性插值。最终踏板映射还包括：

- 油门上升和下降速率限制。
- 刹车上升和下降速率限制。
- 转向时降低油门。
- 轮胎滑移过大时削减油门。
- 油门和刹车同时输出时互斥。

## 6. 标定与模型辨识

![Calibration pipeline](../assets/figures/calibration_pipeline.png)

### 6.1 转向标定

`tools/calibrate_steering.py` 在约 `30/50/80 km/h` 上执行正弦转向扫描，采集：

- 指令轴和实际转向。
- 横摆角速度。
- 横向和纵向加速度。
- 车体局部速度。
- 轮胎滑移和出界状态。

稳态转向关系为：

$$
\frac{u v}{r}=\frac{L}{k}+\frac{K}{k}v^2.
$$

对 $\frac{u v}{r}$ 与 $v^2$ 做线性最小二乘，可同时得到转向比例 $k$ 和不足转向梯度 $K$。

### 6.2 横向动力学辨识

`tools/identify_mpc_model.py` 和 `tools/fit_calibrated_dynamics.py` 使用低滑移、四轮在界内、路面抓地力正常的样本。

Clean-sample 条件包括：

```text
lap >= configured first lap
0.005 s <= dt <= 0.10 s
vx > 5 m/s
abs(vy) < 15 m/s
abs(yaw_rate) < 1.5 rad/s
abs(steer) < 0.95 * max_steer
tyres_out == 0
surface_grip >= 0.85
```

横向速度导数模型：

$$
\dot v_y+v_x r =
c_1\left(-\frac{v_y}{v_x}\right)
+c_2\delta+c_3.
$$

横摆角加速度模型：

$$
\dot r =
c_4\frac{v_y}{v_x}
+c_5\frac{r}{v_x}
+c_6\delta+c_7.
$$

系数按速度带拟合后再插值成速度调度表。`configs/models/lmpc_calibrated_model_v10.json` 中五个速度带的横向拟合 $R^2$ 为：

| Speed band | Lateral R² |
| --- | ---: |
| 40-70 km/h | 0.979 |
| 70-100 km/h | 0.986 |
| 100-130 km/h | 0.986 |
| 130-160 km/h | 0.976 |
| 160-200 km/h | 0.989 |

### 6.3 纵向制动表

纵向能力来自踏板-减速度实测。标定时保持固定初始速度，对油门或刹车施加分级输入，记录稳态减速度，并形成 `速度 x 踏板` 表。

最终标定表覆盖：

- 速度点：`0, 30, 60, 90, 120, 160, 200, 260 km/h`。
- 踏板点：`0, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.00`。
- 实测减速度范围约为 `0-17.19 m/s²`。

控制器在车速和踏板两个方向做插值，再结合 MPC 输出的目标减速度生成刹车命令。

### 6.4 速度增益标定

速度增益不是直接从单次人工圈复制，而是结合：

- 路径曲率。
- 人工示教速度包络。
- 赛道边界和横向误差。
- 每圈出界情况。
- 纵向加速和制动能力。
- 圈速与稳定性反馈。

最终倍率表按速度带保存，并且每次修改只调整一个速度带，避免整张速度表同时漂移。

## 7. 实测结果

![Performance dashboard](../assets/figures/performance_dashboard.png)

![Track trajectories](../assets/figures/track_maps.png)

### 7.1 上海

| Item | Value |
| --- | ---: |
| Human | 136.248 s |
| Auto | 143.543 s |
| Gap | +7.295 s |
| Auto max speed | 268.7 km/h |
| p95 lateral error | 0.566 m |
| Tyres out | 0 |

自动车在大直道上接近人工驾驶，但在部分中低速弯的入弯和出弯速度不足。主要差距区段约为：

- `1.45-1.50 km`
- `2.95-3.05 km`
- `4.60-4.70 km`

![Shanghai comparison](../assets/figures/comparison_shanghai.png)

### 7.2 浙赛

| Item | Value |
| --- | ---: |
| Human | 94.329 s |
| Auto | 103.764 s |
| Gap | +9.436 s |
| Auto max speed | 229.2 km/h |
| p95 lateral error | 0.535 m |
| Tyres out | 2 |

浙赛自动圈与人工圈的差距主要来自弯中速度和出弯加速。自动车的横摆角速度更平滑，但横向加速度峰值通常比人工低约 `0.2-0.4 g`。

![Zhejiang comparison](../assets/figures/comparison_zhejiang.png)

### 7.3 纽北

V2 safe 版本第一次完整跑完纽北：

| Item | Value |
| --- | ---: |
| Lap time | 526.825 s |
| Max speed | 293.8 km/h |
| Mean speed | 143.6 km/h |
| p95 lateral error | 0.461 m |
| Max lateral error | 1.877 m |
| Tyres out | 0 |

剩余瓶颈是纵向速度执行，而不是横向稳定性。高速度段的 p95 目标速度差达到 `50-87 km/h`，说明整车动力、阻力模型、目标速度恢复斜率或纵向执行能力仍需要继续优化。

![Nordschleife profile](../assets/figures/nordschleife_profile.png)

## 8. 项目解决的关键问题

### 1. 实时闭环和残旧共享内存

问题：AC 退出后共享内存可能仍然存在，直接读取会把旧帧当成实时状态；键盘 PWM 也无法提供连续转向和踏板。

解决：使用 `packetId` 变化判断 AC 是否仍处于实时状态，输入层切换为 vJoy/vgamepad 连续控制，并对 AC 窗口焦点和控制器释放做统一处理。

### 2. 横向模型失配与转向延迟

问题：标准自行车模型对模拟器车辆的高速横摆响应和转向执行器延迟描述不足，导致高速振荡和弯中修舵。

解决：按速度带辨识横向和横摆模型，加入载荷相关横摆衰减，建立转向执行器一阶模型，并使用滤波、超前和滞后补偿。

### 3. 油门、刹车和减速度映射

问题：单纯 PI 速度控制器无法处理制动力、滑行阻力和高速度下的执行器差异。

解决：采集踏板-减速度实测表，建立 coast-down 阻力映射，改用纵向增量 MPC，并加入加速度、jerk 和横纵载荷相关权重。

### 4. 纽北布局解析错误

问题：控制器最初读取了 `ks_nordschleife\endurance\ai\fast_lane.ai`，而车辆实际运行在普通 `nordschleife` 布局。虽然赛道名称相同，但起点附近路线不同，车辆在约 `14.6 s` 处突然撞墙。

解决：当共享内存没有提供布局名时，从 `Documents/Assetto Corsa/cfg/race.ini` 读取 `CONFIG_TRACK`，并按实际布局解析快速参考线。

### 5. 赛车线边界余量不足

问题：纽北某一位置参考线距离右侧边界仅约 `0.40 m`，小于车身半宽。即使横向误差为零，也会压草或出界。

解决：增加硬边界投影，保证最终参考线的目标余量；当局部宽度不足时，将参考线向可用一侧重新投影，并在平滑后再次执行边界约束。

### 6. 速度倍率突破抓地力

问题：局部曲率速度经过倍率放大后，目标侧向加速度达到约 `1.0-1.23 g`，超过当前车辆和路面的可用能力。

解决：在速度倍率之后增加曲率限速，使：

$$
v \le \sqrt{\frac{a_{\text{scaled}}}{|\kappa|}}\cdot 3.6.
$$

纽北安全版使用 `1.02 g` 的倍率后上限、`1.2 m` 参考线余量和 `0.65 g` 基础规划横向加速度，最终实现四轮不出界的完整圈。

## 9. 自动车与人工驾驶的差异

自动车表现更好的部分：

- 横向轨迹更平滑，横摆角速度的突变较少。
- 同一配置多次重复性高，浙赛四圈的差异只有几十毫秒。
- 高速制动和路径跟踪不受驾驶疲劳影响。
- 在部分上海中速区段，自动车速度比人工高约 `7-12 km/h`。

仍然不足的部分：

- 自动车没有充分利用可用横向抓地力，弯中和出弯速度偏低。
- 出弯加速通常比人工晚，尤其在浙赛中低速弯。
- 速度增益表和控制权重仍偏保守，圈速换取稳定性。
- 纵向模型对高速度和动力系统能力的描述还不完整。

## 10. 扩展到实车

可直接复用的部分：

- Frenet 投影、路径重采样、曲率和速度包络。
- 赛车线和边界安全投影。
- 横向 LMPC、纵向 MPC 和标定/辨识流程。
- 日志、圈次分析和离线回放工具。

必须替换或强化的部分：

- 用 RTK-GNSS、IMU、轮速和车辆状态估计替换 AC 真值。
- 用 CAN/线控执行器替换虚拟手柄。
- 重新辨识质量和惯量、轮胎刚度、载荷转移、制动和动力能力。
- 增加独立安全限制、紧急停车、冗余、看门狗和驾驶员接管。
- 在低风险测试场地逐步验证，禁止直接把仿真参数用于实车闭环。

完整说明见 `docs/REAL_CAR_TRANSFER.md`。

## 11. 复现实验

安装依赖并运行测试：

```powershell
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

重新生成人类对比图和项目总览图：

```powershell
python -m tools.build_comparison_figures
python -m tools.build_project_assets
```

运行指定赛道配置：

```powershell
.\tools\start_v64_shanghai.ps1
.\tools\start_zhejiang_final.ps1
.\tools\start_nordschleife_v2_safe.ps1
```

完整环境准备和日志命令见 `docs/QUICKSTART.md`。

## 12. 当前边界

- 最终结果基于 Assetto Corsa，不代表实车性能。
- 浙赛最终同步配置尚未重新完成整圈验证。
- V64 上海是最终候选，验证圈速仍来自 V63。
- 纽北目前是安全验证版本，不是圈速最优版本。
- 原始日志和 vendored acados 不进入公开 Git 历史，使用最终模型文件和依赖清单复现。
