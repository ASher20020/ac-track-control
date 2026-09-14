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

### 3.6 横向与纵向耦合方式

本项目的“联合速度规划”不是把横向和纵向合并成一个六自由度联合 NLP，而是显式耦合的协调架构：

- 曲率速度包络和速度倍率决定纵向 MPC 的未来速度参考。
- 横向 MPC 的当前转向和横向加速度输入 `combined_braking_scale()`，缩放纵向 MPC 的可用制动能力。
- 高速响应时间补偿进一步提前制动目标。
- 横向载荷会同时增加纵向 MPC 的加速度和 jerk 代价，降低强横向载荷下的纵向激进程度。
- 纵向 MPC 输出加速度，踏板映射器再结合转向、滑移和速率限制输出油门或刹车。

![Coupled speed planning](../assets/figures/coupled_speed_planning.png)

曲率速度、速度倍率和倍率后抓地上限分别为：

$$
v_{\kappa,i}
=
\min\left(
v_{\max},
\max\left(
v_{\min},
\sqrt{\frac{a_{\mathrm{lat,plan}}}{|\kappa_i|}}
\right)
\right),
$$

$$
v_{s,i}
=
\min\left(
v_{\kappa,i}s(v_{\kappa,i}),
\sqrt{\frac{a_{\mathrm{lat,cap}}}{|\kappa_i|}}
\right).
$$

高速制动预瞄和有效制动距离为：

$$
\tau_{\mathrm{eff}}
=
\tau_0
+ \max(0,v_i-200)g_{\mathrm{hs}},
$$

$$
d_{\mathrm{eff}}
=
\max(0,d_i-v_i\tau_{\mathrm{eff}})
+ d_{\mathrm{lead}}.
$$

横向载荷和转向量共同缩放制动能力：

$$
u_b
=
\max\left(
\frac{|\delta|}{\delta_{\max}},
\frac{|a_y|}{a_{y,\mathrm{ref}}}
\right),
\qquad
\gamma_b
=
\sqrt{\max(0.25,1-u_b^2)},
\qquad
a_{\mathrm{brake,eff}}
=
a_{\mathrm{brake}}\gamma_b.
$$

纵向预览先由前向加速度约束生成：

$$
v_{\mathrm{ref},k}
=
\min\left(
v_{\mathrm{ref},k-1}+a_{\mathrm{accel}}\Delta t,
v_{s,k}
\right),
$$

再从后向前施加制动约束：

$$
v_{\mathrm{ref},k}
=
\min\left(
v_{\mathrm{ref},k},
\sqrt{v_{\mathrm{ref},k+1}^2
+2a_{\mathrm{brake,eff}}\Delta s_k}
\right).
$$

横向载荷与速度同时缩放纵向 MPC 的代价权重：

$$
r_a
=
r_{a,0}s_a(v)s_a(|a_y|),
\qquad
r_j
=
r_{j,0}s_j(v)s_j(|a_y|).
$$

因此，当前实现是“共享状态和约束参数的两层 MPC 协调”，不是单个同时求解横向和纵向的联合优化器。这个区别会影响后续扩展：如果实车需要显式联合摩擦圆约束，可以把制动缩放提升为联合 MPC 的约束项。

## 4. 模型构建、辨识与验证

![From nonlinear vehicle equations to real-time QP models](../assets/figures/model_construction.png)

控制器需要先有一个能在 20 ms 控制周期内求解的车辆模型。本章按固定顺序处理：
非线性车辆方程、轮胎近似、速度带辨识、转向执行器增广、纵向能力标定和回放验证。
横向 LMPC 与纵向 MPC 只消费这里得到的模型，不再重复建模。

### 4.1 输入量、状态量与数据清洗

最终 LMPC 的转向输入是虚拟手柄归一化轴
$\delta_{\mathrm{axis}}\in[-1,1]$，车辆状态来自 AC 共享内存。
AC 的 `physics.steerAngle` 与下发的 `cmd_steer` 几乎一比一，只存在约一个采样周期的
执行器滞后：

$$
\mathrm{steer}_{k+1}
\approx
0.9905\,\mathrm{cmd\_steer}_k-0.00058,
\qquad
\rho=0.9957.
$$

因此它描述的是归一化转向轴的内部响应，不是独立前轮转角量测。
模型直接把“轴输入到横向、横摆响应”的映射辨识出来，避免拆分一个当前数据无法辨识的
物理转角比例。

横向辨识使用五个速度带：

```text
40-70, 70-100, 100-130, 130-160, 160-200 km/h
```

进入拟合的样本满足：

```text
0.005 s <= dt <= 0.10 s
vx > 5 m/s
abs(vy) < 15 m/s
abs(yaw_rate) < 1.5 rad/s
abs(steer) < 0.95 * max_steer
tyres_out == 0
surface_grip >= 0.85
wheel_slip_abs_max < 0.25
|lateral_accel| <= 0.80 g
```

速度、横摆角速度和横向速度保留原始时序，导数采用中心差分：

$$
\dot v_y[k]\approx\frac{v_y[k+1]-v_y[k-1]}{t[k+1]-t[k-1]},
\qquad
\dot r[k]\approx\frac{r[k+1]-r[k-1]}{t[k+1]-t[k-1]}.
$$

### 4.2 从非线性自行车模型到线性预测模型

车体坐标下的横向和横摆动力学为：

$$
m(\dot v_y+v_xr)=F_{yf}+F_{yr},
\qquad
I_z\dot r=aF_{yf}-bF_{yr}.
$$

在低滑移区间使用线性轮胎：

$$
F_{yf}=C_f\alpha_f,
\qquad
F_{yr}=C_r\alpha_r.
$$

轮胎侧偏角近似为：

$$
\alpha_f
\approx
\delta-\frac{v_y+ar}{v_x},
\qquad
\alpha_r
\approx
-\frac{v_y-br}{v_x}.
$$

因此侧偏角的核心变量是 $v_y/v_x$ 和 $r/v_x$。
把轮胎力代回动力学方程：

$$
\dot v_y+v_xr
=
-\frac{C_f+C_r}{mv_x}v_y
+\left(\frac{bC_r-aC_f}{mv_x}-v_x\right)r
+\frac{C_f}{m}\delta.
$$

$$
\dot r
=
\frac{bC_r-aC_f}{I_zv_x}v_y
-\frac{a^2C_f+b^2C_r}{I_zv_x}r
+\frac{aC_f}{I_z}\delta.
$$

这就是为什么辨识特征中会出现 $v_y/v_x$ 和 $r/v_x$：
它们来自轮胎侧偏角的定义，不是人为添加的缩放。若直接对 $v_y$ 回归，
系数会变成 $-(C_f+C_r)/(m v_x)$ 并混入速度；使用归一化侧偏角后，
辨识层系数更接近轮胎和车辆参数的组合，速度依赖再由速度调度显式处理。

横向回归目标使用车身横向加速度：

$$
a_y=\dot v_y+v_xr,
$$

而不是只使用 $\dot v_y$。车体横向加速度包含向心项 $v_xr$，
丢掉这一项会把横摆耦合错误地吸收到横向速度导数中。

### 4.3 横向动力学辨识

最终速度带模型由 `tools/fit_calibrated_dynamics.py` 生成；
`tools/identify_mpc_model.py` 用于早期参数估计和交叉检查。
速度带内拟合式为：

$$
a_y
=c_{\mathrm{lat}}\left(-\frac{v_y}{v_x}\right)
+b_{\mathrm{lat}}\delta_{\mathrm{axis}}
+c_{\mathrm{lat},0},
$$

$$
\dot r
=c_{\mathrm{yaw,vel}}\frac{v_y}{v_x}
+c_{\mathrm{yaw,damp}}\frac{r}{v_x}
+b_{\mathrm{yaw}}\delta_{\mathrm{axis}}
+c_{\mathrm{yaw},0}.
$$

系数含义如下：

- `c_lat` 是横向轮胎合力对归一化侧偏角的响应，量级接近 $(C_f+C_r)/m$。
- `c_yaw,vel` 描述横向速度通过轮胎力矩对横摆加速度的耦合。
- `c_yaw,damp` 是横摆阻尼，负号表示它抑制现有横摆角速度。
- `b_lat` 是单位归一化转向轴产生的横向加速度增益。
- `b_yaw` 是单位归一化转向轴产生的横摆角加速度增益。

名义横向方程中还存在一个 `r/v_x` 耦合项。当前最终模型将该耦合设为
`0`，因为单独拟合时它与其他横摆项高度共线、数值条件不好。这个选择是明确的工程简化，
不是理论必然结果，后续可以用更丰富的数据恢复并重新验证。

### 4.4 转向执行器增广与状态矩阵

转向指令不能瞬时变成实际转向量，因此把转向状态保留在预测模型中：

$$
\dot\delta
=
\frac{\delta_{\mathrm{cmd}}-\delta}{\tau_{\mathrm{steer}}}.
$$

五状态模型为：

$$
x=
\begin{bmatrix}
e_y & e_\psi & v_x & v_y & r
\end{bmatrix}^{T},
$$

$$
A_c=
\begin{bmatrix}
0 & v_x & 0 & 1 & 0\\
0 & 0 & 0 & 0 & 1\\
0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & a_{33} & a_{34}\\
0 & 0 & 0 & a_{43} & a_{44}
\end{bmatrix},
\quad
B_c=
\begin{bmatrix}
0 & 0\\
0 & 0\\
0 & 1\\
b_{31} & 0\\
b_{41} & 0
\end{bmatrix}.
$$

加入执行器状态后，状态变为
$[e_y,e_\psi,v_x,v_y,r,\delta]^T$，`b31` 和 `b41` 从输入矩阵移动到状态矩阵，
转向指令只通过 $\pm1/\tau_{\mathrm{steer}}$ 进入执行器状态。

辨识系数还原到状态矩阵的映射为：

$$
a_{33}=-\frac{c_{\mathrm{lat}}}{v_x},
\qquad
a_{34}=\frac{c_{\mathrm{lat,yaw}}}{v_x}-v_x,
\qquad
a_{43}=\frac{c_{\mathrm{yaw,vel}}}{v_x},
$$

$$
a_{44}=\frac{c_{\mathrm{yaw,damp}}}{v_x},
\qquad
b_{31}=b_{\mathrm{lat}},
\qquad
b_{41}=b_{\mathrm{yaw}}.
$$

模型按 `0, 55, 85, 115, 145, 180, 240 km/h` 插值，并在每个预测阶段缓存
离散矩阵：

$$
A_d=I+A_{\mathrm{aug}}(v_x)\Delta t,
\qquad
B_d=B_{\mathrm{aug}}(v_x)\Delta t.
$$

低速时把 $v_x$ 下限限制为 `3 m/s`，避免 $c/v_x$ 数值发散。

### 4.5 纵向模型与能力标定

纵向模型从速度动力学和一阶执行器响应开始：

$$
\dot v=a_x-a_{\mathrm{coast}}(v),
\qquad
\dot a=\frac{u-a}{\tau_a}.
$$

其中 $a_{\mathrm{coast}}(v)$ 是实测滑行阻力，
$u$ 是 MPC 的目标加速度。前向欧拉离散为：

$$
\alpha=\frac{\Delta t}{\tau_a},
\qquad
a_{k+1}=(1-\alpha)a_k+\alpha u_k,
$$

$$
v_{k+1}=v_k+\Delta t\left(a_{k+1}-a_{\mathrm{coast}}(v_k)\right).
$$

状态空间形式为：

$$
\begin{bmatrix}
v_{k+1}\\
a_{k+1}
\end{bmatrix}
=
\begin{bmatrix}
1 & \Delta t(1-\alpha)\\
0 & 1-\alpha
\end{bmatrix}
\begin{bmatrix}
v_k\\
a_k
\end{bmatrix}
+
\begin{bmatrix}
\Delta t\alpha\\
\alpha
\end{bmatrix}
u_k
+
\begin{bmatrix}
-\Delta t a_{\mathrm{coast}}(v_k)\\
0
\end{bmatrix}.
$$

纵向能力来自踏板实测。标定得到：

- 油门加速能力表 $a_{\mathrm{cap}}(v)$。
- 制动减速度表 $D(v,p_b)$。
- 响应时间常数 `response_tau_s = 0.08847 s`。
- 速度点 `0, 30, 60, 90, 120, 160, 200, 260 km/h`。
- 踏板点 `0, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.00`。

### 4.6 回放验证

回归 $R^2$ 不能替代时序检查，因此取两段干净回放做单步预测对比：

![Lateral identification replay validation](../assets/figures/model_replay_comparison.png)

| 回放 | 中位速度 | 横向加速度 $R^2$ | 横摆角加速度 $R^2$ |
| --- | ---: | ---: | ---: |
| Replay A | 63.7 km/h | 0.978 | 0.946 |
| Replay B | 119.5 km/h | 0.962 | 0.984 |

图中实线是从日志读取的实测值，虚线是用实测状态和实测转向输入计算的单步模型预测。
两段都没有进行开环长时间积分，因此它验证的是辨识模型的一步预测能力，
不夸大为完整车辆仿真。

### 4.7 拟合结果与适用边界

| 速度带 | 样本 | c_lat | c_yaw,vel | c_yaw,damp | b_lat | b_yaw | 横向 R² | 横摆 R² |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 40-70 | 6205 | 212.98 | 28.37 | -254.70 | 72.60 | 42.77 | 0.979 | 0.923 |
| 70-100 | 5969 | 322.30 | 36.10 | -249.86 | 81.27 | 41.48 | 0.986 | 0.894 |
| 100-130 | 3773 | 332.79 | 44.76 | -234.27 | 76.94 | 40.76 | 0.986 | 0.913 |
| 130-160 | 1540 | 312.83 | 42.36 | -241.01 | 72.68 | 43.04 | 0.976 | 0.972 |
| 160-200 | 1497 | 319.38 | 43.20 | -241.90 | 73.08 | 42.91 | 0.989 | 0.951 |

模型边界为：

- 只覆盖低到中等滑移区间，不描述轮胎接近附着极限时的强非线性。
- 当前横向模型省略 `r/v_x` 耦合项。
- 高速带样本较少，`160-200 km/h` 以上主要依赖速度表外推和在线修正。
- 模型输入是归一化转向轴，不声称辨识出了真实前轮转角。

## 5. 横向 LMPC

![MPC equations, page 1](../assets/figures/mpc_equations_page-1.png)

![MPC equations, page 2](../assets/figures/mpc_equations_page-2.png)

![Linear model matrices](../assets/figures/mpc_equations_page-3.png)

![From nonlinear vehicle equations to real-time QP models](../assets/figures/model_construction.png)

### 5.1 Frenet 状态与横向跟踪

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

设参考路径为 $\mathbf r(s)$，其单位切向量和法向量分别为
$\mathbf t(s)$ 和 $\mathbf n(s)$。车辆位置先投影到最近路径点，
再将位置和航向表示为

$$
\mathbf p=\mathbf r(s)+e_y\mathbf n(s),
\qquad
e_\psi=\psi-\psi_{\mathrm{ref}}(s).
$$

其中 $e_y$ 是带符号横向误差，$e_\psi$ 是相对路径切线的航向误差。
$v_x$、$v_y$ 和 $r$ 仍保留在车体坐标系中，所以状态同时包含“赛道相对误差”
和“车辆自身动态”两类信息。

这里的 $\delta$ 是模型层面的有效转向输入。名义自行车模型中它表示前轮转角；
辨识后的线性模型直接使用归一化转向轴，并把量纲和游戏内部转向映射全部吸收到
速度调度的输入增益中。6.1 节会说明为什么在没有独立物理转角量测时，
再拆出一个“轴到物理转角”的比例是数学上不可辨识的。

使用 Frenet 坐标有三个直接原因。

1. 控制器真正关心的是赛道相对误差，而不是世界坐标中的绝对位置。
   路肩边界、赛车线偏移和目标路径都可以直接写成 $e_y$ 的约束；
   在笛卡尔坐标里，同样的边界需要每帧重新搜索最近点，并且约束会随赛道位置变化。
2. 曲率可以自然进入预测模型。参考路径在 $s$ 处的稳态横摆角速度为
   $r_{\mathrm{ref}}=v_x\kappa$，因此规划器给出的曲率可以直接变成 MPC 的前馈项。
   世界坐标本身并不携带赛道曲率，仍需要在控制器内部重新估计。
3. 在赛车线附近，$e_y$、$e_\psi$、$v_y$ 和 $r$ 都是围绕参考状态的小量，
   可以按速度段线性化并预计算矩阵，随后把预测写成小型 condensed QP。
   这套结构比在每个控制周期内求解完整非线性世界坐标模型更适合 50 Hz 闭环。

代码中的路径投影使用连续线段而不是直接取最近的离散索引，并传入上一帧索引
`previous_index` 保持跨起点和不同圈数的投影连续性。代价是模型依赖两个假设：
横向误差满足 $|e_y\kappa|<1$，且路径切线与车辆航向的夹角较小。
当车辆已经远离赛道或参考路径出现尖角时，小角度 Frenet 模型会失真；
因此工程实现仍保留投影有效性检查和降级控制器。

### 5.2 离散化与参考

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

### 5.3 Cost function

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

### 5.4 权重调度

权重不是一次性静态常数，而是由速度、横向载荷和纵向速度共同调度：

| 调度项 | 节点 | 最终倍率 |
| --- | --- | --- |
| 横向误差 $q_y$ | 0, 40, 80, 140, 220 km/h | 1.000, 1.276, 1.050, 0.700, 0.650 |
| 航向误差 $q_\psi$ | 0, 40, 80, 140, 220 km/h | 1.000, 1.217, 1.040, 0.950, 0.900 |
| 横摆角速度 $q_r$ | 0, 40, 80, 140, 220 km/h | 1.000, 1.469, 1.242, 1.150, 1.200 |
| 转向 $r_\delta$ | 0, 40, 80, 140, 220 km/h | 1.600, 2.645, 3.024, 3.500, 4.000 |
| 转向速率 $r_{\dot\delta}$ | 0, 40, 80, 140, 220 km/h | 1.600, 3.620, 3.680, 4.000, 5.000 |

纵向 MPC 使用两组调度：

| 调度项 | 节点 | 最终倍率 |
| --- | --- | --- |
| 加速度代价-横向载荷 | 0, 0.3, 0.6, 0.9, 1.2 g | 1.00, 1.10, 1.30, 1.55, 1.80 |
| Jerk 代价-横向载荷 | 0, 0.3, 0.6, 0.9, 1.2 g | 1.00, 1.20, 1.50, 1.90, 2.40 |
| 加速度代价-速度 | 0, 80, 140, 200, 260 km/h | 1.00, 1.05, 1.15, 1.30, 1.45 |
| Jerk 代价-速度 | 0, 80, 140, 200, 260 km/h | 1.00, 1.15, 1.40, 1.75, 2.00 |

![MPC weight schedules](../assets/figures/weight_schedules.png)

速度越高，转向控制越平滑；横向载荷越大，纵向 MPC 越不愿意快速改变加速度或 jerk。这样可以减少高速修舵和强横向载荷下的纵向扰动。

### 5.5 约束与 QP

主要约束包括：

- 转向位置上下限。
- 转向速率限制。
- 转向 jerk 限制。
- 横向加速度和纵向加速度上下限。
- 转向执行器状态限制。

控制问题转换为 condensed QP，并使用 OSQP 在线求解。若 QP 失败，控制器保留上一帧转向命令，而不是输出不可信解。

## 6. 纵向 MPC与踏板映射

### 6.1 纵向预测模型

纵向模型已经在 4.5 节辨识完成。MPC 直接使用状态
$x_{\mathrm{lon}}=[v,a]^T$、滑行阻力表以及离散矩阵
$A_{\mathrm{lon}}$、$B_{\mathrm{lon}}$：

$$
\begin{bmatrix}
v_{k+1}\\
a_{k+1}
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
1 & \Delta t(1-\alpha)\\
0 & 1-\alpha
\end{bmatrix}}_{A_{\mathrm{lon}}}
\begin{bmatrix}
v_k\\
a_k
\end{bmatrix}
+
\underbrace{
\begin{bmatrix}
\Delta t\alpha\\
\alpha
\end{bmatrix}}_{B_{\mathrm{lon}}}
u_k
+
\begin{bmatrix}
-\Delta t\,a_{\mathrm{coast}}(v_k)\\
0
\end{bmatrix}.
$$

QP 的控制量不是绝对加速度 $u_k$，而是加速度增量
$\Delta u_k=u_k-u_{k-1}$。这样 jerk 惩罚可以直接写成
$\Delta u_k^2$，同时绝对加速度仍通过累积关系参与上下限约束。
这种写法保持问题为小型二次规划，适合在线求解。

### 6.2 Cost function

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

### 6.3 约束

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

### 6.4 加速度到油门与刹车

MPC 输出的是目标加速度 $a_{\mathrm{cmd}}$，踏板映射器负责把它变成虚拟手柄的
油门 $T$ 和刹车 $B$。映射前先补偿滑行阻力：

$$
a_{\mathrm{prop}}
=
a_{\mathrm{cmd}}+\max(0,a_{\mathrm{coast}}(v)).
$$

这样当 MPC 要求匀速时，控制器仍需输出少量油门抵消空气阻力和滚动阻力，
而不是把零加速度误解成“两个踏板都必须为零”。

踏板工作模式带有滞回，避免油门和刹车在零加速度附近抖动：

- 强减速时进入刹车模式，减速度回到较小值后退出。
- 小幅正加速度时进入油门模式，需求回落并穿越阈值后退出。
- 中间死区进入滑行模式，油门和刹车同时为零。

油门由实测加速能力归一化：

$$
 T =
\operatorname{clamp}
\left(
\frac{a_{\mathrm{prop}}}
{a_{\mathrm{cap}}(v)},
0,1
\right),
$$

再乘以轮胎滑移和转向降扭：

$$
T_{\mathrm{limit}}
=
T_{\mathrm{speed}}(v)
\left(1-r_{\delta}|\delta|\right)
T_{\mathrm{slip}}.
$$

刹车不是简单按减速度比例缩放到 1，而是反查实测二维表：

$$
B
=
\mathcal{I}_{v,a}^{-1}
\left(-a_{\mathrm{prop}}\right).
$$

踏板表先在速度方向插值，再在减速度方向反查踏板。最后还会执行以下限制：

- 油门上升和下降速率限制。
- 刹车上升和下降速率限制。
- 转向时降低油门。
- 轮胎滑移过大时削减油门。
- 油门和刹车同时输出时互斥。
- 明显超速时，在进入踏板映射前额外插入一个保底制动请求。

因此纵向执行链是“MPC 目标加速度 → 阻力补偿 → 工作模式 → 油门能力归一化或
刹车表反查 → 速率/转向/滑移限制 → 虚拟踏板”，不是把 MPC 输出直接当成踏板百分比。

## 7. 实测结果

![Performance dashboard](../assets/figures/performance_dashboard.png)

![Track trajectories](../assets/figures/track_maps.png)

### 7.1 上海

| Item | Value |
| --- | ---: |
| Human | 2:16.248 |
| Auto | 2:23.543 |
| Gap | +0:07.295 |
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
| Human | 1:34.329 |
| Auto | 1:43.764 |
| Gap | +0:09.436 |
| Auto max speed | 229.2 km/h |
| p95 lateral error | 0.535 m |
| Tyres out | 2 |

浙赛自动圈与人工圈的差距主要来自弯中速度和出弯加速。自动车的横摆角速度更平滑，但横向加速度峰值通常比人工低约 `0.2-0.4 g`。

![Zhejiang comparison](../assets/figures/comparison_zhejiang.png)

### 7.3 纽北

V2 safe 版本第一次完整跑完纽北：

| Item | Value |
| --- | ---: |
| Human reference lap | 7:17.632 |
| Automatic lap | 8:46.825 |
| Gap | +1:29.193 |
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
