# AeroThrow 3D 技术实现方案

> 基于 `mrd.md` 的完整技术实现方案。本文先列出关键假设，再分层展开模块设计。

---

## 关键假设（先对齐）

1. **引擎与语言**：Cocos Creator 3.8.x + TypeScript（MRD 已定）。
2. **物理用自研而非引擎刚体**：Cocos 内置物理（Bullet/PhysX）只解算刚体碰撞，不会算升力/阻力/失速，对纸飞机不合适。地面也只需做 `y <= 0` 判定，不需要碰撞解算。
3. **MRD 第 4 节流程缺第 2 步**：补成「瞄准/校准」——读取重力锁定初始姿态，提示玩家进入握持姿势。最终需要确认。
4. **初始速度由加速度积分得到**，再用一次现场标定补偿系统偏差（详见 §3.2）。
5. **飞行中机身姿态固定为出手瞬间姿态**（不做被动气动对齐），这样横滚直接转化为侧向偏移，最贴合 MRD 的「横滚角影响飞行路线」描述。

---

## 一、整体分层

```
┌──────────────────────────────────────────────────┐
│  Scenes:  Ready → Aim → Throw → Flight → Result  │   场景/状态机
├──────────────────────────────────────────────────┤
│  Gameplay Controllers  ·  HUD  ·  VFX/Audio      │   表现层
├──────────────────────────────────────────────────┤
│  PhysicsSim  ·  ThrowDetector  ·  OrientTracker  │   领域逻辑
├──────────────────────────────────────────────────┤
│  SensorService  ·  StorageService  ·  WxBridge   │   平台抽象
└──────────────────────────────────────────────────┘
```

数据单向：Sensor → 领域逻辑（纯函数 / 状态对象）→ Controller → 渲染。所有跨层通信走轻量 EventBus，便于回放与单测。

---

## 二、工程结构

```
assets/
  scenes/  Boot.scene  Game.scene  (单 Game 场景内切子节点)
  scripts/
    core/         GameStateMachine.ts  EventBus.ts  Constants.ts
    input/        SensorService.ts      ThrowDetector.ts
                  OrientationTracker.ts PhoneFrame.ts
    physics/      PaperPlaneSim.ts      AeroCurves.ts
    gameplay/     ReadyCtrl.ts AimCtrl.ts ThrowCtrl.ts
                  FlightCtrl.ts ResultCtrl.ts
    render/       FlightCamera.ts GroundRuler.ts HUD.ts
    services/     StaminaService.ts StorageService.ts
                  VibrationService.ts LeaderboardBridge.ts
    config/       GameConfig.ts (g, ρ, S, m, CL/CD 表, 阈值)
  prefabs/        Plane.prefab  RulerSegment.prefab
  resources/      textures/  models/  audio/
build-templates/wechatgame/   (小游戏壳)
```

---

## 三、核心模块设计

### 3.1 传感器层（SensorService + PhoneFrame）

**采样**：`wx.startAccelerometer({ interval: 'game' })`（约 20 ms / 50 Hz）+ `wx.startGyroscope({ interval: 'game' })`。MRD 要求 ≥60 Hz——`'game'` 不一定够，需要在 onChange 回调里做时间戳插值，或退而求其次用 60 Hz 等效估计冲量积分（积分对采样率不敏感，峰值检测会有 ±15 ms 抖动，可接受）。

**坐标系约定**：Cocos Creator 默认右手系，**+X 右 / +Y 上 / +Z 背向场景（朝玩家）**，所以"飞行前进方向" = 世界 **-Z**。手机本体系也是右手系：+X 机右 / +Y 机顶 / +Z 屏幕外（出屏方向）。下面所有推导都基于这两个基。

**坐标变换**。MRD §3.1 规定的握持姿态（四点完全自洽，由前三点经叉乘唯一推出第四点）：
- 手机底部（机头，麦克风 / 充电口侧）→ 世界前方 (-Z)
- 手机左侧边缘 → 世界上方 (+Y)（屏幕左侧朝上）
- 手机右侧边缘（物理按键侧）→ 世界下方 (-Y)（屏幕右侧朝下）
- 屏幕法向 +Z → 世界左方 (-X)（屏幕朝左）

由此得到从手机本体系到世界系的常值旋转矩阵 R（列向量为各手机轴在世界系下的方向）：

| 世界方向 | 手机本体轴 |
|---|---|
| 前 (-Z_w) | -Y (机底) |
| 上 (+Y_w) | -X (机左) |
| 右 (+X_w) | -Z (屏幕背向 / 入屏方向) |

写成代码：

```ts
PhoneFrame.toWorld(v) = (-v.z, -v.x, v.y)   // 输入手机系 (x,y,z)，输出世界系 (X,Y,Z)
```

所有后续的「加速度 / 角速度」都先过这一步，再叠加「动态姿态偏差」（陀螺仪累积的相对 q0 的旋转）——把「握姿基准」和「动态偏差」分开调试，比硬写一个组合矩阵清楚得多。

**握姿验证**：在 ARMED 状态下读取 200 ms 静止加速度均值 `a_rest`（WeChat 加速度计采用「比力」约定，静止时输出 = -gravity 在手机系下的表示）。本姿态下重力 (0,-g,0)_world 投影到手机系是 (+g, 0, 0)，所以 `a_rest ≈ (-g, 0, 0)`。判据：`a_rest.x < -0.85·g` 且 |a_rest.y|、|a_rest.z| < 0.4·g。不满足则提示玩家调整姿势。

### 3.2 投掷检测与出手参数采集

四步状态机：`IDLE → ARMED → SWINGING → RELEASED`。

1. **ARMED**：玩家点「准备」。用 200 ms 静止窗口的加速度均值 `a_rest` 执行 §3.1 末尾的握姿验证；不通过则 UI 提示调整。**握姿合法后不做任何姿态校准**——直接用加速度计读数作为绝对姿态来源（见第 4 步）。
2. **SWINGING**：每帧维护一个 ~800 ms 的滚动缓冲，存原始加速度 `a` 和前向加速度大小 `a_fwd`（标准握姿下 `a_fwd ≈ -a_y`，因为机头沿手机 -Y 方向，前进意味着手机系下 -Y 方向加速）。
3. **触发释放**：当 `a_fwd` 出现 3 样本局部峰值且大于阈值（默认 10 m/s²），即判定为出手瞬间 `t_release`。
4. **RELEASED**：在 `t_release` 取样：
   - **初速** `v0 = ∫ a_fwd dt` 从摆动起点到 `t_release`，再乘以经验系数 `k≈1.1`（弥补离散积分截断）。给一个 `[3, 18] m/s` 的硬钳位防异常。
   - **俯仰 / 横滚**：**直接从加速度计绝对值解算**，无需陀螺仪积分。在挥动起始（`a_fwd < 0.5`）前的最后 5 个样本对 `a` 做平均（此时运动加速度可忽略，`a_phone ≈ -g_phone`），然后：
     - `pitch = atan2(-a.y, -a.x)` —— 底部抬高 → `a.y` 变负 → pitch 正值
     - `roll  = atan2( a.z, -a.x)` —— 机身右倾 → `a.z` 变正 → roll 正值
     - 推导：标准握姿下 `g_phone = (g, 0, 0)`，绕机身 right（手机 -Z）旋转 θ 得 `g_phone = (g·cos θ, g·sin θ, 0)`；绕机头（手机 -Y）旋转 φ 得 `g_phone = (g·cos φ, 0, -g·sin φ)`。
   - 触发 `wx.vibrateShort` 给脱手反馈。

**为什么不用陀螺仪积分**：iOS Safari 的 `DeviceMotionEvent.rotationRate` 在不同设备/屏幕方向锁定下，alpha/beta/gamma 的轴定义和符号约定经常与 W3C 标准不一致；加上积分漂移和 ARMED 通过后强制 snap 到 `Q_CANON` 丢失实际握持偏差，结果会出现"pitch/roll 互换"的诡异现象。加速度计读数在挥动起始前是稳定的重力向量，直接解算无积分误差、无轴混淆风险。

**给物理仿真的初始姿态**：从 `pitch`、`roll` 重建世界系下的 `forward` 和 `up`：
- 标准姿态 `fwd₀=(0,0,-1)`、`up₀=(0,1,0)`、`right₀=(1,0,0)`
- 应用 pitch（绕世界 +X 轴）：`fwd=(0, sin p, -cos p)`，`up=(0, cos p, sin p)`
- 应用 roll（绕新 fwd 轴）：`up = up·cos r + right·sin r = (sin r, cos r·cos p, cos r·sin p)`

健壮性：释放时如果 `pitch` 在 ±60° 之外，提示重投。

### 3.3 物理仿真（PaperPlaneSim）

**状态向量**：位置 `p`、速度 `v`、机身姿态四元数 `q_body`（出手后保持不变）。

**每帧（固定 dt=1/60 s，game frame 内可多步）：**

```
forward_body = q_body · (0,0,1)
up_body      = q_body · (0,1,0)
speed = |v|
v_hat = v / speed

α = signed_angle(v_hat, forward_body)       // 迎角

CL = aeroCurve.lift(α)                       // 见下
CD = aeroCurve.drag(α)
q_dyn = 0.5 * ρ * speed²                     // 动压

# 升力方向：在 v 与 up_body 张成的平面内、与 v 垂直的单位向量
lift_dir = normalize(cross(v_hat, cross(up_body, v_hat)))

F  = q_dyn * S * (CL * lift_dir - CD * v_hat) + (0, -m*g, 0)
v += (F / m) * dt
p += v * dt
```

**升阻系数表**（线性 + 失速分段）：
- `CL(α) = 2π·α`，α∈[-12°, 12°]
- α∈[12°, 18°]：从峰值 `CL_max ≈ 1.3` 线性降到 `0.6`
- α > 18°：保持 0.6（深失速）
- `CD(α) = CD0 + k·CL²`，`CD0=0.06`、`k=0.10`

**横滚 → 偏移的物理来源**：`up_body` 因横滚倾斜，升力方向也随之倾斜，水平分量造成持续侧向加速度。这就是 MRD 要的「横滚角影响飞行路线」，无需额外特殊代码。

**积分器**：semi-implicit Euler 足够（速度 < 25 m/s、dt=1/60，能量误差可忽略）。如果实测发现失速段震荡，再换 RK4。

**结束**：`p.y <= 0` 即落地，记录此时 `p`。

**结算计算**：
- 距离 = `p.z`（沿基准线方向投影；基准线是从原点指向 +Z 的射线，垂直距离即 `p.z`）
- 横向偏移 = `|p.x|`
- 最高点 = 飞行过程中 `max(p.y)`
- 飞行时长 = 累计仿真时间

### 3.4 渲染与相机

**地面与标尺**（GroundRuler）：用一块 200×400 m 的草地平面（可滚动平铺纹理，避免做大模型），`+Z` 方向画一条粗白线作为基准线，每 5 m 一个刻度 prefab，按相机距离做 LOD 隐藏。

**第一人称相机**（FlightCamera）：挂在飞机节点下，本地偏移 `(0, 0.05, -0.1)`（机身正后稍上方）。相机 forward 跟随机身 forward，但用 0.15 s 一阶低通避免完全刚体跟随导致的眩晕。

**HUD**：Canvas 覆盖层，60 Hz 更新文本：距离、最大高度、时长、横向偏移。

**VFX**：飞机后方拖尾粒子；速度高时屏幕边缘速度线（fragment shader 简单 mask）。

### 3.5 状态机

`Boot → Ready → Aim → Throw → Flight → Result → Ready`。每个状态对应一个 Controller 节点，状态切换时只 enable/disable 子树，无场景切换，避免微信小游戏的资源重新加载延迟。

---

## 四、微信小游戏平台适配

| 项 | 实现 |
|---|---|
| 启动 | 在 Boot 场景内一次性 `wx.login` + `wx.getSystemInfoSync` 检查传感器可用性 |
| 权限 | `wx.startAccelerometer` 失败时回退到 UI 提示 + 触屏滑动投掷的降级模式（Phase 2，可先不做） |
| 包体 | 主包仅 Boot + 通用脚本，Game 场景资源走分包（subpackage） |
| 持久化 | `wx.setStorage` 存体力、最佳成绩、设置；用 wrapper 做 schema 版本号 |
| 排行榜 | 开放数据域（`open-data`）单独项目，主域通过 `wx.getOpenDataContext()` postMessage 同步分数 |
| 振动 | `wx.vibrateShort({ type: 'medium' })` 在出手与落地各一次 |
| 性能 | 关闭物理引擎（`physics.enabled=false`），地面纹理 ETC2 压缩，目标静态内存 < 80 MB |

---

## 五、配置与可调参数（GameConfig.ts）

把所有「会调」的常量集中在一处，避免散在各模块：

```ts
PLANE: { mass: 0.005 /*kg*/, area: 0.02 /*m²*/ }
AIR:   { rho: 1.225 }
WORLD: { g: 9.81 }
AERO:  { CLMax: 1.3, alphaStall: 12*deg, CD0: 0.06, k: 0.10 }
THROW: { triggerAccel: 25 /*m/s²*/, vMin: 3, vMax: 18, kImpulse: 1.1 }
SIM:   { dtFixed: 1/60, maxSubsteps: 3 }
```

调参直接改这一份；上线前用一组录制的传感器轨迹做回归测试。

---

## 六、关键风险与待澄清

1. **MRD 第 4 节缺第 2 步流程**——是单独的「姿势校准」页，还是直接进入持续监听？建议确认。
2. **初速估计精度**：纯加速度积分误差 ±20%。若觉得不够稳，可在新手引导里收集 5 次「挥但不投」的轨迹做用户级标定，把 `kImpulse` 个性化。
3. **采样率**：微信 `'game'` interval 在低端机会降到 30 Hz 以下。是否对低端机做硬性帧率门槛或降级提示，需要产品决策。
4. **失速效果可见性**：在第一人称相机下「俯冲失速」很难被玩家感知到——是否要在 HUD 上给个失速指示灯，或在结算页用回放展示？
5. **iOS 设备方向权限**：iOS Safari / 小游戏需要用户首次手势触发后才允许读取陀螺仪，UI 流程要把「准备」按钮作为首次手势。
6. **多分辨率**：第一人称相机的 FOV 在不同设备宽高比下视觉距离感差异较大，需要按 aspect 调 FOV。

---

## 落地建议

建议从 §3.1 + §3.2（传感器 → 投掷检测）起步——这部分的体感是整个游戏的成败点，越早能在真机上跑越好。
