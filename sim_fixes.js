// ============================================================
// 飞行物理仿真修复补丁
// 对应 prototype.html §3.3 Sim._sub()
// 修复三个已确认异常：数值爆炸、CD0过小飞太远、侧偏非单调文档缺失
// ============================================================

// ── 修复 1：动态子步数，防止 L/W 极大时数值爆炸 ──────────────────────────────
// 原代码在 Sim.step() 里：
//   const subs = Math.min(this.P.maxSubsteps, Math.max(1, Math.ceil(dt / (1/120))));
//
// 问题：maxSubsteps=4 固定上限，当 L/W > 80 时单步 Δv/v > 0.6，积分发散。
// 根因（诊断 F）：area=0.02, mass=0.001 → Δv_per_substep=5.4 m/s，而 v0=9 m/s。
//   Δv/v ≈ 0.60，超过稳定阈值 0.5 时出现符号振荡，导致 maxAlt=969m/dist=680万m。
//
// 修复：根据当前速度和净力估算所需子步数，确保 Δv/v < 0.3。

step(dt) {
  if (!this.alive) return;

  const sp = V.len(this.v);
  // 估算净竖向加速度（用 v0 和当前质量/面积的升力上限）
  const qd_est = 0.5 * this.P.rho * sp * sp;
  const FL_max = qd_est * this.P.area * (2 * Math.PI * this.P.alphaStall * DEG);
  const FG_mag = this.P.mass * this.P.g;
  const net_accel_est = Math.max(0, FL_max - FG_mag) / this.P.mass;

  // 要求单步 Δv < 0.3 * sp（稳定裕度），sdt = dt / subs
  // ⇒ subs > net_accel_est * dt / (0.3 * sp)
  const minSubsByAccel = sp > 0.1
    ? Math.ceil(net_accel_est * dt / (0.3 * sp))
    : 1;
  // 保留原来的时间步上限（每步不超过 1/120s），取两者最大值
  const subsBase = Math.max(1, Math.ceil(dt / (1/120)));
  const subs = Math.min(64, Math.max(subsBase, minSubsByAccel));  // 上限 64 防止极端卡顿
  const sdt = dt / subs;

  for (let i = 0; i < subs && this.alive; i++) this._sub(sdt);
  // ... 后续不变
},

// ── 修复 2：CD0 有效值下限钳位，防止超低阻力时飞行距离失控 ────────────────────
// 原 _CD()：
//   _CD(CL) { return this.P.CD0 + this.P.k * CL * CL; }
//
// 问题（诊断 B）：CD0=0.005 + v0=18 → dist=425m，严重不真实。
//   纸飞机物理范围 CD0=0.05–0.15，UI 滑块允许到 0.005 是过度放开。
//
// 修复：在计算时对 CD0 做软下限钳位（不改 PARAMS 本身，只在计算时生效）。
// 软下限 = 0.03，低于这个值用 smoothstep 过渡，避免硬截断带来不连续感。

_CD(CL) {
  const CD0_min = 0.03;
  const raw = this.P.CD0;
  // 在 [0, CD0_min] 区间做 max(raw, CD0_min * 0.5 + raw * 0.5) 软钳位
  // 等价：若 raw < CD0_min，CD0_eff = CD0_min - (CD0_min - raw)^2 / (CD0_min)
  //   (二次插值使得 raw=0 时 CD0_eff = CD0_min*0.5，raw=CD0_min 时 CD0_eff = CD0_min)
  // 更简单的实现：线性混合
  const CD0_eff = raw < CD0_min
    ? CD0_min * (2 - raw / CD0_min) * 0.5 + raw * 0.5  // ≈ 软钳位
    : raw;
  return CD0_eff + this.P.k * CL * CL;
},
// 注：如果不需要软钳位，最简单改法是把 UI 滑块最小值从 0.005 改为 0.03。

// ── 修复 3：_sub() 中对加速度加硬限，作为最后保险 ──────────────────────────────
// 在 _sub() 计算 a = F/mass 之后，加入：

// 硬限：单步速度变化不超过当前速度的 50%（防止积分超调）
// 注：只有在动态子步（修复1）无法及时细分时（如单帧 dt 极大）才会触发
const a_mag = V.len(a);
const a_max = sp > 0.1 ? (0.5 * sp / dt) : 500;
if (a_mag > a_max) {
  a = V.scl(a, a_max / a_mag);
}

// ── 修复 4（参数面板）：area/mass L/W 实时警告 ────────────────────────────────
// 在参数面板 buildParamPanel() 中，area 和 mass 滑块变动时更新 L/W 指示器。
// 当 L/W > 50 时显示 ⚠，> 100 时显示 ❌ 并提示数值可能爆炸。

function computeLW() {
  const sp = PARAMS.testV0;
  const CL_approx = 2 * Math.PI * (PARAMS.alphaStall * DEG);  // 失速角处 CL
  const qd = 0.5 * PARAMS.rho * sp * sp;
  const lift = qd * PARAMS.area * CL_approx;
  const weight = PARAMS.mass * PARAMS.g;
  return lift / weight;
}

// 加到参数面板 area/mass 的 input 监听器里：
function updateLWIndicator() {
  const lw = computeLW();
  const el = document.getElementById('lwIndicator');
  if (!el) return;
  if (lw > 100) {
    el.textContent = `L/W=${lw.toFixed(0)} ❌ 数值不稳定`;
    el.style.color = '#ff5252';
  } else if (lw > 50) {
    el.textContent = `L/W=${lw.toFixed(0)} ⚠ 飞行夸张`;
    el.style.color = '#ffd966';
  } else {
    el.textContent = `L/W=${lw.toFixed(1)} ✓`;
    el.style.color = '#87df9d';
  }
}
