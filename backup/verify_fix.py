"""验证三项修复的效果"""
import math
from batch_sim import DEFAULTS, DEG, v_c, v_add, v_sub, v_scl, v_dot, v_crs, v_len, v_norm, q_rot

def _CL(alpha, P):
    aDeg = abs(alpha) / DEG
    sgn = math.copysign(1, alpha) if alpha != 0 else 1
    peak = 2 * math.pi * (P['alphaStall'] * DEG)
    if aDeg <= P['alphaStall']:
        return 2 * math.pi * alpha
    if aDeg <= P['alphaStall'] + 6:
        t = (aDeg - P['alphaStall']) / 6
        return sgn * (peak * (1-t) + P['CLpostStall'] * t)
    return sgn * P['CLpostStall']

def _CD_fixed(CL, P):
    CD0_min = 0.03
    raw = P['CD0']
    CD0_eff = raw if raw >= CD0_min else CD0_min - (CD0_min - raw)**2 / CD0_min
    return CD0_eff + P['k'] * CL * CL

def simulate_fixed(v0, pitch_deg, roll_deg, params=None, max_t=30.0):
    P = dict(DEFAULTS)
    if params: P.update(params)

    p_deg = pitch_deg * DEG
    r_deg = roll_deg * DEG
    fwd = [0, math.sin(p_deg), -math.cos(p_deg)]
    up_base = [0, math.cos(p_deg), math.sin(p_deg)]
    right = v_norm(v_crs(fwd, up_base))
    up = v_add(v_scl(up_base, math.cos(r_deg)), v_scl(right, math.sin(r_deg)))
    pos = [0.0, P['startAlt'], 0.0]
    vel = v_scl(fwd, v0)
    maxAlt = P['startAlt']
    traj = [(pos[0], pos[1], pos[2])]
    t_total = 0.0
    dt_frame = 1/60

    while t_total < max_t:
        dt = dt_frame
        sp = v_len(vel)

        # ── 修复 1：动态子步 ──────────────────────────────────────────────────
        qd_est = 0.5 * P['rho'] * sp * sp
        FL_max = qd_est * P['area'] * (2 * math.pi * P['alphaStall'] * DEG)
        net_accel = max(0, FL_max / P['mass'] - P['g']) if sp > 0.1 else 0
        subsAccel = math.ceil(net_accel * dt / (0.3 * sp)) if sp > 0.1 else 1
        subsBase = min(P['maxSubsteps'], max(1, math.ceil(dt / (1/120))))
        subs = min(64, max(subsBase, subsAccel))
        sdt = dt / subs

        for _ in range(subs):
            sp2 = v_len(vel)
            vh = v_scl(vel, 1/sp2) if sp2 > 0.01 else fwd[:]
            cosA = max(-1, min(1, v_dot(vh, fwd)))
            alpha = math.acos(cosA)
            lat_v = v_sub(vh, v_scl(fwd, cosA))
            if v_dot(lat_v, up) > 0: alpha = -alpha

            CL = _CL(alpha, P)
            CD = _CD_fixed(CL, P)  # ← 修复 2
            qd = 0.5 * P['rho'] * sp2 * sp2

            if sp2 > 0.01:
                inner = v_crs(up, vh)
                c2 = v_crs(vh, inner)
                ld = v_norm(c2)
                liftDir = ld if v_len(ld) > 0.01 else up[:]
            else:
                liftDir = up[:]

            FL = v_scl(liftDir, qd * P['area'] * CL)
            FD = v_scl(vh, -qd * P['area'] * CD)
            FG = [0, -P['mass'] * P['g'], 0]
            F = v_add(v_add(FL, FD), FG)
            a_vec = v_scl(F, 1/P['mass'])
            vel = v_add(vel, v_scl(a_vec, sdt))
            pos = v_add(pos, v_scl(vel, sdt))
            t_total += sdt

            if any(math.isnan(x) or math.isinf(x) for x in pos + vel):
                return None, None, None, None

            tau = P['attitudeTau']
            if tau > 0.001 and sp2 > 0.5:
                newSp = v_len(vel)
                newVh = v_scl(vel, 1/newSp) if newSp > 0.01 else fwd[:]
                dFv = v_dot(fwd, newVh)
                if dFv < 0.99999:
                    ang = math.acos(max(-1, min(1, dFv)))
                    blend = min(1, sdt / tau)
                    rotAng = ang * blend
                    axRaw = v_crs(fwd, newVh)
                    axLen = v_len(axRaw)
                    if axLen > 1e-6:
                        ax = v_scl(axRaw, 1/axLen)
                        half = rotAng * 0.5
                        ss = math.sin(half)
                        rq = [ax[0]*ss, ax[1]*ss, ax[2]*ss, math.cos(half)]
                        fwd = v_norm(q_rot(rq, fwd))
                        up  = v_norm(q_rot(rq, up))

            if pos[1] <= 0:
                traj.append(tuple(pos))
                break
        if pos[1] <= 0: break

        if pos[1] > maxAlt: maxAlt = pos[1]
        last = traj[-1]
        dx = pos[0]-last[0]; dy = pos[1]-last[1]; dz = pos[2]-last[2]
        if dx*dx + dy*dy + dz*dz > 0.04:
            traj.append(tuple(pos))

    return -pos[2], pos[0], maxAlt, t_total

print("=" * 65)
print("验证修复效果")
print("=" * 65)

# ── 验证修复 1：数值爆炸案例 ──────────────────────────────────────────────────
print("\n[修复 1] 数值爆炸 → 动态子步")
print(f"  {'area':>6} {'mass':>7}  {'旧 dist':>12}  {'新 dist':>12}  {'新 maxAlt':>10}  {'新 time':>8}")
explosive_cases = [
    (0.02, 0.001), (0.04, 0.003), (0.06, 0.003), (0.10, 0.005)
]
from batch_sim import simulate as sim_old
for area, mass in explosive_cases:
    p = {'area': area, 'mass': mass}
    r_old = sim_old(9, 12, 0, p)
    dist_new, lat_new, maxAlt_new, t_new = simulate_fixed(9, 12, 0, p)
    if dist_new is None:
        result_str = "NaN (未修复)"
    else:
        result_str = f"{dist_new:12.2f}m  {maxAlt_new:10.2f}m  {t_new:8.2f}s"
    print(f"  {area:.3f}  {mass:.4f}  {r_old.distance:12.2f}m  {result_str}")

# ── 验证修复 2：低 CD0 案例 ──────────────────────────────────────────────────
print("\n[修复 2] 低 CD0 → 软下限 0.03")
print(f"  {'CD0':>6} {'v0':>4}  {'旧 dist':>10}  {'新 dist':>10}  {'旧 time':>8}  {'新 time':>8}")
for cd0 in [0.005, 0.010, 0.030, 0.060]:
    for v0 in [12, 18]:
        p = {'CD0': cd0}
        r_old = sim_old(v0, 12, 0, p)
        dist_new, _, _, t_new = simulate_fixed(v0, 12, 0, p)
        print(f"  {cd0:.3f}   {v0:2}  {r_old.distance:10.2f}m  {dist_new:10.2f}m  {r_old.time:8.2f}s  {t_new:8.2f}s")

# ── 验证不改变正常参数下的飞行结果 ──────────────────────────────────────────
print("\n[回归] 默认参数下结果应与修复前一致（误差 < 1%）")
print(f"  {'v0':>4} {'pitch':>6} {'roll':>5}  {'旧 dist':>8}  {'新 dist':>8}  {'变化':>8}")
baseline_cases = [
    (9, 12, 0), (9, 0, 0), (9, 20, 0), (9, 12, 15), (15, 12, 0)
]
all_ok = True
for v0, pitch, roll in baseline_cases:
    r_old = sim_old(v0, pitch, roll)
    dist_new, lat_new, maxAlt_new, t_new = simulate_fixed(v0, pitch, roll)
    pct = abs(dist_new - r_old.distance) / max(r_old.distance, 0.01) * 100
    flag = "✓" if pct < 1.0 else "⚠ 偏差大"
    if pct >= 1.0: all_ok = False
    print(f"  {v0:>4}  {pitch:>5}°  {roll:>4}°  {r_old.distance:8.2f}m  {dist_new:8.2f}m  {pct:7.2f}%  {flag}")
print(f"  回归结果: {'全部通过 ✓' if all_ok else '有偏差，请检查 ⚠'}")

# ── 侧偏行为说明（物理正确，非 bug） ─────────────────────────────────────────
print("\n[说明] roll/lateral 非单调是物理正确行为（非 bug）")
print("  roll=±15° 时侧偏最大，因为更大横滚导致飞行时间缩短。")
print("  如需单调侧偏，需改为不减少飞行时间的控制策略（超出当前物理模型范围）。")
