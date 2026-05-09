"""
针对批量扫描发现的异常做逐步诊断
"""
import math, sys
sys.path.insert(0, '.')
from batch_sim import simulate, DEFAULTS, DEG, _CL

def trace_sim(v0, pitch_deg, roll_deg, params=None, max_frames=20):
    """逐帧打印前几步，观察数值发散过程"""
    from batch_sim import v_c, v_add, v_sub, v_scl, v_dot, v_crs, v_len, v_norm, q_rot
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

    print(f"  初始 fwd={[f'{x:.3f}' for x in fwd]}  up={[f'{x:.3f}' for x in up]}")
    print(f"  {'frame':>5}  {'pos.y':>9}  {'pos.z':>12}  {'vel.y':>9}  {'speed':>8}  {'alpha':>7}  {'CL':>7}  {'FL.y':>9}")

    from batch_sim import _CD
    frame = 0
    dt_frame = 1/60
    sdt = dt_frame / 4
    t = 0.0

    for frame in range(max_frames * 4):
        sp = v_len(vel)
        vh = v_scl(vel, 1/sp) if sp > 0.01 else fwd[:]
        cosA = max(-1, min(1, v_dot(vh, fwd)))
        alpha = math.acos(cosA)
        lat_v = v_sub(vh, v_scl(fwd, cosA))
        if v_dot(lat_v, up) > 0:
            alpha = -alpha
        CL = _CL(alpha, P)
        CD = _CD(CL, P)
        qd = 0.5 * P['rho'] * sp * sp
        if sp > 0.01:
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

        if frame % 4 == 0:
            print(f"  {frame//4:>5}  {pos[1]:>9.3f}  {pos[2]:>12.2f}  {vel[1]:>9.3f}  {sp:>8.3f}  {alpha/DEG:>7.2f}°  {CL:>7.4f}  {FL[1]:>9.3f}")

        vel = v_add(vel, v_scl(a_vec, sdt))
        pos = v_add(pos, v_scl(vel, sdt))
        t += sdt

        # 被动对齐
        tau = P['attitudeTau']
        if tau > 0.001 and sp > 0.5:
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

        if pos[1] <= 0 or any(math.isnan(x) or math.isinf(x) for x in pos + vel):
            print(f"  [terminated at frame {frame//4} substep {frame%4}]")
            break


print("=" * 70)
print("诊断 A：area/mass 极端比值导致数值爆炸")
print("=" * 70)

print("\n● 稳定案例 (area=0.005, mass=0.001, L/W≈33)")
area, mass = 0.005, 0.001
CL_approx = _CL(12*DEG, dict(DEFAULTS, area=area, mass=mass))
lw = 0.5 * 1.225 * 81 * area * CL_approx / (mass * 9.81)
print(f"  L/W ≈ {lw:.1f}  加速度 (升力-重力)/mass ≈ {(lw-1)*9.81:.1f} m/s²")
trace_sim(9, 12, 0, {'area': area, 'mass': mass}, max_frames=5)

print("\n● 异常案例 (area=0.01, mass=0.001, L/W≈67)")
area, mass = 0.01, 0.001
CL_approx = _CL(12*DEG, dict(DEFAULTS, area=area, mass=mass))
lw = 0.5 * 1.225 * 81 * area * CL_approx / (mass * 9.81)
print(f"  L/W ≈ {lw:.1f}  加速度 ≈ {(lw-1)*9.81:.1f} m/s²")
trace_sim(9, 12, 0, {'area': area, 'mass': mass}, max_frames=5)

print("\n● 爆炸案例 (area=0.02, mass=0.001, L/W≈133)")
area, mass = 0.02, 0.001
CL_approx = _CL(12*DEG, dict(DEFAULTS, area=area, mass=mass))
lw = 0.5 * 1.225 * 81 * area * CL_approx / (mass * 9.81)
print(f"  L/W ≈ {lw:.1f}  加速度 ≈ {(lw-1)*9.81:.1f} m/s²")
trace_sim(9, 12, 0, {'area': area, 'mass': mass}, max_frames=5)

print("\n")
print("=" * 70)
print("诊断 B：低 CD0 + 高 v0 → 飞太远")
print("=" * 70)
print("\n当 CD0=0.005, v0=18：")
for cd0 in [0.005, 0.01, 0.03, 0.06, 0.10]:
    r = simulate(18, 12, 0, {'CD0': cd0})
    print(f"  CD0={cd0:.3f}  dist={r.distance:.1f}m  time={r.time:.2f}s  maxAlt={r.maxAlt:.2f}m")

print("\n")
print("=" * 70)
print("诊断 C：大负 pitch → 立即落地的速度依赖性")
print("=" * 70)
print(f"  {'v0':>5}  {'pitch':>7}  {'time':>6}  {'dist':>7}  是否预期?")
for v0 in [3, 6, 9, 12, 18]:
    for pitch in [-45, -30, -20, -10]:
        r = simulate(v0, pitch, 0)
        # 分析纯弹道轨迹（如果没升力）
        # y(t) = startAlt + v0*sin(pitch)*t - 0.5*g*t^2 = 0
        # 对于纯弹道（无升力），解出 t
        vpy = v0 * math.sin(pitch * DEG)
        # t = (vpy + sqrt(vpy^2 + 2*g*startAlt)) / g
        discriminant = vpy**2 + 2*9.81*1.5
        t_ballistic = (vpy + math.sqrt(discriminant)) / 9.81 if discriminant >= 0 else 0
        expected = f"弹道={t_ballistic:.2f}s"
        print(f"  {v0:>5}  {pitch:>7}°  {r.time:>6.2f}s  {r.distance:>7.2f}m  {expected}")

print("\n")
print("=" * 70)
print("诊断 D：roll/lateral 非单调分析（峰值在 ±15°）")
print("=" * 70)
print("  横滚与侧偏的关系（v0=9, pitch=12°）")
print(f"  {'roll':>6}  {'lateral':>9}  {'dist':>8}  {'time':>6}  {'vert_lift%':>10}")
for roll in range(-30, 35, 5):
    r = simulate(9, 12, roll)
    # 侧偏 / 距离 比值代表转弯效率
    ratio = abs(r.lateral) / r.distance if r.distance > 0.1 else 0
    print(f"  {roll:>6}°  {r.lateral:>9.3f}m  {r.distance:>8.2f}m  {r.time:>6.2f}s  {ratio:>9.3f}")

print("\n")
print("=" * 70)
print("诊断 E：验证 attitudeTau=5.0 的 phugoid 振荡")
print("=" * 70)
print("  (v0=9, pitch=12°, tau=5.0)")
r_tau5 = simulate(9, 12, 0, {'attitudeTau': 5.0})
ys = [p[1] for p in r_tau5.traj]
peaks_idx = [i for i in range(1, len(ys)-1) if ys[i] > ys[i-1] and ys[i] > ys[i+1]]
print(f"  轨迹点数: {len(r_tau5.traj)}")
print(f"  高度极大值索引: {peaks_idx}")
for i in peaks_idx:
    print(f"    traj[{i}]: y={ys[i]:.3f}m  z={r_tau5.traj[i][2]:.3f}m")

print("\n")
print("=" * 70)
print("诊断 F：高 v0 下升力-速度正反馈的临界条件")
print("=" * 70)
print("  判断数值稳定的约束：单步速度变化 << 当前速度")
print(f"  {'area':>7} {'mass':>7} {'L/W':>7}  {'Δv_per_substep':>15}  {'v0比':>8}  {'稳定?':>6}")
for area in [0.005, 0.01, 0.02, 0.04]:
    for mass in [0.001, 0.003, 0.005]:
        P = dict(DEFAULTS, area=area, mass=mass)
        CL = _CL(12*DEG, P)
        CD = P['CD0'] + P['k'] * CL**2
        sp = 9.0
        qd = 0.5 * P['rho'] * sp**2
        FL_mag = qd * area * CL
        FD_mag = qd * area * CD
        FG_mag = mass * 9.81
        net_vert_accel = (FL_mag - FG_mag) / mass  # 净垂直加速度
        sdt = (1/60) / 4
        dv = abs(net_vert_accel) * sdt
        ratio = dv / sp
        lw = FL_mag / FG_mag
        stable = "✓" if ratio < 0.5 else "⚠ 不稳"
        print(f"  {area:.3f}  {mass:.4f}  {lw:>6.1f}  {dv:>14.3f} m/s  {ratio:>7.3f}  {stable}")
