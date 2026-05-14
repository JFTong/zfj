"""
AeroThrow 3D — 批量参数扫描 & 轨迹异常检测
把 prototype.html §3.3 Sim 类完整移植到 Python，逐一改变参数，
统计结果并标注异常。
"""
import math
import itertools
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import sys

# ─── Vec3 ──────────────────────────────────────────────────────────────────────
def v_c(x=0.0, y=0.0, z=0.0): return [x, y, z]
def v_add(a, b): return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
def v_sub(a, b): return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
def v_scl(a, s): return [a[0]*s, a[1]*s, a[2]*s]
def v_dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def v_crs(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def v_len(a): return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
def v_norm(a):
    l = v_len(a)
    return v_scl(a, 1/l) if l > 1e-9 else v_c()

# ─── Quaternion ─────────────────────────────────────────────────────────────────
def q_rot(q, v):
    u = [q[0], q[1], q[2]]
    t = v_scl(v_crs(u, v), 2)
    return v_add(v_add(v, v_scl(t, q[3])), v_crs(u, t))

DEG = math.pi / 180

# ─── 物理参数默认值 ──────────────────────────────────────────────────────────────
DEFAULTS = dict(
    mass=0.005, area=0.02, rho=1.225, g=9.81,
    CD0=0.06, k=0.10, alphaStall=12.0, CLpostStall=0.6,
    maxSubsteps=4, startAlt=1.5, attitudeTau=0.3,
)

# ─── 单次仿真 ────────────────────────────────────────────────────────────────────
@dataclass
class SimResult:
    distance: float      # -p.z (m)
    lateral: float       # p.x (m)
    maxAlt: float        # (m)
    time: float          # (s)
    traj: list           # [(x,y,z), ...]
    anomaly: str = ""    # 异常描述，空则正常

def _CL(alpha_rad, P):
    aDeg = abs(alpha_rad) / DEG
    sgn = math.copysign(1, alpha_rad) if alpha_rad != 0 else 1
    peak = 2 * math.pi * (P['alphaStall'] * DEG)
    if aDeg <= P['alphaStall']:
        return 2 * math.pi * alpha_rad
    if aDeg <= P['alphaStall'] + 6:
        t = (aDeg - P['alphaStall']) / 6
        return sgn * (peak * (1-t) + P['CLpostStall'] * t)
    return sgn * P['CLpostStall']

def _CD(CL, P):
    return P['CD0'] + P['k'] * CL * CL

def simulate(v0, pitch_deg, roll_deg, params=None, max_t=30.0) -> SimResult:
    P = dict(DEFAULTS)
    if params:
        P.update(params)

    p_deg = pitch_deg * DEG
    r_deg = roll_deg * DEG

    # 初始姿态（与 JS 一致）
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

    frames = 0
    nan_hit = False

    while True:
        # 安全超时
        if t_total >= max_t:
            break
        dt = dt_frame
        subs = min(P['maxSubsteps'], max(1, math.ceil(dt / (1/120))))
        sdt = dt / subs

        for _ in range(subs):
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

            vel = v_add(vel, v_scl(a_vec, sdt))
            pos = v_add(pos, v_scl(vel, sdt))
            t_total += sdt

            # NaN 检测
            if any(math.isnan(x) or math.isinf(x) for x in pos + vel):
                nan_hit = True
                break

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

            # 落地
            if pos[1] <= 0:
                pos[1] = 0
                traj.append(tuple(pos))
                break

        if nan_hit:
            break

        if pos[1] > maxAlt:
            maxAlt = pos[1]

        last = traj[-1]
        dx = pos[0]-last[0]; dy = pos[1]-last[1]; dz = pos[2]-last[2]
        if dx*dx + dy*dy + dz*dz > 0.04:
            traj.append(tuple(pos))

        if pos[1] <= 0:
            break

    r = SimResult(
        distance=-pos[2],
        lateral=pos[0],
        maxAlt=maxAlt,
        time=t_total,
        traj=traj,
    )
    return r

# ─── 异常判断 ─────────────────────────────────────────────────────────────────────
def check_anomaly(r: SimResult, v0, pitch, roll, params) -> List[str]:
    issues = []
    P = dict(DEFAULTS)
    if params: P.update(params)

    # NaN / Inf
    if any(math.isnan(x) or math.isinf(x) for x in [r.distance, r.lateral, r.maxAlt, r.time]):
        issues.append("NaN/Inf")
        return issues

    # 飞行时间超时
    if r.time >= 29.9:
        issues.append("超时未落地(≥30s)")

    # 飞机向后飞 (distance < 0)
    if r.distance < -0.1:
        issues.append(f"向后飞 dist={r.distance:.2f}m")

    # 几乎不动 (初速 >3 m/s 但 distance < 0.5m)
    if v0 > 3 and r.distance < 0.5:
        issues.append(f"几乎不动 dist={r.distance:.2f}m")

    # 异常高度（对于纸飞机，最高超过 20m 很可疑）
    if r.maxAlt > 20:
        issues.append(f"极端高度 maxAlt={r.maxAlt:.1f}m")

    # 侧偏方向与横滚方向相反（roll>0 应右偏 lateral>0，反之相反）
    if abs(roll) > 5 and abs(r.lateral) > 0.3:
        expected_sign = 1 if roll > 0 else -1
        if math.copysign(1, r.lateral) != expected_sign:
            issues.append(f"侧偏方向反 roll={roll}° lateral={r.lateral:.2f}m")

    # 很大的侧偏但没有横滚
    if abs(roll) < 1 and abs(r.lateral) > 2:
        issues.append(f"无横滚但大侧偏 lateral={r.lateral:.2f}m")

    # 异常远 (>40m 对纸飞机不现实)
    if r.distance > 40:
        issues.append(f"异常远 dist={r.distance:.1f}m")

    # 立即落地 (飞行时间极短 < 0.5s 且初速 > 3 m/s)
    if v0 > 3 and r.time < 0.5:
        issues.append(f"立即落地 t={r.time:.2f}s")

    return issues

# ─── 批量扫描 ────────────────────────────────────────────────────────────────────
def run_sweep():
    print("=" * 70)
    print("AeroThrow 3D · 批量参数扫描")
    print("=" * 70)

    all_results = []

    # ── 扫描 1：v0 × pitch（基准参数）────────────────────────────────────────────
    print("\n[扫描 1] v0 × pitch  (roll=0, 默认物理参数)")
    v0_range     = [3, 6, 9, 12, 15, 18]
    pitch_range  = [-30, -20, -10, 0, 10, 20, 30, 45]
    for v0, pitch in itertools.product(v0_range, pitch_range):
        r = simulate(v0, pitch, 0)
        issues = check_anomaly(r, v0, pitch, 0, None)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, v0, pitch, 0, None, r))
        if issues:
            print(f"  v0={v0:4.0f} pitch={pitch:4.0f}°  "
                  f"dist={r.distance:6.2f}m alt={r.maxAlt:5.2f}m t={r.time:5.2f}s  {tag}")

    # ── 扫描 2：v0 × roll（基准参数，pitch=12°）─────────────────────────────────
    print("\n[扫描 2] v0 × roll  (pitch=12°, 默认物理参数)")
    roll_range = [-30, -20, -10, -5, 0, 5, 10, 20, 30]
    for v0, roll in itertools.product(v0_range, roll_range):
        r = simulate(v0, 12, roll)
        issues = check_anomaly(r, v0, 12, roll, None)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, v0, 12, roll, None, r))
        if issues:
            print(f"  v0={v0:4.0f} roll={roll:4.0f}°  "
                  f"dist={r.distance:6.2f}m lat={r.lateral:6.2f}m t={r.time:5.2f}s  {tag}")

    # ── 扫描 3：attitudeTau × pitch（轨迹振荡相关）──────────────────────────────
    print("\n[扫描 3] attitudeTau × pitch  (v0=9, roll=0)")
    tau_range = [0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 5.0]
    for tau, pitch in itertools.product(tau_range, [-20, -10, 0, 10, 20, 30]):
        p = {'attitudeTau': tau}
        r = simulate(9, pitch, 0, p)
        issues = check_anomaly(r, 9, pitch, 0, p)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, 9, pitch, 0, p, r))
        if issues:
            print(f"  tau={tau:.2f} pitch={pitch:4.0f}°  "
                  f"dist={r.distance:6.2f}m alt={r.maxAlt:5.2f}m t={r.time:5.2f}s  {tag}")

    # ── 扫描 4：area × mass（升重比）────────────────────────────────────────────
    print("\n[扫描 4] area × mass  (v0=9, pitch=12, roll=0)")
    area_range = [0.005, 0.01, 0.02, 0.04, 0.06, 0.10]
    mass_range = [0.001, 0.003, 0.005, 0.01, 0.02]
    for area, mass in itertools.product(area_range, mass_range):
        p = {'area': area, 'mass': mass}
        r = simulate(9, 12, 0, p)
        issues = check_anomaly(r, 9, 12, 0, p)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, 9, 12, 0, p, r))
        if issues:
            print(f"  area={area:.3f} mass={mass:.4f}  "
                  f"dist={r.distance:6.2f}m alt={r.maxAlt:5.2f}m t={r.time:5.2f}s  {tag}")

    # ── 扫描 5：alphaStall × attitudeTau（失速 + 振荡相互作用）────────────────
    print("\n[扫描 5] alphaStall × attitudeTau  (v0=9, pitch=20, roll=0)")
    stall_range = [6, 8, 10, 12, 15, 20, 25]
    tau_range2  = [0.05, 0.1, 0.3, 0.5, 1.0, 3.0]
    for stall, tau in itertools.product(stall_range, tau_range2):
        p = {'alphaStall': stall, 'attitudeTau': tau}
        r = simulate(9, 20, 0, p)
        issues = check_anomaly(r, 9, 20, 0, p)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, 9, 20, 0, p, r))
        if issues:
            print(f"  stall={stall:3.0f}° tau={tau:.2f}  "
                  f"dist={r.distance:6.2f}m alt={r.maxAlt:5.2f}m t={r.time:5.2f}s  {tag}")

    # ── 扫描 6：CD0 × v0（阻力对距离的影响）────────────────────────────────────
    print("\n[扫描 6] CD0 × v0  (pitch=12, roll=0)")
    cd0_range = [0.005, 0.01, 0.02, 0.04, 0.06, 0.10, 0.15, 0.20]
    for cd0, v0 in itertools.product(cd0_range, v0_range):
        p = {'CD0': cd0}
        r = simulate(v0, 12, 0, p)
        issues = check_anomaly(r, v0, 12, 0, p)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, v0, 12, 0, p, r))
        if issues:
            print(f"  CD0={cd0:.3f} v0={v0:4.0f}  "
                  f"dist={r.distance:6.2f}m alt={r.maxAlt:5.2f}m t={r.time:5.2f}s  {tag}")

    # ── 扫描 7：极端 pitch（正负大角度）────────────────────────────────────────
    print("\n[扫描 7] 极端 pitch & v0  (roll=0)")
    for v0, pitch in itertools.product([3, 9, 18], [-45, -30, 45, 60]):
        r = simulate(v0, pitch, 0)
        issues = check_anomaly(r, v0, pitch, 0, None)
        tag = "❌ " + "; ".join(issues) if issues else "  ok"
        all_results.append((tag, v0, pitch, 0, None, r))
        print(f"  v0={v0:4.0f} pitch={pitch:4.0f}°  "
              f"dist={r.distance:6.2f}m alt={r.maxAlt:5.2f}m t={r.time:5.2f}s  {tag}")

    # ─── 总结 ──────────────────────────────────────────────────────────────────
    anomaly_list = [(tag, v0, pitch, roll, p, r) for tag, v0, pitch, roll, p, r in all_results if tag.startswith("❌")]
    print(f"\n{'='*70}")
    print(f"总计: {len(all_results)} 次仿真，{len(anomaly_list)} 个异常")
    print('='*70)

    # 按异常类型分类
    categories = {}
    for tag, v0, pitch, roll, p, r in anomaly_list:
        key = tag[2:].split(';')[0].strip().split(' ')[0]  # 取第一个异常词
        categories.setdefault(key, []).append((tag, v0, pitch, roll, p, r))
    for cat, cases in sorted(categories.items(), key=lambda x: -len(x[1])):
        print(f"\n[{cat}]  共 {len(cases)} 个:")
        for tag, v0, pitch, roll, p, r in cases[:6]:
            extra = ""
            if p:
                kv = " ".join(f"{k}={v}" for k, v in p.items())
                extra = f" params=({kv})"
            print(f"    v0={v0} pitch={pitch} roll={roll}{extra}")
            print(f"      dist={r.distance:.2f}m  lat={r.lateral:.2f}m  maxAlt={r.maxAlt:.2f}m  t={r.time:.2f}s")
        if len(cases) > 6:
            print(f"    ... 及另外 {len(cases)-6} 个")

    return anomaly_list

# ─── 深度分析：phugoid 振荡 ──────────────────────────────────────────────────────
def analyze_phugoid():
    """检查不同 tau 下的高度振荡幅度"""
    print(f"\n{'='*70}")
    print("深度分析：高度振荡（phugoid）随 attitudeTau 变化")
    print('='*70)
    print(f"{'tau':>6} {'pitch':>6} {'maxAlt':>8} {'dist':>8} {'t':>6}  {'振荡特征'}")
    for tau in [0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1.5, 3.0, 5.0]:
        r = simulate(9, 12, 0, {'attitudeTau': tau})
        # 粗略判断振荡：从轨迹的 y 变化分析
        ys = [p[1] for p in r.traj]
        # 找局部极大值个数
        peaks = sum(1 for i in range(1, len(ys)-1) if ys[i] > ys[i-1] and ys[i] > ys[i+1])
        note = f"振荡峰 {peaks} 个" if peaks > 1 else "单峰/单调"
        print(f"  {tau:5.2f}  {12:5}° {r.maxAlt:8.2f}  {r.distance:8.2f}  {r.time:6.2f}  {note}")

# ─── 深度分析：横滚→侧偏线性度 ─────────────────────────────────────────────────
def analyze_roll_lateral():
    """检验侧偏量是否随 roll 单调且方向一致"""
    print(f"\n{'='*70}")
    print("深度分析：横滚→侧偏线性度  (v0=9, pitch=12°)")
    print('='*70)
    print(f"{'roll':>6}  {'lateral':>8}  {'dist':>8}  符号正确?")
    prev_lat = 0
    monotone = True
    for roll in range(-30, 31, 5):
        r = simulate(9, 12, roll)
        sign_ok = (roll == 0) or (roll > 0 and r.lateral >= 0) or (roll < 0 and r.lateral <= 0)
        if roll != -30 and not (r.lateral > prev_lat - 0.1):
            monotone = False
        flag = "" if sign_ok else "  ⚠ 方向反!"
        print(f"  {roll:5}°  {r.lateral:8.3f}m  {r.distance:8.2f}m  {'✓' if sign_ok else '✗'}{flag}")
        prev_lat = r.lateral
    print(f"  侧偏随横滚单调递增: {'✓' if monotone else '⚠ 局部不单调'}")

# ─── 深度分析：升重比边界 ─────────────────────────────────────────────────────────
def analyze_lift_weight():
    """分析 area/mass 临界点：何时飞机无法飞行 vs. 飞太高"""
    print(f"\n{'='*70}")
    print("深度分析：升重比边界  (v0=9, pitch=12°, roll=0°)")
    print('='*70)
    print(f"{'area':>7} {'mass':>7} {'L/W':>6}  {'dist':>8} {'maxAlt':>7}  状态")
    for area in [0.005, 0.01, 0.02, 0.04, 0.08]:
        for mass in [0.001, 0.003, 0.005, 0.01, 0.02]:
            p = {'area': area, 'mass': mass}
            r = simulate(9, 12, 0, p)
            # 近似升重比：使用 v0=9, alpha=12° 时的 CL
            CL_approx = _CL(12*DEG, dict(DEFAULTS, **p))
            liftApprox = 0.5 * 1.225 * 81 * area * CL_approx
            weightApprox = mass * 9.81
            lw = liftApprox / weightApprox
            state = "正常"
            if r.time >= 29.9: state = "超时(困在空中)"
            elif r.maxAlt > 20: state = f"超高({r.maxAlt:.1f}m)"
            elif r.time < 0.3: state = "即落地"
            elif r.distance < 1: state = "几乎不飞"
            print(f"  {area:.3f}  {mass:.4f}  {lw:6.2f}  {r.distance:8.2f} {r.maxAlt:7.2f}  {state}")

if __name__ == '__main__':
    anomaly_list = run_sweep()
    analyze_phugoid()
    analyze_roll_lateral()
    analyze_lift_weight()
    print(f"\n✓ 分析完成")
