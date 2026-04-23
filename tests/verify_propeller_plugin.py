#!/usr/bin/env python3
"""Verify the PropellerPerformancePlugin thrust calculation chain.

This script traces the entire signal path from ArduPilot PWM through to the
thrust applied by the PropellerPerformancePlugin, checking whether the hovering
thrust matches physics expectations.

Usage:
  python3 tests/verify_propeller_plugin.py
"""

import csv
import math
import sys
import os

# --------------------------------------------------------------------------- #
#  Constants
# --------------------------------------------------------------------------- #

CSV_PATH = os.path.join(
    os.path.dirname(__file__), "..", "models", "waterdrop", "propellers", "PER3_7x11E.csv"
)

# Vehicle parameters from model.sdf
MASS_KG    = 4.3 + 4 * 0.025 + 0.15   # base_link + 4 motor links + imu link
GRAVITY    = 9.81
NUM_MOTORS = 4

# ArduPilotPlugin control parameters from model.sdf (motor1 CCW example)
SERVO_MIN    = 1000
SERVO_MAX    = 2000
MULTIPLIER   = 2200   # rad/s velocity target scale  (CCW: +2200, CW: -2200)
OFFSET       = 0.0
# rotorVelocitySlowdownSim defaults to 1 (not set in SDF)
ROTOR_VELOCITY_SLOWDOWN_SIM = 1.0

# Physics
RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * math.pi)


# --------------------------------------------------------------------------- #
#  Load CSV table
# --------------------------------------------------------------------------- #

def load_perf_table(path):
    """Return list of (rpm, v_ms, thrust_N, torque_Nm) tuples."""
    table = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            table.append((
                float(row["rpm"]),
                float(row["v_ms"]),
                float(row["thrust_N"]),
                float(row["torque_Nm"]),
            ))
    return table

# --------------------------------------------------------------------------- #
#  1-D linear interpolation (match plugin Lerp1D)
# --------------------------------------------------------------------------- #

def lerp1d(xs, ys, x):
    if not xs:
        return 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    # find bracket
    for i in range(1, len(xs)):
        if xs[i] >= x:
            lo, hi = i - 1, i
            t = (x - xs[lo]) / (xs[hi] - xs[lo])
            return ys[lo] + t * (ys[hi] - ys[lo])
    return ys[-1]

# --------------------------------------------------------------------------- #
#  Build the grouped PerfTable (match plugin PerfTable)
# --------------------------------------------------------------------------- #

def build_perf_table(rows):
    """Group rows by RPM, return (rpms, speeds[], thrust[], torque[])."""
    rpms, speeds, thrust, torque = [], [], [], []
    current_rpm = None
    for rpm, v, t, q in sorted(rows, key=lambda r: (r[0], r[1])):
        if rpm != current_rpm:
            rpms.append(rpm)
            speeds.append([])
            thrust.append([])
            torque.append([])
            current_rpm = rpm
        speeds[-1].append(v)
        thrust[-1].append(t)
        torque[-1].append(q)
    return rpms, speeds, thrust, torque


def perf_table_at(rpms, speeds, thrust, torque, rpm, v):
    """Bilinear lookup matching the C++ PerfTable::At()."""
    if not rpms:
        return 0.0, 0.0

    if rpm <= rpms[0]:
        lo_idx = hi_idx = 0
        t = 0.0
    elif rpm >= rpms[-1]:
        lo_idx = hi_idx = len(rpms) - 1
        t = 0.0
    else:
        import bisect
        hi_idx = bisect.bisect_right(rpms, rpm)
        lo_idx = hi_idx - 1
        t = (rpm - rpms[lo_idx]) / (rpms[hi_idx] - rpms[lo_idx])

    thr_lo = lerp1d(speeds[lo_idx], thrust[lo_idx], v)
    tor_lo = lerp1d(speeds[lo_idx], torque[lo_idx], v)
    if lo_idx == hi_idx:
        return thr_lo, tor_lo
    thr_hi = lerp1d(speeds[hi_idx], thrust[hi_idx], v)
    tor_hi = lerp1d(speeds[hi_idx], torque[hi_idx], v)
    return thr_lo + t * (thr_hi - thr_lo), tor_lo + t * (tor_hi - tor_lo)


# --------------------------------------------------------------------------- #
#  PWM → cmd → velocity target → RPM (match ArduPilotPlugin)
# --------------------------------------------------------------------------- #

def pwm_to_velocity_target(pwm, servo_min, servo_max, multiplier, offset,
                            rotor_velocity_slowdown_sim):
    """
    ArduPilotPlugin::UpdateMotorCommands → ApplyMotorForces:
      raw_cmd = clamp((pwm - servo_min) / (servo_max - servo_min), 0, 1)
      cmd     = multiplier * (raw_cmd + offset)
      velTarget = cmd / rotorVelocitySlowdownSim    (rad/s)
    """
    raw_cmd = max(0.0, min(1.0, (pwm - servo_min) / (servo_max - servo_min)))
    cmd = multiplier * (raw_cmd + offset)
    vel_target = cmd / rotor_velocity_slowdown_sim
    return raw_cmd, cmd, vel_target


# --------------------------------------------------------------------------- #
#  Main analysis
# --------------------------------------------------------------------------- #

def main():
    print("=" * 72)
    print("PropellerPerformancePlugin Hover Thrust Verification")
    print("=" * 72)

    # Load table
    rows = load_perf_table(CSV_PATH)
    rpms_tbl, speeds_tbl, thrust_tbl, torque_tbl = build_perf_table(rows)
    print(f"\nLoaded {len(rows)} data points from {os.path.basename(CSV_PATH)}")
    print(f"  RPM range : {rpms_tbl[0]:.0f} – {rpms_tbl[-1]:.0f}")
    print(f"  RPM steps : {len(rpms_tbl)}")

    # Required hover thrust
    weight = MASS_KG * GRAVITY
    thrust_per_motor = weight / NUM_MOTORS
    print(f"\n--- Vehicle ---")
    print(f"  Total mass       : {MASS_KG:.3f} kg")
    print(f"  Weight           : {weight:.3f} N")
    print(f"  Thrust per motor : {thrust_per_motor:.3f} N")

    # Find the RPM that gives the required static (v=0) thrust
    # from the performance table via bisection
    rpm_lo, rpm_hi = rpms_tbl[0], rpms_tbl[-1]
    for _ in range(100):
        rpm_mid = (rpm_lo + rpm_hi) / 2.0
        t, _ = perf_table_at(rpms_tbl, speeds_tbl, thrust_tbl, torque_tbl,
                             rpm_mid, 0.0)
        if t < thrust_per_motor:
            rpm_lo = rpm_mid
        else:
            rpm_hi = rpm_mid
    hover_rpm = (rpm_lo + rpm_hi) / 2.0
    hover_thrust, hover_torque = perf_table_at(
        rpms_tbl, speeds_tbl, thrust_tbl, torque_tbl, hover_rpm, 0.0)

    print(f"\n--- Required hover RPM (from table, v=0 m/s) ---")
    print(f"  Hover RPM        : {hover_rpm:.1f}")
    print(f"  Thrust at RPM    : {hover_thrust:.4f} N")
    print(f"  Torque at RPM    : {hover_torque:.4f} Nm")

    # Reverse the ArduPilot velocity pipeline to find the PWM needed
    # velTarget = cmd / slowdown = (multiplier * raw_cmd) / slowdown
    # RPM = |velTarget| * 60 / (2*pi)
    # =>  |velTarget| = hover_rpm / RAD_PER_SEC_TO_RPM
    hover_omega = hover_rpm / RAD_PER_SEC_TO_RPM   # rad/s
    raw_cmd_needed = hover_omega * ROTOR_VELOCITY_SLOWDOWN_SIM / abs(MULTIPLIER)
    pwm_needed = raw_cmd_needed * (SERVO_MAX - SERVO_MIN) + SERVO_MIN

    print(f"\n--- Reverse pipeline: RPM → PWM ---")
    print(f"  ω hover          : {hover_omega:.2f} rad/s")
    print(f"  raw_cmd needed   : {raw_cmd_needed:.4f}")
    print(f"  PWM needed       : {pwm_needed:.1f}")
    if raw_cmd_needed > 1.0:
        print(f"  ⚠ raw_cmd > 1.0  : MOTOR SATURATED — cannot reach hover RPM!")
    if pwm_needed > SERVO_MAX:
        print(f"  ⚠ PWM > servo_max: UNREACHABLE with current SDF settings!")

    # Show full range of RPM achievable
    max_raw_cmd = 1.0
    max_cmd = abs(MULTIPLIER) * (max_raw_cmd + OFFSET)
    max_vel_target = max_cmd / ROTOR_VELOCITY_SLOWDOWN_SIM
    max_rpm = abs(max_vel_target) * RAD_PER_SEC_TO_RPM
    max_thrust, _ = perf_table_at(
        rpms_tbl, speeds_tbl, thrust_tbl, torque_tbl, max_rpm, 0.0)

    print(f"\n--- Maximum achievable (PWM={SERVO_MAX}, raw_cmd=1.0) ---")
    print(f"  cmd              : {max_cmd:.2f} rad/s")
    print(f"  velTarget        : {max_vel_target:.2f} rad/s")
    print(f"  Max RPM          : {max_rpm:.1f}")
    print(f"  Max static thrust: {max_thrust:.3f} N  (per motor)")
    print(f"  Max total thrust : {4 * max_thrust:.3f} N")
    print(f"  Weight           : {weight:.3f} N")
    if 4 * max_thrust < weight:
        print(f"  ⚠ Total max thrust < weight! Aircraft CANNOT hover!")
    else:
        print(f"  ✓ Thrust-to-weight ratio: {4 * max_thrust / weight:.2f}")

    # Now show some representative PWM points
    print(f"\n--- Thrust at representative PWM values ---")
    print(f"  {'PWM':>6s}  {'raw_cmd':>8s}  {'cmd(rad/s)':>10s}  "
          f"{'RPM':>8s}  {'Thrust(N)':>10s}  {'4xThrust':>10s}  "
          f"{'vs Weight':>10s}")
    print(f"  {'------':>6s}  {'--------':>8s}  {'----------':>10s}  "
          f"{'--------':>8s}  {'----------':>10s}  {'----------':>10s}  "
          f"{'----------':>10s}")
    for pwm in [1100, 1200, 1300, 1400, 1500, 1550, 1600, 1700, 1800, 1900, 2000]:
        raw, cmd, vel = pwm_to_velocity_target(
            pwm, SERVO_MIN, SERVO_MAX, MULTIPLIER, OFFSET,
            ROTOR_VELOCITY_SLOWDOWN_SIM)
        rpm_val = abs(vel) * RAD_PER_SEC_TO_RPM
        t_per_motor, _ = perf_table_at(
            rpms_tbl, speeds_tbl, thrust_tbl, torque_tbl, rpm_val, 0.0)
        total_t = 4 * t_per_motor
        marker = " ← hover" if abs(total_t - weight) / weight < 0.05 else ""
        print(f"  {pwm:6.0f}  {raw:8.4f}  {cmd:10.2f}  "
              f"{rpm_val:8.1f}  {t_per_motor:10.4f}  {total_t:10.3f}  "
              f"{total_t - weight:+10.3f}{marker}")

    # Specific: what is Q_M_THST_HOVER?
    # Q_M_THST_HOVER = 0.6  (from waterdrop.parm)
    # This means ArduPilot expects hover at 60% throttle
    # 60% throttle → PWM = servo_min + 0.6 * (servo_max - servo_min)
    thst_hover = 0.7  # from waterdrop.parm (updated)
    hover_pwm_ap = SERVO_MIN + thst_hover * (SERVO_MAX - SERVO_MIN)
    raw_ap, cmd_ap, vel_ap = pwm_to_velocity_target(
        hover_pwm_ap, SERVO_MIN, SERVO_MAX, MULTIPLIER, OFFSET,
        ROTOR_VELOCITY_SLOWDOWN_SIM)
    rpm_ap = abs(vel_ap) * RAD_PER_SEC_TO_RPM
    t_ap, _ = perf_table_at(
        rpms_tbl, speeds_tbl, thrust_tbl, torque_tbl, rpm_ap, 0.0)

    print(f"\n--- ArduPilot hover estimate (Q_M_THST_HOVER={thst_hover}) ---")
    print(f"  Expected PWM     : {hover_pwm_ap:.0f}")
    print(f"  raw_cmd          : {raw_ap:.4f}")
    print(f"  cmd              : {cmd_ap:.2f} rad/s")
    print(f"  RPM              : {rpm_ap:.1f}")
    print(f"  Thrust per motor : {t_ap:.4f} N")
    print(f"  Total thrust     : {4 * t_ap:.3f} N")
    print(f"  Weight           : {weight:.3f} N")
    print(f"  Mismatch         : {4 * t_ap - weight:+.3f} N "
          f"({(4 * t_ap - weight) / weight * 100:+.1f}%)")

    # ---------- Summary Diagnostic ----------
    print(f"\n{'=' * 72}")
    print("DIAGNOSTICS SUMMARY")
    print(f"{'=' * 72}")

    issues = []

    # Check 1: Can max RPM produce enough thrust?
    if 4 * max_thrust < weight:
        issues.append(
            f"CRITICAL: Max achievable RPM is {max_rpm:.0f} → max total "
            f"thrust {4*max_thrust:.2f} N < weight {weight:.2f} N. "
            f"The multiplier ({MULTIPLIER}) is too low."
        )

    # Check 2: Is the CSV RPM range sufficient?
    if hover_rpm > rpms_tbl[-1]:
        issues.append(
            f"WARNING: Required hover RPM ({hover_rpm:.0f}) exceeds CSV "
            f"maximum ({rpms_tbl[-1]:.0f}). Table will clamp to max row."
        )

    # Check 3: Are multiplier/servo settings putting RPM in the right range?
    if max_rpm > rpms_tbl[-1] * 1.5:
        issues.append(
            f"WARNING: Max RPM ({max_rpm:.0f}) far exceeds CSV max "
            f"({rpms_tbl[-1]:.0f}). The plugin will clamp, producing flat "
            f"thrust at high throttle."
        )

    # Check 4: Does the hover throttle produce the right thrust?
    if abs(4 * t_ap - weight) / weight > 0.1:
        issues.append(
            f"MISMATCH: At Q_M_THST_HOVER={thst_hover} (PWM {hover_pwm_ap:.0f}), "
            f"total thrust = {4*t_ap:.2f} N but weight = {weight:.2f} N "
            f"(error {(4*t_ap-weight)/weight*100:+.1f}%)."
        )

    if not issues:
        print("  ✓ All checks passed — thrust chain looks correct.")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")

    print()
    return 0 if not issues else 1


if __name__ == "__main__":
    raise SystemExit(main())
