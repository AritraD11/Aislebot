# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║       AISLEBOT — PID TELEMETRY ANALYSIS  (v2 — corrected)                  ║
# ║       IIT Bombay · Aritra Das                                               ║
# ║                                                                             ║
# ║  CSV FORMAT THIS NOTEBOOK EXPECTS:                                          ║
# ║    pi_time_s, FR_target_rads, FR_actual_rads, FR_pwm,                      ║
# ║    FL_target_rads, FL_actual_rads, FL_pwm,                                 ║
# ║    RR_target_rads, RR_actual_rads, RR_pwm,                                 ║
# ║    RL_target_rads, RL_actual_rads, RL_pwm                                  ║
# ║    (13 columns, header row present, error computed here)                    ║
# ╚══════════════════════════════════════════════════════════════════════════════╝


# ─────────────────────────────────────────────────────────────────────────────
# CELL 1 ── Imports & Config
# ─────────────────────────────────────────────────────────────────────────────

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from google.colab import files
import io, warnings
warnings.filterwarnings('ignore')

# Motor identity
MOTORS  = ['FR', 'FL', 'RR', 'RL']
COLORS  = {'FR': '#3b82f6', 'FL': '#10b981', 'RR': '#f59e0b', 'RL': '#ef4444'}
PWM_SAT      = 230     # |PWM| above this = saturated
SETTLE_BAND  = 0.05    # ±5% of setpoint counts as settled
SETTLE_MIN_STEP = 0.1  # rad/s — ignore tiny setpoint changes

plt.rcParams.update({
    'figure.dpi': 130,
    'font.family': 'monospace',
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': True,
    'grid.alpha': 0.22,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'legend.fontsize': 9,
})

print("✓  Cell 1 done. Run Cell 2 to upload your CSV.")


# ─────────────────────────────────────────────────────────────────────────────
# CELL 2 ── Upload & Parse
# ─────────────────────────────────────────────────────────────────────────────

uploaded = files.upload()
fname    = list(uploaded.keys())[0]
raw      = pd.read_csv(io.BytesIO(uploaded[fname]))

# Force all non-time columns to numeric (some may come in as strings)
for c in raw.columns:
    if c != 'pi_time_s':
        raw[c] = pd.to_numeric(raw[c], errors='coerce')
raw['pi_time_s'] = pd.to_numeric(raw['pi_time_s'], errors='coerce')

# Compute error (not logged — derived here)
for m in MOTORS:
    raw[f'{m}_error'] = raw[f'{m}_actual_rads'] - raw[f'{m}_target_rads']

# Normalise time to start at 0
raw['time'] = raw['pi_time_s'] - raw['pi_time_s'].iloc[0]
raw.dropna(subset=['time'], inplace=True)
raw.reset_index(drop=True, inplace=True)

df = raw.copy()
T  = df['time'].values
total_dur = T[-1] - T[0]
fs = len(df) / total_dur if total_dur > 0 else 10.0

print(f"✓  Loaded  '{fname}'")
print(f"   Samples  : {len(df)}")
print(f"   Duration : {total_dur:.2f} s")
print(f"   Sample rate ≈ {fs:.1f} Hz")
print()
print(f"  {'Motor':<6} {'RMS Error':>10}  {'PWM sat%':>9}  {'Target mean':>12}  {'Actual mean':>12}")
print(f"  {'──────':<6} {'─────────':>10}  {'────────':>9}  {'───────────':>12}  {'───────────':>12}")
for m in MOTORS:
    rms  = np.sqrt(np.nanmean(df[f'{m}_error'].values**2))
    sat  = (np.abs(df[f'{m}_pwm'].values) >= PWM_SAT).mean() * 100
    tmn  = df[f'{m}_target_rads'].mean()
    amn  = df[f'{m}_actual_rads'].mean()
    print(f"  {m:<6} {rms:>10.3f}  {sat:>8.1f}%  {tmn:>12.3f}  {amn:>12.3f}")

print("\n✓  Data ready. Run Cells 3, 4, 5 for the three plots.")


# ─────────────────────────────────────────────────────────────────────────────
# CELL 3 ── PLOT 1 : Speed Tracking
# ─────────────────────────────────────────────────────────────────────────────
# WHAT TO LOOK FOR:
#   Gap between dashed target and solid actual → PID can't reach setpoint
#   Oscillation around target → Kp/Kd imbalance or encoder noise
#   Actual near zero despite nonzero target → motor deadband or E-STOP latch
#   One motor tracking well while others don't → individual gain tuning needed

fig1, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
fig1.suptitle('AISLEBOT  ·  PLOT 1 — Speed Tracking  (target vs actual)',
              fontweight='bold', fontsize=13)

for ax, m in zip(axes.flat, MOTORS):
    col = COLORS[m]
    tgt = df[f'{m}_target_rads'].values
    act = df[f'{m}_actual_rads'].values
    err = act - tgt

    # ±5% settling band shaded around target
    band = SETTLE_BAND * np.abs(tgt)
    ax.fill_between(T, tgt - band, tgt + band, alpha=0.12, color=col,
                    label='±5% band')
    ax.plot(T, tgt, '--', color=col, lw=1.3, alpha=0.6, label='Target')
    ax.plot(T, act, '-',  color=col, lw=0.9, alpha=0.9, label='Actual')
    ax.axhline(0, color='gray', lw=0.6, ls=':')

    rms = np.sqrt(np.nanmean(err**2))
    ax.set_title(f'Motor  {m}  |  RMS error = {rms:.3f} rad/s',
                 fontweight='bold', color=col)
    ax.set_ylabel('Speed  (rad/s)')
    ax.legend(loc='upper right', framealpha=0.5)

for ax in axes[1]:
    ax.set_xlabel('Time  (s)')

fig1.tight_layout()
plt.savefig('plot1_speed_tracking.png', bbox_inches='tight', dpi=150)
plt.show()
print("✓  Plot 1 saved → plot1_speed_tracking.png")


# ─────────────────────────────────────────────────────────────────────────────
# CELL 4 ── PLOT 2 : Error + PWM Dynamics  (dual Y-axis)
# ─────────────────────────────────────────────────────────────────────────────
# WHAT TO LOOK FOR:
#   Error oscillates while PWM is saturated → integral windup
#   PWM flat at ±255 while error is large → setpoint physically unreachable
#   Error sign flips rapidly → instability / Kp too high
#   PWM = 0 while error ≠ 0 → deadband too wide or ESTOP latch active
#   Red shading = saturation periods — more red = more windup risk

fig2, axes2 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
fig2.suptitle('AISLEBOT  ·  PLOT 2 — Error & PWM Dynamics  (dual axis)',
              fontweight='bold', fontsize=13)

for ax, m in zip(axes2.flat, MOTORS):
    col = COLORS[m]
    err = df[f'{m}_error'].fillna(0).values
    pwm = pd.to_numeric(df[f'{m}_pwm'], errors='coerce').fillna(0).values
    sat_mask = np.abs(pwm) >= PWM_SAT

    # Safe y-limits
    e_min = float(np.nanmin(err)); e_max = float(np.nanmax(err))
    pad   = max(abs(e_max - e_min) * 0.1, 0.3)
    ylim  = (e_min - pad, e_max + pad)

    ax.fill_between(T[:len(err)], ylim[0], ylim[1],
                    where=sat_mask[:len(T)], alpha=0.10, color='#dc2626')
    ax.plot(T[:len(err)], err, '-', color=col, lw=1.0, alpha=0.9)
    ax.axhline(0, color='gray', lw=0.6, ls=':')
    ax.set_ylim(ylim)
    ax.set_ylabel('Error  (rad/s)', color=col)
    ax.tick_params(axis='y', labelcolor=col)

    ax2 = ax.twinx()
    ax2.plot(T[:len(pwm)], pwm, '-', color='#9ca3af', lw=0.6, alpha=0.5)
    ax2.axhline( PWM_SAT, color='#dc2626', lw=0.7, ls='--', alpha=0.5)
    ax2.axhline(-PWM_SAT, color='#dc2626', lw=0.7, ls='--', alpha=0.5)
    ax2.set_ylabel('PWM', color='#6b7280')
    ax2.tick_params(axis='y', labelcolor='#6b7280')
    ax2.set_ylim(-285, 285)

    rms     = np.sqrt(np.nanmean(err**2))
    sat_pct = sat_mask.mean() * 100
    ax.set_title(f'Motor  {m}  |  RMS={rms:.3f} rad/s   PWM sat={sat_pct:.1f}%',
                 fontweight='bold', color=col)

    legend_handles = [
        Line2D([0],[0], color=col,      lw=1.5, label='Error (rad/s)'),
        Line2D([0],[0], color='#9ca3af', lw=1.0, alpha=0.6, label='PWM'),
        Patch(facecolor='#dc2626', alpha=0.2, label=f'PWM sat ≥{PWM_SAT}'),
    ]
    ax.legend(handles=legend_handles, loc='upper right', framealpha=0.5)

for ax in axes2[1]:
    ax.set_xlabel('Time  (s)')

fig2.tight_layout()
plt.savefig('plot2_error_pwm.png', bbox_inches='tight', dpi=150)
plt.show()
print("✓  Plot 2 saved → plot2_error_pwm.png")


# ─────────────────────────────────────────────────────────────────────────────
# CELL 5 ── PLOT 3 : Diagnostic Dashboard
# ─────────────────────────────────────────────────────────────────────────────
# Panel A — RMS error bar: who is worst overall
# Panel B — Diagonal deviation FR−RL and FL−RR: root cause of stutter
# Panel C — PWM saturation %: who is hitting the wall
# Panel D — Settling time: how long each motor takes to lock on after a step

def detect_settling_times(time, target, actual,
                          band=SETTLE_BAND, min_step=SETTLE_MIN_STEP):
    """
    For every step-change in target > min_step rad/s,
    finds the first moment where actual stays within ±band*|target|
    for at least 10 consecutive samples.
    Returns list of settling times in seconds (None = did not settle).
    """
    valid = ~(np.isnan(target) | np.isnan(actual))
    t  = time[valid];   tgt = target[valid];   act = actual[valid]
    edges = np.where(np.abs(np.diff(tgt)) > min_step)[0]
    results = []
    for ei in edges:
        step_val = tgt[ei + 1]
        if abs(step_val) < min_step:
            continue
        thr = abs(step_val) * band
        settled_idx = None
        for k in range(ei + 1, len(t) - 10):
            if np.all(np.abs(act[k:k+10] - step_val) <= thr):
                settled_idx = k; break
        results.append(t[settled_idx] - t[ei+1] if settled_idx else None)
    return results


# Pre-compute metrics
rms_vals     = [np.sqrt(np.nanmean(df[f'{m}_error'].values**2)) for m in MOTORS]
sat_pct_vals = [
    (np.abs(pd.to_numeric(df[f'{m}_pwm'], errors='coerce').fillna(0).values) >= PWM_SAT).mean() * 100
    for m in MOTORS
]
settle_means = []
settle_lbls  = []
for m in MOTORS:
    tgt = df[f'{m}_target_rads'].values
    act = df[f'{m}_actual_rads'].values
    sts = detect_settling_times(T, tgt, act)
    settled = [s for s in sts if s is not None]
    settle_means.append(np.mean(settled) if settled else np.nan)
    settle_lbls.append(f'{m}\n({len(settled)}/{len(sts)} steps)')

fr_rl     = df['FR_actual_rads'].values - df['RL_actual_rads'].values
fl_rr     = df['FL_actual_rads'].values - df['RR_actual_rads'].values
fr_rl_rms = np.sqrt(np.nanmean(fr_rl**2))
fl_rr_rms = np.sqrt(np.nanmean(fl_rr**2))

# Build figure
fig3 = plt.figure(figsize=(14, 10))
fig3.suptitle('AISLEBOT  ·  PLOT 3 — Diagnostic Dashboard',
              fontweight='bold', fontsize=13)
gs = gridspec.GridSpec(2, 2, figure=fig3, hspace=0.45, wspace=0.38)

# ── Panel A — RMS Error ───────────────────────────────────────────────────────
ax_a = fig3.add_subplot(gs[0, 0])
bars = ax_a.bar(MOTORS, rms_vals,
                color=[COLORS[m] for m in MOTORS], edgecolor='white', lw=0.8)
for bar, val in zip(bars, rms_vals):
    ax_a.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
              f'{val:.3f}', ha='center', va='bottom', fontsize=9)
worst = MOTORS[int(np.argmax(rms_vals))]
ax_a.set_title(f'A — RMS Error  (worst: {worst} = {max(rms_vals):.3f} rad/s)',
               fontweight='bold')
ax_a.set_ylabel('RMS Error  (rad/s)')
ax_a.set_xlabel('Motor')

# ── Panel B — Cross-Motor Diagonal Deviation ──────────────────────────────────
ax_b = fig3.add_subplot(gs[0, 1])
ax_b.plot(T, fr_rl, color=COLORS['FR'], lw=0.85,
          label=f'FR − RL  (RMS={fr_rl_rms:.3f})')
ax_b.plot(T, fl_rr, color=COLORS['FL'], lw=0.85,
          label=f'FL − RR  (RMS={fl_rr_rms:.3f})')
ax_b.axhline(0, color='gray', lw=0.7, ls='--', alpha=0.5)
ax_b.fill_between(T, -0.2, 0.2, alpha=0.08, color='gray', label='±0.2 tolerance')
ax_b.set_title('B — Cross-Motor Diagonal Deviation', fontweight='bold')
ax_b.set_ylabel('Speed diff  (rad/s)')
ax_b.set_xlabel('Time  (s)')
ax_b.legend(framealpha=0.6)

# ── Panel C — PWM Saturation ──────────────────────────────────────────────────
ax_c = fig3.add_subplot(gs[1, 0])
bars_c = ax_c.bar(MOTORS, sat_pct_vals,
                  color=[COLORS[m] for m in MOTORS], edgecolor='white', lw=0.8)
ax_c.axhline(20, color='#dc2626', lw=1.0, ls='--', alpha=0.7, label='20% warning')
for bar, val in zip(bars_c, sat_pct_vals):
    ax_c.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
              f'{val:.1f}%', ha='center', va='bottom', fontsize=9,
              color='#dc2626', fontweight='bold')
ax_c.set_ylim(0, 110)
ax_c.set_title(f'C — PWM Saturation  (|PWM| ≥ {PWM_SAT})', fontweight='bold')
ax_c.set_ylabel('% samples saturated')
ax_c.set_xlabel('Motor')
ax_c.legend(framealpha=0.6)

# ── Panel D — Settling Time ───────────────────────────────────────────────────
ax_d = fig3.add_subplot(gs[1, 1])
x_pos = np.arange(len(MOTORS))
valid = ~np.isnan(settle_means)
if any(valid):
    bars_d = ax_d.bar(
        x_pos[valid],
        [v for v, ok in zip(settle_means, valid) if ok],
        color=[COLORS[m] for m, ok in zip(MOTORS, valid) if ok],
        edgecolor='white', lw=0.8
    )
    for bar in bars_d:
        ax_d.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                  f'{bar.get_height():.2f}s', ha='center', va='bottom', fontsize=9)
else:
    ax_d.text(0.5, 0.5, 'No step responses\ndetected in capture',
              ha='center', va='center', transform=ax_d.transAxes,
              fontsize=10, color='gray')
ax_d.set_xticks(x_pos)
ax_d.set_xticklabels(settle_lbls, fontsize=8.5)
ax_d.set_title('D — Mean Settling Time  (±5% band, 10-sample window)',
               fontweight='bold')
ax_d.set_ylabel('Settling time  (s)')

plt.savefig('plot3_diagnostic_dashboard.png', bbox_inches='tight', dpi=150)
plt.show()
print("✓  Plot 3 saved → plot3_diagnostic_dashboard.png")

# ── Terminal summary ──────────────────────────────────────────────────────────
print("\n" + "═"*60)
print("  DIAGNOSTIC SUMMARY")
print("═"*60)
print(f"  Capture : {total_dur:.1f}s  @  {fs:.0f} Hz  ({len(df)} samples)")
print()
print(f"  {'Motor':<6}  {'RMS Error':>10}  {'PWM Sat':>9}  {'Settle time':>12}")
print(f"  {'──────':<6}  {'──────────':>10}  {'─────────':>9}  {'────────────':>12}")
for i, m in enumerate(MOTORS):
    st = f"{settle_means[i]:.2f}s" if not np.isnan(settle_means[i]) else "N/A"
    flag = "  ← WORST" if m == worst else ""
    print(f"  {m:<6}  {rms_vals[i]:>10.3f}  {sat_pct_vals[i]:>8.1f}%  {st:>12}{flag}")
print()
print(f"  Cross-motor deviation (diagonal mismatch):")
print(f"    FR−RL RMS : {fr_rl_rms:.3f} rad/s")
print(f"    FL−RR RMS : {fl_rr_rms:.3f} rad/s")
print("═"*60)
print()
print("  INTERPRETATION FLAGS:")
if max(sat_pct_vals) > 20:
    print(f"  ⚠  PWM saturation > 20% on all motors — PID setpoints may")
    print(f"     exceed what motors can physically deliver. Consider reducing")
    print(f"     MAX_LINEAR / MAX_ANGULAR in teleop, or check motor load.")
if max(rms_vals) > 0.5:
    print(f"  ⚠  RMS errors > 0.5 rad/s — large tracking gap.")
    print(f"     If PWM is also saturated, this is a power/load issue, not PID.")
    print(f"     If PWM is NOT saturated, increase Kp or reduce deadband.")
if fr_rl_rms > 0.3 or fl_rr_rms > 0.3:
    print(f"  ⚠  Diagonal mismatch > 0.3 rad/s — this is the stutter source.")
    print(f"     RL motor (weakest) needs higher Kp/Kff than FR.")
