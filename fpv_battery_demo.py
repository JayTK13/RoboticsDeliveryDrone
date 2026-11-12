"""
Webots controller: how LiPo battery state affects a small FPV quad (v2, gentler).

Flies the Mavic 2 Pro model as a stand-in body. Battery and motor physics use the
FPV numbers below. Because the sim body has no air drag and far less thrust margin
than a real 5" quad, the sim's thrust headroom is scaled down by SIM_SCALE so the
punch-out is a ~0.5-0.9 g climb instead of a rocket launch. Relative differences
between battery states are preserved.

Flight plan: hover at 1 m -> 1 s near-full-throttle punch -> brake and hover again.
"""
import math
from controller import Robot

# ------------------------- YOUR BUILD (edit these) --------------------------
CELLS = 4                       # 4S = 4, 6S = 6
CAPACITY_MAH = 1300
START_SOC = 0.1                 # 1.0 = fresh pack, 0.1 = nearly empty
PACK_RESISTANCE = 0.020         # ohms total (cheap/old packs: 0.03-0.06)
MASS_KG = 0.65                  # all-up weight
MAX_THRUST_PER_MOTOR_N = 12.0   # at nominal voltage (about 1.2 kgf)
CURRENT_WOT_PER_MOTOR = 28.0    # amps per motor at full throttle, nominal voltage
IDLE_CURRENT = 1.0              # FC + VTX + camera, amps
LVC_CELL_V = 3.3                # loaded per-cell voltage where you'd land

# ---------------------- Sim tuning (leave unless needed) ---------------------
SIM_SCALE = 0.15                # fraction of real thrust headroom used in the sim
OMEGA_HOVER = 68.5              # rad/s the Mavic needs to hover
K_ROLL_P, K_PITCH_P = 50.0, 30.0
K_ALT, V_MAX = 0.8, 1.5         # altitude loop: target climb speed (m/s per m, cap)
K_VEL, VERT_LIMIT = 6.0, 12.0   # velocity loop: rad/s per m/s, cap in rad/s
PUNCH_THROTTLE = 0.95           # fraction of available motor speed during punch

# --------------------------- Battery / motor model --------------------------
G = 9.81
V_NOM = 3.7 * CELLS
T_HOVER = MASS_KG * G / 4.0

OCV = [(0.0, 3.30), (0.1, 3.55), (0.2, 3.66), (0.3, 3.72), (0.4, 3.76), (0.5, 3.80),
       (0.6, 3.84), (0.7, 3.90), (0.8, 3.98), (0.9, 4.08), (1.0, 4.20)]


def cell_ocv(soc):
    soc = max(0.0, min(1.0, soc))
    for (s0, v0), (s1, v1) in zip(OCV, OCV[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return OCV[-1][1]


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


def real_thrust(w):
    """Sim motor speed -> equivalent real FPV thrust per motor (N), for current draw."""
    r = (w / OMEGA_HOVER) ** 2
    return T_HOVER * (r if r < 1.0 else 1.0 + (r - 1.0) / SIM_SCALE)


robot = Robot()
dt_ms = int(robot.getBasicTimeStep())
dt = dt_ms / 1000.0

gps = robot.getDevice("gps")
imu = robot.getDevice("inertial unit")
gyro = robot.getDevice("gyro")
for d in (gps, imu, gyro):
    d.enable(dt_ms)

motors = [robot.getDevice(n) for n in
          ("front left propeller", "front right propeller",
           "rear left propeller", "rear right propeller")]
for m in motors:
    m.setPosition(float("inf"))
    m.setVelocity(1.0)

while robot.step(dt_ms) != -1:          # sensor warm-up
    if robot.getTime() > 1.0:
        break

t0 = robot.getTime()
soc = START_SOC
current = IDLE_CURRENT
last_alt = None
climb_f = 0.0
last_print = -1.0
peak_climb = 0.0
punch_start_alt = None
punch_gain = 0.0
min_cell_loaded = 99.0
lvc_tripped = False

print(f"{CELLS}S {CAPACITY_MAH}mAh | start SoC {START_SOC*100:.0f}% | "
      f"{MASS_KG*1000:.0f} g | TWR at nominal {4*MAX_THRUST_PER_MOTOR_N/(MASS_KG*G):.1f}")
print(f"{'t':>5} {'phase':>7} {'SoC%':>5} {'Vpack':>6} {'V/cell':>6} {'amps':>6} "
      f"{'TWR':>5} {'alt':>6} {'climb m/s':>9} {'sat':>4}")

while robot.step(dt_ms) != -1:
    t = robot.getTime() - t0

    # ---- battery under load ------------------------------------------------
    v_open = CELLS * cell_ocv(soc)
    v_load = max(v_open - PACK_RESISTANCE * current, 0.5)
    cell_loaded = v_load / CELLS
    min_cell_loaded = min(min_cell_loaded, cell_loaded)
    if cell_loaded < LVC_CELL_V:
        lvc_tripped = True

    t_max = MAX_THRUST_PER_MOTOR_N * (v_load / V_NOM) ** 2   # thrust ~ V^2
    twr = 4.0 * t_max / (MASS_KG * G)
    omega_max = OMEGA_HOVER * math.sqrt(1.0 + (twr - 1.0) * SIM_SCALE)

    # ---- sensors -----------------------------------------------------------
    roll, pitch, _ = imu.getRollPitchYaw()
    alt = gps.getValues()[2]
    roll_rate, pitch_rate, _ = gyro.getValues()
    raw_climb = 0.0 if last_alt is None else (alt - last_alt) / dt
    last_alt = alt
    climb_f = 0.8 * climb_f + 0.2 * raw_climb

    # ---- flight plan -------------------------------------------------------
    if t < 5.0:
        phase = "hover"
    elif t < 6.0:
        phase = "PUNCH"
    elif t < 14.0:
        phase = "recover"
    else:
        break

    if phase == "PUNCH":
        if punch_start_alt is None:
            punch_start_alt = alt
        base = PUNCH_THROTTLE * omega_max
        peak_climb = max(peak_climb, climb_f)
        punch_gain = alt - punch_start_alt
    else:
        v_target = clamp(K_ALT * (1.0 - alt), -V_MAX, V_MAX)
        base = OMEGA_HOVER + clamp(K_VEL * (v_target - climb_f), -VERT_LIMIT, VERT_LIMIT)

    # keep attitude loop gain constant as motor speed rises (thrust slope ~ omega)
    att = OMEGA_HOVER / max(base, OMEGA_HOVER)
    roll_in = att * (K_ROLL_P * clamp(roll, -1, 1) + roll_rate)
    pitch_in = att * (K_PITCH_P * clamp(pitch, -1, 1) + pitch_rate)

    cmds = [base + pitch_in - roll_in,     # FL
            base + pitch_in + roll_in,     # FR
            base - pitch_in - roll_in,     # RL
            base - pitch_in + roll_in]     # RR

    # ---- the battery limit -------------------------------------------------
    actual = [clamp(c, 0.0, omega_max) for c in cmds]
    saturated = any(c > omega_max for c in cmds)

    motors[0].setVelocity(actual[0])
    motors[1].setVelocity(-actual[1])
    motors[2].setVelocity(-actual[2])
    motors[3].setVelocity(actual[3])

    # ---- current draw and state of charge ----------------------------------
    amps = IDLE_CURRENT
    for w in actual:
        amps += (CURRENT_WOT_PER_MOTOR
                 * (real_thrust(w) / MAX_THRUST_PER_MOTOR_N) ** 1.5 * (V_NOM / v_load))
    current = 0.7 * current + 0.3 * amps
    soc = max(0.0, soc - current * dt / (CAPACITY_MAH * 3.6))

    if t - last_print >= 1.0:
        last_print = t
        print(f"{t:5.1f} {phase:>7} {soc*100:5.1f} {v_load:6.2f} {cell_loaded:6.2f} "
              f"{current:6.1f} {twr:5.1f} {alt:6.2f} {climb_f:9.2f} {'YES' if saturated else 'no':>4}")

print("\n--- summary ---")
print(f"Peak climb rate in punch-out: {peak_climb:.1f} m/s")
print(f"Altitude gained in 1 s punch: {punch_gain:.1f} m")
print(f"Lowest loaded cell voltage:   {min_cell_loaded:.2f} V "
      f"({'BELOW' if lvc_tripped else 'above'} the {LVC_CELL_V} V landing threshold)")