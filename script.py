import numpy as np
import matplotlib.pyplot as plt

# ---------- Robot parameters ----------
r = 3.2            # wheel radius [cm]
wheelbase = 13.7   # distance between wheel centers [cm]
b = wheelbase / 2  # half-wheelbase [cm]
T = 5.0            # run duration [s]
IN2CM = 2.54

duty = np.array([50, 60, 70, 80, 100])
pwm = duty / 100 * 255          # 127.5, 153, 178.5, 204, 255

# ---------- Floor runs: forward distance x [in] (5 trials each) ----------
# 50% trial 1 (34 in) is the outlier, excluded
floor_x_in = {
    50:  [41, 42, 42, 45],               # outlier 34 left out
    60:  [49, 51, 51, 52, 53],
    70:  [55, 54, 57, 56, 53],
    80:  [60, 50, 59, 55, 58],
    100: [76, 74, 76, 76, 75.5],
}

x_mean_in = np.array([np.mean(floor_x_in[d]) for d in duty])
x_mean_cm = x_mean_in * IN2CM
v = x_mean_cm / T                # [cm/s]
phi_floor = v / r                # [rad/s]

kf, cf = np.polyfit(pwm, phi_floor, 1)   # line of best fit (least squares)

# ---------- In-air runs: revolutions in 5 s ----------
air_R = {
    50:  [5.4, 5.5, 5.7, 5.5, 5.8],
    60:  [6.6, 6.6, 6.8, 6.8, 6.9, 6.6],
    70:  [7.2, 7.2, 7.4, 7.5, 7.3],
    80:  [8.0, 8.1, 8.1, 8.05, 8.0],
    100: [9.4, 9.3, 9.4, 9.2, 9.35],
}
air_L = {
    50:  [5.2, 5.3, 5.5, 5.4, 5.2],
    60:  [6.2, 6.0, 5.9, 5.9, 5.9, 6.1],
    70:  [6.7, 6.6, 6.7, 6.6, 6.5],
    80:  [7.25, 7.35, 7.2, 7.3, 7.25],
    100: [8.2, 8.25, 8.25, 8.25, 8.25],
}

N_R = np.array([np.mean(air_R[d]) for d in duty])
N_L = np.array([np.mean(air_L[d]) for d in duty])
phi_R = 2 * np.pi * N_R / T
phi_L = 2 * np.pi * N_L / T

kR, cR = np.polyfit(pwm, phi_R, 1)
kL, cL = np.polyfit(pwm, phi_L, 1)

# ---------- Hybrid model (1 = right, 2 = left) ----------
k1, k2 = kf + 0.5 * (kR - kL), kf - 0.5 * (kR - kL)
c1, c2 = cf + 0.5 * (cR - cL), cf - 0.5 * (cR - cL)

def wheel_speeds(pwm1, pwm2):
    """PWM -> wheel angular velocities [rad/s]"""
    return k1 * pwm1 + c1, k2 * pwm2 + c2

def forward_kinematics(pwm1, pwm2):
    """PWM -> (v_x [cm/s], omega [rad/s])"""
    p1, p2 = wheel_speeds(pwm1, pwm2)
    return r / 2 * (p1 + p2), r / (2 * b) * (p1 - p2)

def inverse_to_pwm(vx, omega):
    """Desired (v_x, omega) -> (PWM1, PWM2)"""
    p1 = vx / r + b * omega / r
    p2 = vx / r - b * omega / r
    return (p1 - c1) / k1, (p2 - c2) / k2

def step(pose, pwm1, pwm2, dt):
    """One forward-Euler step of the motion model, pose = (x, y, theta)"""
    x, y, th = pose
    vx, w = forward_kinematics(pwm1, pwm2)
    return (x + np.cos(th) * vx * dt, y + np.sin(th) * vx * dt, th + w * dt)

# ---------- Print results ----------
print("Floor summary")
for d, p, xi, xc, vv, ph in zip(duty, pwm, x_mean_in, x_mean_cm, v, phi_floor):
    print(f"  {d:3d}%  PWM {p:6.1f}  x={xi:5.1f} in = {xc:6.2f} cm  v={vv:5.2f} cm/s  phi={ph:5.2f} rad/s")
print(f"  fit: phi = {kf:.4f}*PWM + {cf:.2f}\n")

print("In-air summary")
for d, p, nr, nl, pr, pl in zip(duty, pwm, N_R, N_L, phi_R, phi_L):
    print(f"  {d:3d}%  PWM {p:6.1f}  N_R={nr:.2f} N_L={nl:.2f}  phi_R={pr:5.2f} phi_L={pl:5.2f}")
print(f"  right fit: phi_R = {kR:.4f}*PWM + {cR:.2f}")
print(f"  left fit:  phi_L = {kL:.4f}*PWM + {cL:.2f}\n")

print("Hybrid model")
print(f"  phi_1 (right) = {k1:.4f}*PWM1 + {c1:.2f}")
print(f"  phi_2 (left)  = {k2:.4f}*PWM2 + {c2:.2f}\n")

print("Kinematics with PWM input")
print(f"  v_x   = {r/2*k1:.4f}*PWM1 + {r/2*k2:.4f}*PWM2 + {r/2*(c1+c2):.2f} cm/s")
print(f"  omega = {r/(2*b)*k1:.5f}*PWM1 - {r/(2*b)*k2:.5f}*PWM2 + {r/(2*b)*(c1-c2):.4f} rad/s")

# ---------- Plots ----------
xs = np.linspace(120, 260, 100)

fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))

# raw floor data
for d, p in zip(duty, pwm):
    ax[0].scatter([p] * len(floor_x_in[d]), floor_x_in[d], c="tab:blue")
ax[0].scatter([127.5], [34], c="red", marker="x", label="outlier (excluded)")
ax[0].set(xlabel="PWM", ylabel="Forward distance x [in]", title="Floor: raw data")
ax[0].legend(); ax[0].grid(True)

# floor fit
ax[1].scatter(pwm, phi_floor, label="Floor mean")
ax[1].plot(xs, kf * xs + cf, label="Line of best fit")
ax[1].set(xlabel="PWM", ylabel="phi_dot [rad/s]", title="Floor: average wheel speed")
ax[1].legend(); ax[1].grid(True)

# air fits
ax[2].scatter(pwm, phi_R, c="tab:blue", label="Right (mean)")
ax[2].plot(xs, kR * xs + cR, c="tab:blue", label="Right (fit)")
ax[2].scatter(pwm, phi_L, c="tab:red", marker="s", label="Left (mean)")
ax[2].plot(xs, kL * xs + cL, c="tab:red", label="Left (fit)")
ax[2].set(xlabel="PWM", ylabel="phi_dot [rad/s]", title="In air: per wheel")
ax[2].legend(); ax[2].grid(True)

plt.tight_layout()
plt.savefig("pwm_calibration.png", dpi=200)
plt.show()