# Yaw plant model and parameter list (M1.1)

| | |
|---|---|
| WBS | M1.1 (team WBS 1.6, control design) |
| Owner | Maurício (control architecture) |
| Used by | Ana (simulator), Edson (moteus limits), M1.2 to M2.7 (tuning) |
| Version | v1.0, 12 Nov 2026 |
| Numbers | `docs/plant_params.h` holds the same values as this note |

This note fixes the equations and the parameters that the controller design and the simulator must share. M1.1 is done when the simulator integrates exactly the equations in section 3 with the parameters in section 6.

**Status of the numbers:** every physical value here is an estimate copied from the current simulator (`v5/teste.c`). Nothing has been measured on the real cube yet. Section 6 says how to measure each one.

## 1. Scope

One reaction wheel on the yaw axis, as in the lecturer's brief. Roll and pitch are not modelled. The model covers momentum exchange between body and wheel, friction in the gimbal's yaw bearing, gravity torque from imbalance, and the wheel actuator (moteus) in both torque and velocity mode.

## 2. Symbols and sign conventions

Units are SI throughout: rad, rad/s, s, N·m, kg·m².

| Symbol | Meaning | Unit |
|---|---|---|
| $\psi$ | Body yaw angle | rad |
| $\omega$ | Body yaw rate, $\dot\psi$ (what gyro z measures) | rad/s |
| $\Omega$ | Wheel speed **relative to the body** (what the moteus reports) | rad/s |
| $\tau_m$ | Motor torque on the wheel | N·m |
| $\tau_f$ | Friction torque of the gimbal yaw bearing on the body | N·m |
| $\tau_g$ | Gravity torque about the yaw axis | N·m |
| $\tau_{ext}$ | Any other external torque (a push by hand) | N·m |
| $J_b$ | Body yaw inertia **without** the wheel's spin inertia | kg·m² |
| $J_w$ | Wheel plus rotor inertia about the spin axis | kg·m² |
| $J_t = J_b + J_w$ | Total yaw inertia | kg·m² |
| $J_r = J_b J_w / J_t$ | Reduced inertia seen by the motor | kg·m² |

Sign rules:

- $\psi$, $\omega$ and $\Omega$ are positive counter-clockwise seen from above (z axis up).
- Positive $\tau_m$ accelerates the wheel in the positive direction, so the body receives $-\tau_m$.
- Check on the bench: positive motor torque must turn the body towards negative yaw.

These match `v5/hal_cubo.h` and the first hardware test in `v5/README.md`.

## 3. Equations of motion

Newton's law for the body, and for the wheel in absolute terms (its absolute speed is $\omega + \Omega$):

```math
J_b\,\dot\omega = -\tau_m + \tau_f + \tau_g + \tau_{ext}
```

```math
J_w\,(\dot\omega + \dot\Omega) = \tau_m
```

Adding the two removes the internal torque $\tau_m$ and gives the momentum balance:

```math
\frac{d}{dt}\Big[\,J_t\,\omega + J_w\,\Omega\,\Big] = \tau_f + \tau_g + \tau_{ext}
```

With no external torque the total angular momentum $H = J_t\,\omega + J_w\,\Omega$ is constant, so a change in wheel speed gives a proportional, opposite change in body rate:

```math
\Delta\omega = -\frac{J_w}{J_t}\,\Delta\Omega
```

Two consequences for the controller:

1. To slew the body at $\omega_{ref}$ the wheel must change speed by $-(J_t/J_w)\,\omega_{ref}$. This is the feed-forward of the rate loop.
2. Any steady external torque $\tau$ held off by the controller makes the wheel accelerate at $\tau/J_w$ until it saturates. The wheel can only store momentum, it cannot remove it.

### 3.1 Bearing friction

Viscous plus Coulomb, with stiction at rest (Karnopp form, which avoids chatter around zero speed):

```math
\tau_f =
\begin{cases}
-b\,\omega - \tau_c\,\mathrm{sgn}(\omega) & |\omega| > \omega_\varepsilon \\
-\mathrm{clamp}(\tau_{net},\,-\tau_s,\,+\tau_s) & |\omega| \le \omega_\varepsilon
\end{cases}
```

$\tau_{net}$ is the sum of all the other torques on the body. While $|\tau_{net}| \le \tau_s$ the body stays stuck and $\omega$ is held at zero. Setting $\tau_s = \tau_c$ switches stiction off.

### 3.2 Gravity torque

If the yaw axis were exactly vertical, gravity would give no yaw torque. It appears when the axis is tilted by an angle $\gamma$ and the centre of mass sits a distance $r$ off the axis:

```math
\tau_g = -k_g\,\sin(\psi - \psi_0), \qquad k_g = m\,g\,r\,\sin\gamma
```

$\psi_0$ is the yaw angle at which the heavy side hangs lowest. The cube then behaves as a slow pendulum about $\psi_0$ with natural frequency $\sqrt{k_g/J_t}$.

### 3.3 Wheel actuator

The moteus applies $\tau_m$, limited to $\pm\tau_{max}$. There are two ways to command it, and the team has not yet frozen which one is used (section 8, item 1). The simulator should support both behind one switch.

**Torque mode** (what `v5` does now): the controller output is $\tau_m$ directly.

**Velocity mode** (what the WBS plan assumes): the controller sends $\Omega_{ref}$ and the moteus closes its own loop, modelled as a proportional velocity loop:

```math
\tau_m = \mathrm{clamp}\big(k_v\,(\Omega_{ref} - \Omega),\,-\tau_{max},\,+\tau_{max}\big)
```

Inside the torque limit this is a first-order lag from $\Omega_{ref}$ to $\Omega$ with time constant

```math
\tau_w = \frac{J_r}{k_v}
```

In both modes the wheel speed is limited: when $|\Omega| \ge \Omega_{max}$, any torque that would speed the wheel up further is set to zero, and braking torque still passes.

Wheel bearing drag and motor cogging are internal torques between wheel and body. They do not change the momentum balance. In velocity mode the moteus loop absorbs them. In torque mode they subtract from the commanded torque: $\tau_m \rightarrow \tau_m - b_w\,\Omega$.

## 4. State-space form

State $x = [\psi,\ \omega,\ \Omega]^T$. The nonlinear model the simulator integrates is:

```math
\dot\psi = \omega
```

```math
\dot\omega = \frac{-\tau_m + \tau_f(\omega) + \tau_g(\psi) + \tau_{ext}}{J_b}
```

```math
\dot\Omega = \frac{\tau_m}{J_w} - \dot\omega
```

Linearised for design (velocity mode, no Coulomb friction, small angle about $\psi_0 = 0$, inside all limits), with input $u = \Omega_{ref}$ and disturbance $\tau_{ext}$:

```math
\dot x =
\begin{bmatrix}
0 & 1 & 0 \\
-\dfrac{k_g}{J_b} & -\dfrac{b}{J_b} & \dfrac{k_v}{J_b} \\
\dfrac{k_g}{J_b} & \dfrac{b}{J_b} & -\dfrac{k_v}{J_r}
\end{bmatrix} x
+
\begin{bmatrix} 0 \\ -\dfrac{k_v}{J_b} \\ \dfrac{k_v}{J_r} \end{bmatrix} \Omega_{ref}
+
\begin{bmatrix} 0 \\ \dfrac{1}{J_b} \\ -\dfrac{1}{J_b} \end{bmatrix} \tau_{ext}
```

Outputs: $\omega$ (gyro), $\psi$ (attitude estimate or camera), $\Omega$ (moteus readback).

## 5. Transfer functions

Velocity mode, wheel-speed setpoint to yaw angle, with $k_g = 0$:

```math
\frac{\psi(s)}{\Omega_{ref}(s)} \approx -\,\frac{J_w}{(J_t\,s + b)\,(\tau_w\,s + 1)}
```

With negligible friction this becomes an integrator, a gain and the wheel lag:

```math
\frac{\psi(s)}{\Omega_{ref}(s)} \approx -\,\frac{J_w}{J_t}\cdot\frac{1}{s\,(\tau_w\,s + 1)}
```

Torque mode, motor torque to yaw angle:

```math
\frac{\psi(s)}{\tau_m(s)} = -\,\frac{1}{s\,(J_b\,s + b)}
```

External torque to yaw angle (open loop, wheel speed held):

```math
\frac{\psi(s)}{\tau_{ext}(s)} = \frac{1}{s\,(J_t\,s + b)}
```

Note on the first form: because of viscous friction a wheel-speed step produces a finite yaw displacement, not a constant rate. The body rate decays with time constant $J_t/b$ (about 25 s with the nominal values), which is slow compared with the loops, so the integrator form is the one to design with.

## 6. Parameter list

Nominal values are the ones in `v5/teste.c` unless stated. "Range" is what to sweep in the robustness campaign (M3.3).

### 6.1 Plant

| Parameter | Symbol | Nominal | Range | Status | How to measure |
|---|---|---|---|---|---|
| Body yaw inertia (no wheel) | $J_b$ | 0.050 kg·m² | 0.002 to 0.06 | Estimate | Wheel step with the cube free: $J_t = -J_w\,\Delta\Omega/\Delta\omega$. Or CAD mass properties. |
| Wheel + rotor inertia | $J_w$ | 0.0010 kg·m² | 0.0001 to 0.002 | Estimate | CAD plus rotor data, or a known torque step on the bench: $J_w = \tau_m/\dot\Omega$ with the body clamped. |
| Viscous friction, yaw bearing | $b$ | 0.002 N·m·s/rad | 0 to 0.01 | Estimate | Coast-down from about 1 rad/s with the wheel idle; fit the speed curve. |
| Coulomb friction, yaw bearing | $\tau_c$ | 0.002 N·m | 0.0005 to 0.006 | Estimate | Same coast-down: the final straight-line part has slope $-\tau_c/J_t$. |
| Stiction (breakaway) torque | $\tau_s$ | 0.003 N·m | $\tau_c$ to $2\tau_c$ | Assumed, not in the simulator | Ramp the wheel slowly from rest; note the wheel acceleration at which the body first moves: $\tau_s = J_w\,\dot\Omega$. |
| Stick velocity threshold | $\omega_\varepsilon$ | 1e-4 rad/s | fixed | Numerical | Not physical. |
| Gravity torque amplitude | $k_g$ | 0 N·m | 0 to 0.005 | Not in the simulator | Release the cube from several angles with the motor off; the swing period gives $k_g = J_t\,(2\pi/T)^2$. Aim for zero by balancing. |
| Heavy-side angle | $\psi_0$ | 0 rad | $-\pi$ to $\pi$ | Not in the simulator | Angle at which the unpowered cube comes to rest. |
| Wheel bearing drag | $b_w$ | 0 N·m·s/rad | 0 to 1e-4 | Not in the simulator | Wheel coast-down with the body clamped. Only matters in torque mode. |

### 6.2 Actuator

| Parameter | Symbol | Nominal | Status | Source or how to measure |
|---|---|---|---|---|
| Torque limit | $\tau_{max}$ | 0.2 N·m | Software limit | `MAX_TORQUE_COMMAND` in `v5/controlo_atitude.h`. To agree with Edson as a moteus limit too. |
| Wheel speed limit | $\Omega_{max}$ | 314.16 rad/s (3000 rpm) | Software limit | `MAX_WHEEL_SPEED_RAD_S`. No-load ceiling is about 7300 rpm (Kv 330 × 22.2 V). |
| Soft wheel speed limit | $\Omega_{soft}$ | 251.33 rad/s (2400 rpm) | Software limit | `SOFT_WHEEL_SPEED_RAD_S`. |
| Wheel velocity-loop time constant | $\tau_w$ | 0.02 s | Guess | Step $\Omega_{ref}$ on the bench and log $\Omega$; time to reach 63%. |
| Velocity-loop gain | $k_v$ | $J_r/\tau_w$ = 0.049 N·m·s/rad | Derived | From the measured $\tau_w$. |
| Motor torque constant | $K_t$ | 0.029 N·m/A | Derived from Kv 330 | $K_t \approx 60/(2\pi\,K_v)$. Check against the moteus calibration. |
| Command delay (CAN-FD + moteus) | $T_d$ | 0 s | Not in the simulator | Timestamp a command and the first change in the readback. |

### 6.3 Derived quantities (nominal values)

| Quantity | Expression | Value |
|---|---|---|
| Total inertia | $J_t$ | 0.051 kg·m² |
| Reduced inertia | $J_r$ | 9.80e-4 kg·m² |
| Body rate per unit wheel speed | $J_w/J_t$ | 0.0196 |
| Wheel speed per unit body rate | $J_t/J_w$ | 51 |
| Wheel speed to slew at 1 rad/s | $51 \times 1$ rad/s | 51 rad/s = 487 rpm |
| Wheel speed to slew at 20 °/s | $51 \times 0.349$ rad/s | 17.8 rad/s = 170 rpm |
| Largest body rate the wheel can absorb | $(J_w/J_t)\,\Omega_{max}$ | 6.2 rad/s |
| Wheel drift under a steady 0.002 N·m torque | $\tau/J_w$ | 2 rad/s², limit reached in about 157 s |
| Friction time constant | $J_t/b$ | 25.5 s |

## 7. Assumptions and where they break

| Assumption | Breaks when |
|---|---|
| Single axis: roll and pitch are ignored | The cube swings in the gimbal. The spinning wheel then couples roll and pitch gyroscopically, and the gyro z axis is no longer the yaw axis. |
| Rigid body, wheel axis aligned with the yaw axis | Loose mounts or a tilted wheel. A tilted wheel puts part of its torque into roll and pitch. |
| Friction is viscous + Coulomb + stiction | Bearings with position-dependent drag, or wires touching the frame. Shows up as different behaviour at different angles. |
| Gravity torque is one sine term | The centre of mass moves (loose battery) or the gimbal axis tilt changes with roll and pitch. |
| moteus velocity loop is a first-order lag | The torque limit is active. With the nominal values a wheel step larger than about 4 rad/s already saturates 0.2 N·m, so large steps ramp at $\tau_{max}/J_r \approx 204$ rad/s² instead. |
| No command delay | The control loop bandwidth approaches $1/T_d$. Not expected below 10 rad/s. |
| Parameters constant | Battery voltage sag lowers the available torque at high wheel speed. |

## 8. Open points

1. **Torque or velocity command.** `v5/hal_cubo.h` commands torque (`hal_moteus_comandar_binario`). The WBS plan and the draft `control.h` command wheel velocity. The plant is the same; only section 3.3 changes. Decide with Edson and Ana, then delete the unused mode from the simulator.
2. **The two simulators use different numbers.** `v5/teste.c` has $J_b$ = 0.05, $J_w$ = 0.001, $\tau_{max}$ = 0.2. The Ground Station demo (`simulation/plant.py`) has $J_b$ = 0.030, $J_w$ = 0.0004, viscous damping 0.0003, no Coulomb friction, $\tau_{max}$ = 0.035. Both should take their values from `docs/plant_params.h`.
3. **$J_b$ = 0.05 kg·m² looks high.** A uniform 1 kg, 10 cm cube has $m a^2/6 \approx 0.0017$ kg·m², and a 2 kg, 20 cm cube about 0.013. Unless the gimbal ring turns with the cube, expect a smaller value. It changes the wheel speed needed for every manoeuvre, so measure it first.
4. **Level 1 criterion (3000 rpm in under 0.5 s, still TBC).** With the nominal $J_w$ this needs about $J_r \times 314 / 0.5 \approx 0.62$ N·m, three times the current 0.2 N·m limit. At 0.2 N·m the run-up takes about 1.5 s. It also spins a free cube to about 6 rad/s, so this test must be done with the cube held.

## 9. What the simulator must implement (hand-off to Ana)

Compared with `v5/teste.c`, which already integrates the two equations of section 3 correctly:

| # | Feature | In `v5/teste.c` today | Needed |
|---|---|---|---|
| 1 | Body and wheel momentum exchange | Yes | Keep |
| 2 | Viscous + Coulomb friction | Yes | Keep |
| 3 | Stiction at rest | No (friction is zero below 1e-4 rad/s) | Add, section 3.1 |
| 4 | Gravity torque | No | Add, section 3.2, default $k_g$ = 0 |
| 5 | Velocity-mode actuator with lag | No (torque only) | Add behind a switch, section 3.3 |
| 6 | Torque and wheel-speed limits inside the plant | In the controller only | Add to the plant too, so a faulty controller cannot exceed them |
| 7 | Parameters from one shared file | `#define` in `teste.c` | Include `docs/plant_params.h` |

### Acceptance checks

These four runs prove the simulator matches this note. Use the nominal parameters.

| # | Run | Expected |
|---|---|---|
| A1 | All friction and gravity off, any wheel command | $J_t\,\omega + J_w\,\Omega$ stays constant to within 1e-6 |
| A2 | Friction off, wheel speed raised by 20 rad/s | Body rate settles at $-0.392$ rad/s |
| A3 | Friction on, body released at 1 rad/s, motor torque zero | Body stops after 17.3 s ($\frac{J_b}{b}\ln(1 + b\,\omega_0/\tau_c)$) |
| A4 | Body held at rest by the controller against a constant 0.002 N·m external torque | Wheel accelerates at 2 rad/s² |
