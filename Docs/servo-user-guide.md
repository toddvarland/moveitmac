# MoveIt Servo — User Guide

Real-time jogging of the robot directly from the toolbar, without planning a full trajectory.

---

## Opening the Panel

Click the **Servo** button (waveform icon) in the toolbar. A popover appears over the 3-D view — the viewport stays fully visible while you jog. The button turns **green** when the servo is actively running.

> The Servo button is disabled until a URDF is loaded.

---

## Status Badge

The top-left of the panel shows the current engine state:

| Badge | Meaning |
|---|---|
| **Idle** (grey) | Engine is stopped. No motion is occurring. |
| **Running** (green) | The 100 Hz timer is active; hold a button to move. |
| **Halted — Collision** (red) | A collision was detected mid-jog. Motion has stopped. |

---

## Modes

Use the **Joint / Cartesian** segmented control at the top of the panel to switch modes. You can switch modes freely while the engine is stopped.

### Joint Mode

Each actuated joint in the robot is listed with:

- **Current angle** (degrees) — updates live at 20 Hz while jogging.
- **− / + buttons** — hold to jog that joint in the negative or positive direction.

Multiple joints can be jog simultaneously by holding multiple buttons (e.g., hold `+` on joint 1 and `−` on joint 3 at the same time).

### Cartesian Mode

Six hold-buttons control the end-effector pose in the **world frame**:

| Button | Axis | Effect |
|---|---|---|
| X ↑ / ↓ | Linear X (red) | Move end-effector along world X |
| Y ↑ / ↓ | Linear Y (green) | Move end-effector along world Y |
| Z ↑ / ↓ | Linear Z (blue) | Move end-effector along world Z |
| Rx ↑ / ↓ | Rotate about X (red) | Roll around world X axis |
| Ry ↑ / ↓ | Rotate about Y (green) | Pitch around world Y axis |
| Rz ↑ / ↓ | Rotate about Z (blue) | Yaw around world Z axis |

Cartesian jogging uses the **active planning group's tip link** as the end-effector. If no planning group is set, the last joint's child link is used. A warning is shown at the bottom of the panel in that case — set an active group in the **Setup Assistant** for accurate Cartesian jogging.

---

## Speed

The **Speed** slider in the footer adjusts the velocity multiplier from **0.1×** (slow, precise) to **2.0×** (fast). It can be changed at any time, including while the engine is running.

Default reference speeds (at 1.0×):

| DOF | Speed |
|---|---|
| Joint | 0.5 rad/s |
| Linear (Cartesian) | 8 cm/s |
| Rotational (Cartesian) | 0.5 rad/s |

---

## Start / Stop / Resume

| Button | Available when | Action |
|---|---|---|
| **Start** | Idle | Starts the 100 Hz servo timer. Hold any jog button to move. |
| **Stop** | Running | Immediately stops the timer and clears all commands. |
| **Resume** | Halted | Restarts the timer after a collision halt. |
| **Stop** | Halted | Clears the halt and returns to Idle without restarting. |

---

## Collision Safety

Every 100 Hz tick, the candidate joint configuration is checked against:

- **Self-collision** — link bounding spheres against each other (ACM-disabled pairs are skipped).
- **Obstacle collision** — all scene obstacles.

If a collision is detected, the engine **halts immediately** before applying the move. The status badge turns red. No joint angles are changed at the moment of halt.

To recover, either:
1. Press **Resume** to restart jogging (useful if the collision was transient due to sensor noise or a tight clearance).
2. Press **Stop** to return to Idle, then manually move the robot away from the collision using the joint sliders before restarting.

---

## Tips

- **Closing the popover** does not stop the servo engine. Click **Stop** first if you want to halt motion when closing the panel.
- Use a slow speed (0.1–0.3×) near obstacles or joint limits.
- Run the **Setup Assistant → Self-Collision** step before using Cartesian mode on a new robot so adjacent link pairs are excluded from collision checking.
- For precise positioning, use Cartesian mode to get close, then fine-tune with the joint sliders or IK gizmo.
