# Claw Calibration

Measured on Orin with the DM4310 claw.

```text
Motor: DM4310
Master ID: 0x18
Slave ID: 0x28
Maximum open/home position: -5.9451056 rad
Maximum close position:      7.5548944 rad
```

Training/control abstraction:

```text
gripper = 0.0  closed
gripper = 1.0  open
```

Mapping:

```python
CLAW_OPEN_RAD = -5.9451056
CLAW_CLOSE_RAD = 7.5548944

def gripper_to_rad(g):
    g = max(0.0, min(1.0, g))
    return CLAW_CLOSE_RAD * (1.0 - g) + CLAW_OPEN_RAD * g
```

Recommended repeated-test range:

```text
gripper = 0.05 to 0.95
```

The exact endpoints are calibrated limits and should be used briefly.
