# 左臂 clearance 调参记录 2026-08-08

## 当前稳定版本

- 脚本：`scripts/left_arm_v2_5_3.py`
- Git commit：`ec19837 Tune left arm clearance motion v2.5.3`
- 主要命令：

```bash
sudo python3 left_arm_v2_5_3.py clearance --deadband-deg 0.5 --max-delta-deg 120 --step-deg 5 --execute
sudo python3 left_arm_v2_5_3.py home --deadband-deg 0.5 --max-delta-deg 120 --execute
```

## 关节电机与 CAN ID

v2.5.3 当前左臂关节配置：

| 关节 | 电机型号 | slave_id | master_id |
| --- | --- | ---: | ---: |
| `shoulder_front` | `DM4340` | `0x1E` | `0x0E` |
| `shoulder_side` | `DM4340` | `0x1F` | `0x0F` |
| `shoulder_rotate` | `DM4310` | `0x20` | `0x10` |
| `elbow` | `DM4340` | `0x21` | `0x11` |
| `arm_roll` | `DM4310` | `0x22` | `0x12` |
| `wrist_side` | `DM4310` | `0x2A` | `0x1A` |
| `wrist` | `DM4310` | `0x29` | `0x19` |

本次硬件变更记录：

- `shoulder_rotate` 已由 4340 改为 4310。
- `arm_roll` 已由 4340 改为 4310。

## v2.5.3 参数表

### clearance 六联动主阶段

当前 clearance 主阶段为六联动，最大用时 10 秒：

| 关节 | kp | kd | 说明 |
| --- | ---: | ---: | --- |
| `shoulder_front` | `124.0` | `4.8` | 主承重关节，启用速度前馈和 `tau=0.6` 前馈 |
| `shoulder_side` | `60.0` | `5.0` | 六联动主阶段 |
| `elbow` | `65.0` | `6.5` | 六联动主阶段；接近物理极限，不继续加 |
| `arm_roll` | `18.0` | `6.0` | 六联动主阶段，进度窗口 `0.65 -> 1.0` |
| `wrist_side` | `22.0` | `1.4` | 六联动主阶段 |
| `wrist` | `24.0` | `4.4` | 六联动主阶段，进度窗口 `0.35 -> 1.0` |

主阶段轨迹参数：

```python
COUPLED_CLEARANCE_MAX_SECONDS = 10.0
COUPLED_CLEARANCE_CONTROL_DT = 0.01
COUPLED_CLEARANCE_TRAJECTORY = "blend_smootherstep"
COUPLED_CLEARANCE_LINEAR_BLEND = 0.35
COUPLED_CLEARANCE_PROGRESS_WINDOWS = {
    "arm_roll": (0.65, 1.0),
    "wrist": (0.35, 1.0),
}
COUPLED_CLEARANCE_PRE_WINDOW_GAINS = {
    "wrist": {"kp": 6.0, "kd": 0.8},
}
COUPLED_CLEARANCE_VELOCITY_FF_JOINTS = {"shoulder_front"}
COUPLED_CLEARANCE_MOVE_TAU_FF = {"shoulder_front": 0.6}
```

### clearance hold 与 fine 阶段

| 关节 | hold kp | hold kd | 说明 |
| --- | ---: | ---: | --- |
| `wrist` | `18.0` | `1.4` | 常规 clearance hold |
| `wrist_side` | `18.0` | `1.4` | 常规 clearance hold |
| `arm_roll` | `26.0` | `5.0` | 常规 clearance hold |
| `elbow` | `92.0` | `6.2` | clearance 强 hold |
| `shoulder_front` | `125.0` | `6.5` | clearance 强 hold，压低下坠 |
| `shoulder_side` | `70.0` | `5.0` | clearance hold |
| `shoulder_rotate` | `45.0` | `4.5` | 常规 clearance hold；主 clearance 中主动移动被跳过 |

特殊 compliant hold：

```python
CLEARANCE_COMPLIANT_HOLD_GAINS = {
    "arm_roll": {"kp": 6.0, "kd": 6.0},
}
CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE = {
    "wrist_side": {"arm_roll"},
    "wrist": {"arm_roll"},
}
CLEARANCE_JOINT_HOLD_TAU = {
    "shoulder_front": 2.5,
}
```

`wrist` fine 参数：

```python
COUPLED_CLEARANCE_WRIST_FINE_GAINS = {
    "kp": 26.0,
    "kd": 4.7,
    "seconds": 6.0,
}
CLEARANCE_WRIST_FINE_DEADBAND_DEG = 1.0
CLEARANCE_WRIST_FINE_MAX_BIAS_DEG = 1.5
CLEARANCE_WRIST_FINE_SETTLE_SECONDS = 1.0
```

### home 参数

home 主顺序：

```text
elbow -> shoulder_front -> shoulder_side -> shoulder_rotate -> wrist -> wrist_side -> arm_roll
```

home 单关节默认移动参数：

| 关节 | kp | kd | seconds |
| --- | ---: | ---: | ---: |
| `wrist` | `22.0` | `1.4` | `5.0` |
| `wrist_side` | `16.0` | `2.2` | `6.0` |
| `elbow` | `70.0` | `3.0` | `6.0` |
| `arm_roll` | `32.0` | `4.0` | `8.0` |
| `shoulder_front` | `90.0` | `4.0` | `6.0` |
| `shoulder_side` | `70.0` | `3.0` | `6.0` |
| `shoulder_rotate` | `24.0` | `4.5` | `8.0` |

home 四联动参数：

| 关节 | kp | kd |
| --- | ---: | ---: |
| `elbow` | `65.0` | `6.5` |
| `shoulder_front` | `75.0` | `7.0` |
| `shoulder_side` | `60.0` | `5.0` |
| `arm_roll` | `18.0` | `6.0` |

home hold 参数：

| 关节 | hold kp | hold kd |
| --- | ---: | ---: |
| `wrist` | `18.0` | `1.4` |
| `wrist_side` | `18.0` | `1.4` |
| `arm_roll` | `32.0` | `3.0` |
| `elbow` | `65.0` | `4.5` |
| `shoulder_front` | `75.0` | `5.0` |
| `shoulder_side` | `55.0` | `3.5` |
| `shoulder_rotate` | `24.0` | `4.0` |

home compliant hold：

```python
HOME_COMPLIANT_HOLD_GAINS = {
    "arm_roll": {"kp": 4.0, "kd": 6.0},
    "shoulder_rotate": {"kp": 6.0, "kd": 5.0},
}
HOME_COMPLIANT_HOLDS_BY_ACTIVE = {
    "wrist": {"arm_roll", "shoulder_rotate"},
    "wrist_side": {"arm_roll", "shoulder_rotate"},
    "shoulder_rotate": {"arm_roll"},
}
```

## 结论

当前 `clearance` 六联动在 `10s` 下测试反馈为“很好、很顺”，作为当前稳定工作速度记录。

关键稳定参数：

```python
COUPLED_CLEARANCE_MAX_SECONDS = 10.0
COUPLED_CLEARANCE_CONTROL_DT = 0.01
COUPLED_CLEARANCE_TRAJECTORY = "blend_smootherstep"
COUPLED_CLEARANCE_LINEAR_BLEND = 0.35
```

`shoulder_front` 主联动参数：

```python
COUPLED_CLEARANCE_MOVE_GAINS["shoulder_front"] = {"kp": 124.0, "kd": 4.8}
COUPLED_CLEARANCE_MOVE_TAU_FF["shoulder_front"] = 0.6
COUPLED_CLEARANCE_VELOCITY_FF_JOINTS = {"shoulder_front"}
```

`shoulder_front` 后段 hold/fine 参数：

```python
CLEARANCE_HOLD_GAINS["shoulder_front"] = {"kp": 125.0, "kd": 6.5}
```

## clearance 联动结构

v2.5.3 的 clearance 主阶段为真实六联动：

```text
shoulder_front
shoulder_side
elbow
arm_roll
wrist_side
wrist
```

`shoulder_rotate` 在 clearance 中不主动移动，只作为 hold joint：

```python
CLEARANCE_JOINT_DEADBANDS_DEG["shoulder_rotate"] = 180.0
```

这是因为 earlier tests 里 `shoulder_rotate` 的小幅尾段动作会触发抖动；跳过主动移动后 clearance 更稳定。

## 轨迹调参记录

纯 `smootherstep`：

- 中段较平顺。
- 开始和结束仍有低速 stick-slip 感。

混合轨迹：

```python
COUPLED_CLEARANCE_TRAJECTORY = "blend_smootherstep"
```

`linear_blend=0.15`：

- 开始和结束有改善。

`linear_blend=0.25`：

- 开始阶段明显更顺。
- 结束仍略卡。

`linear_blend=0.35`：

- 当前最佳体感。
- 开始和结束都更顺，且没有带来抖动。

`linear_blend=0.45`：

- 不如 `0.35` 顺，已回退。

纯 `linear`：

- 不如混合轨迹顺。
- 虽然消除了低速区，但起停速度突变导致整体质感变差，已回退。

## 速度调参记录

在 `linear_blend=0.35`、`control_dt=0.01`、`shoulder_front kp/kd=124/4.8` 的条件下逐步提速：

```text
28s -> 稳定
26s -> 稳定
24s -> 稳定
22s -> 很平稳
20s -> 很平稳
18s -> 平稳
16s -> 平稳
14s -> 平稳
12s -> 平稳
10s -> 很好，很顺
```

当前记录 10 秒为稳定值。

## 数据观察

典型 10 秒测试结果：

```text
shoulder_front target          1.995689
shoulder_front coupled reached ~1.956
shoulder_front final pos       ~1.982
shoulder_front final error     ~0.77 deg
shoulder_front tau             ~4.08
```

`elbow` 多次测试显示接近物理极限，后续不要为了 clearance 末端误差继续调高 elbow 参数。

## 注意事项

- 当前稳定性依赖 wrist 后段 fine 动作。
- `wrist` 主联动参数保持：

```python
COUPLED_CLEARANCE_MOVE_GAINS["wrist"] = {"kp": 24.0, "kd": 4.4}
COUPLED_CLEARANCE_WRIST_FINE_GAINS = {"kp": 26.0, "kd": 4.7, "seconds": 6.0}
```

- `arm_roll` 已换 4310，后段 wrist fine 中使用较软 hold，避免 4310 roll 参与抖动：

```python
CLEARANCE_COMPLIANT_HOLD_GAINS["arm_roll"] = {"kp": 6.0, "kd": 6.0}
```

## 下一步建议

- 先用 `left_arm_v2_5_3.py` 作为 clearance 稳定版本。
- 下一次如继续优化，优先验证 `home` 是否也需要类似轨迹优化。
- 若 10 秒在多次冷启动/重启后仍稳定，可考虑把 v2.5.3 固化为新的默认左臂 bring-up 脚本。
