# 左臂 clearance 调参记录 2026-08-08

## 当前稳定版本

- 脚本：`scripts/left_arm_v2_5_3.py`
- Git commit：`ec19837 Tune left arm clearance motion v2.5.3`
- 主要命令：

```bash
sudo python3 left_arm_v2_5_3.py clearance --deadband-deg 0.5 --max-delta-deg 120 --step-deg 5 --execute
sudo python3 left_arm_v2_5_3.py home --deadband-deg 0.5 --max-delta-deg 120 --execute
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
