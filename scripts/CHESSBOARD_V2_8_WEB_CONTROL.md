# HiveRobot v2.8 综合控制台

入口脚本：`chessboard_vision_v2_8_web_control.py`，默认端口 8098。

## 保留的功能和数据

v2.8 导入现有 v2.7 棋盘控制模块和 v2.6 机械臂控制模块，复用全部原有处理方法，
不修改这两个源文件。请保留它们和原有依赖文件，不要只复制 v2.8 到空目录。

- 棋盘页：实时图像、网格、标定、Inspect、空棋盘基线、YOLO Detect、采样、
  train/val/test、标注审核、训练/停止、模型选择、预测图、高度采样与模型验证。
- 机械臂页：原有动作按钮、关节微调、夹爪、低力矩、急停、动作保存/列表/回放、
  RGB/深度画面、目标检测、运行状态和命令日志。
- 共享一次 ROS 图像订阅，把同一帧交给两套状态；保留深度时间戳和内参信息。
- 默认沿用 v2.7 的 calibration、overlay-config、empty-board baseline、
  datasets/chess_pieces_yolo、相邻 chess_piece_heights 和 runs/chess_piece_yolo。
- 默认沿用 v2.6 的机械臂脚本、saved_moves 和 web_runs 日志路径。回放入口改为
  `left_arm_v2_8_move_library.py`，捕获和列表格式不变。
- 启动不会训练 YOLO，也不会搬迁、删除样本或清理旧模型。

自定义数据路径仍需使用原来的命令行参数，例如 `--yolo-dataset-dir`、
`--calibration`、`--overlay-config`、`--moves-file`。模型选择仍沿用 overlay 配置
和原有模型路径解析规则；v2.8 不强制切换到其他权重。

## Orin 启动

把新脚本放到现有两个控制脚本的同一目录。保留 HP60C 驱动运行，停止旧的
两个 Web 控制进程后，用原来已加载 ROS 环境的终端执行：

```bash
python3 chessboard_vision_v2_8_web_control.py --host 0.0.0.0 --port 8098 --enable-execute
```

浏览器打开 `http://192.168.0.199:8098/`，在两个页签切换。
`/vision` 和 `/arm` 可以单独打开；原有 API 路径保持有效。
不传 `--enable-execute` 时沿用原版只查看/命令预览模式，不执行机械臂动作。
原先使用的相机、ROI、相机到肩部偏移等自定义参数也要继续传入，参数可用 `--help` 查看。

不要同时使用旧 Web 控制器和 v2.8 向同一机械臂下发动作；进程之间没有共享动作锁。
v2.8 保留原有动作执行流程，没有新增识别后自动抓取的行为。

机械臂页 Replay 按钮旁的“多流程”下拉框提供 `none`、`Placement1`、
`Placement_C4`、`White Bishop Placement`、`white knight place_B1` 五项，一次只能选择一个。
`none` 只回放当前 Saved Move；选 Placement1 或 White Bishop Placement 时
Saved Move 须为 `bishop01`，选 Placement_C4 或 white knight place_B1 时
须为 `white_knight_c4`。新流程先执行 Placement_C4 的夹取和专属 Clearance，
随后保持夹爪执行已训练的 `white_knight_place_b1` 到 b1，七关节到位验收后
保持 wrist 执行 Claw Home 张爪。目标 saved move 不存在或缺少关节时会在
起始动作前拒绝执行；放置姿态按原有 pose-local 规则匹配学习锚点，
目标不同且姿态相距较远时使用独立锚点，近似姿态可能继承已有局部补偿。
单独的“Replay 后合拢夹爪”勾选项仍保留；与多流程同时选择时，多流程按
其内置的夹取步骤执行，该勾选项不会在末尾重复合爪。

## 回放补偿继承

v2.6 的自适应力矩、保持力矩和通用目标角补偿文件继续作为共享基础值使用。
v2.8 在其上增加按七个关节目标姿态匹配的局部目标角修正，保存于
`data/left_arm_v2_8_local_target_bias.json`。新姿态立即继承共享基础值；只有当
目标姿态同时满足均方根差不超过 10 度、任一关节差不超过 20 度时，才复用已有
局部修正。相差较大的姿态建立独立锚点，避免一个动作的残差改坏另一个动作。

运行日志中的 `v2.8 pose-local correction` 会显示锚点编号、是否新建、姿态距离及
已有局部样本数。v2.8 的回放不再更新 v2.6 通用目标角补偿，只学习当前姿态附近的
残差；其余 v2.6 通用力矩学习仍按原逻辑工作。

v2.8 的局部目标角学习和 wrist 前到位检查名义阈值均为 0.5 度。进入最后的 wrist 动作
前，系统读取其他六个关节的实际角度。若存在超限关节，本次回放会在 wrist 前停止，
把残差写入当前姿态的局部补偿，且不发额外的短促修正指令。下一次 replay 由原有
20 秒平滑主轨迹应用新补偿。日志中的 `v2.8 pre-wrist verification` 会列出超限关节，
`v2.8 pre-wrist local learning update` 会显示保存的补偿；只有超限列表为空才执行 wrist。
由于实机关节反馈约以 0.0218566 度为一个编码器计数，验收比较时加入
0.011 度的半计数余量，有效放行边界为 0.511 度。这仅用于避免浮点数在量化
边界反复阻塞；名义训练目标仍为 0.5 度。

最后 wrist 运动造成的静态关节偏移使用同一姿态锚点下的局部保持补偿。该补偿叠加在
已继承的 v2.6 保持补偿上；连续同方向误差会继续按受限积分步长学习，不会因为两次
误差数值相同而停在原处。方向反转时自动缩小步长，单关节局部保持补偿限制为 3 度。
每次 wrist 动作仅使用动作完成后的最终关节快照学习一次，避免 elbow 的中间报告与
最终报告重复累计。
一旦某关节已有 v2.8 局部保持补偿，该补偿可直接用于下一次 wrist 轨迹；不会被旧版
保持力矩记录中长期停留的 `improved` 状态阻塞。旧版共享保持偏置仍只在其原有条件
判定就绪后叠加。

上述训练流程适用于通过 v2.8 回放的全部保存动作。wrist 完成后会对七个关节执行统一
的 0.5 度最终验收，并把每个动作的 `validated` 或 `training` 状态写入同一姿态锚点的
`move_validation`。超限时只保存学习结果并以明确错误结束，不追加现场修正运动；返回
Home 后再次回放。七个关节全部合格时才记录为已验证。

独立 Table Clearance 及 Replay 启动的前置 clearance 也使用相同流程。它以新捕获的
clearance 为名义目标，把局部补偿保存到 `data/left_arm_v2_8_clearance_bias.json`，
每次执行后按 0.5 度检查所有实际参与运动的关节。原流程明确以 180 度 deadband 跳过的
`shoulder_rotate` 不计入验收。超限时保存补偿并停止后续 Home/Replay，重新执行
clearance 直到输出状态为 `validated`。
Clearance 命令对 Web 指定的最大单关节角度增加 5 度边界余量，处理当前位置与捕获点
刚好跨过限制的情况；日志同时打印实际 clearance 文件绝对路径、名义目标、请求上限
和有效上限，便于发现文件被覆盖或路径不一致。

## Home 修复

v2.8 默认机械臂入口为 `left_arm_v2_8.py`，其他命令仍交给 v2.6 实现。对于带
`--prehome-clearance` 的 Home，v2.8 会强制让全部选中关节参加 clearance 后的正式
回 Home，解决关节在 clearance 前落入死区、随后被 clearance 移走却未返回的问题。
第一遍完成后会重新打开控制器、读取最新关节角，再执行一次不含 clearance 的有限
校正。日志应依次出现 `v2.8 Home pre-clearance fix` 和
`v2.8 Home verification correction pass`；校正只执行一遍，不会无限循环。
已捕获的 clearance 与 Home 端点若只比 Web 提供的最大角度多不超过 5 度，v2.8 会把
正式 Home 的有效上限扩展到该端点差值再加 0.1 度，避免测量边界误差阻断 Home。
超过这段余量仍会拒绝执行，并报告具体关节。

## 验证范围

本地使用临时目录测试网页、路由、共享帧、数据默认路径和未启用执行时的拒绝逻辑。
真实相机、CUDA/YOLO 训练及机械臂动作仍需在 Orin 现场验证。
回退时停止 v2.8，按原方式启动 v2.7/v2.6，数据无需转换。
